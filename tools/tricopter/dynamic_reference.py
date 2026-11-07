"""Independent NumPy face enumeration and lexicographic validation oracles.

Only synthetic tests/report audits use these routines. Runtime allocation is C.
"""
from itertools import product
import numpy as np


def box_primary_oracle(matrix,target,lower,upper,scales):
    matrix=np.asarray(matrix,np.float64);target=np.asarray(target,np.float64)
    lower,upper,scales=(np.asarray(value,np.float64) for value in (lower,upper,scales))
    if matrix.shape!=(3,3) or target.shape!=(3,) or lower.shape!=(3,) or upper.shape!=(3,) or scales.shape!=(3,):
        raise ValueError('Primary oracle requires three responses and three actuator efforts')
    if not all(np.isfinite(value).all() for value in (matrix,target,lower,upper,scales)) or np.any(lower>upper) or np.any(scales<=0):
        raise ValueError('Finite ordered box and positive response scales required')
    a=matrix/scales[:,None];b=target/scales
    if np.linalg.matrix_rank(a)!=3:raise ValueError('This oracle contract requires a unique full-rank primary objective')
    best=None
    for face in product((-1,0,1),repeat=3):
        face=np.asarray(face);free=face==0;fixed=~free
        point=np.where(face<0,lower,upper).astype(float)
        if free.any():point[free]=np.linalg.lstsq(a[:,free],b-a[:,fixed]@point[fixed],rcond=None)[0]
        tolerance=1e-9*max(1.,float(np.max(abs(np.r_[lower,upper]))))
        if np.any(point<lower-tolerance) or np.any(point>upper+tolerance):continue
        point=np.clip(point,lower,upper);residual=a@point-b;cost=.5*float(residual@residual)
        if best is None or cost<best['cost']:
            gradient=a.T@residual;support=np.where(gradient<0,upper,lower)
            best={'effort':point,'cost':cost,'face':face.tolist(),
                  'objective_gap_upper_bound':max(0.,float(gradient@(point-support))),'gradient':gradient}
    if best is None:raise ValueError('No feasible box face found')
    return best


def box_certificate(matrix,target,lower,upper,scales,effort):
    a=np.asarray(matrix,np.float64)/np.asarray(scales,np.float64)[:,None]
    b=np.asarray(target,np.float64)/scales;effort=np.asarray(effort,np.float64)
    lower,upper=np.asarray(lower,np.float64),np.asarray(upper,np.float64)
    tolerance=2e-6*max(1.,float(np.max(abs(np.r_[lower,upper]))))
    projected=np.clip(effort,lower,upper);correction=float(np.linalg.norm(projected-effort))
    if correction>tolerance:raise ValueError('Published effort fails independent box feasibility')
    residual=a@projected-b;gradient=a.T@residual;support=np.where(gradient<0,upper,lower)
    return {'cost':.5*float(residual@residual),'global_primary_gap_upper_bound':max(0.,float(gradient@(projected-support))),
            'publication_projection_correction':correction,'primal_tolerance':tolerance}


def lexicographic_oracle(high_cost,yaw_cost,movement,valid,high_tolerance,yaw_tolerance,move_tolerance):
    """Three global passes; candidate input order cannot cause epsilon ratchets."""
    high,yaw,move=np.asarray(high_cost),np.asarray(yaw_cost),np.asarray(movement);valid=np.asarray(valid,bool)
    if not valid.any():return {'available':False}
    min_high=float(high[valid].min());primary=valid&(high<=min_high+high_tolerance)
    min_yaw=float(yaw[primary].min());secondary=primary&(yaw<=min_yaw+yaw_tolerance)
    min_move=float(move[secondary].min());eligible=secondary&(move<=min_move+move_tolerance)
    return {'available':True,'minimum_high_cost':min_high,'minimum_yaw_cost':min_yaw,'minimum_move_cost':min_move,
            'primary_eligible':primary,'secondary_eligible':secondary,'final_eligible':eligible,
            'selected_indices':np.flatnonzero(eligible)}


def analytic_motor_endpoint(rpm,target,dt,tau_increasing,tau_decreasing,slew_increasing,slew_decreasing):
    """Independent scalar constant-command lag/slew endpoint test oracle."""
    error=target-rpm;distance=abs(error);tau=tau_increasing if error>=0 else tau_decreasing
    rate=slew_increasing if error>=0 else slew_decreasing
    if tau<=0 or rate<0 or dt<0:raise ValueError('Positive lag, nonnegative slew/duration required')
    if rate==0 or distance<=rate*tau:remaining=distance*np.exp(-dt/tau)
    else:
        linear=(distance-rate*tau)/rate
        remaining=distance-rate*dt if dt<=linear else rate*tau*np.exp(-(dt-linear)/tau)
    return target-np.copysign(remaining,error)


def static_request_classification(matrix,request,maximum,minimum_direction,maximum_direction):
    """Pure controller-B actuator envelope; excludes reaction and dynamics.

    Full-rank B makes its four-response inverse unique. High3 feasibility is
    separately proved by a zero-direction witness or a collective support
    bound; failure to find that witness alone is reported as unknown.
    """
    b=np.asarray(matrix,np.float64);r=np.asarray(request,np.float64);maximum=np.asarray(maximum,np.float64)
    if b.shape!=(4,4) or r.shape!=(4,) or maximum.shape!=(3,) or not all(np.isfinite(v).all() for v in (b,r,maximum)):
        raise ValueError('Finite 4x4 map, four-response request and three effort bounds required')
    tolerance=1e-7*max(1.,float(maximum.max()));angle_tolerance=1e-7
    controls=np.linalg.solve(b,r);tail=float(np.hypot(controls[2],controls[3]))
    direction=float(np.arctan2(controls[3],controls[2])) if tail>tolerance else 0.
    effort=np.asarray([tail,controls[0],controls[1]])
    violations=[]
    for i,name in enumerate(('tail','front_right','front_left')):
        if effort[i]<-tolerance:violations.append(name+'_effort_below_zero')
        if effort[i]>maximum[i]+tolerance:violations.append(name+'_effort_above_maximum')
    if controls[2]<-tolerance:violations.append('tail_vertical_effort_below_zero')
    if tail>tolerance and direction<minimum_direction-angle_tolerance:violations.append('tail_direction_below_minimum')
    if tail>tolerance and direction>maximum_direction+angle_tolerance:violations.append('tail_direction_above_maximum')
    angles=[minimum_direction,maximum_direction]
    stationary=float(np.arctan2(b[0,3],b[0,2]))
    for theta in (stationary,stationary+np.pi,stationary-np.pi):
        if minimum_direction<=theta<=maximum_direction:angles.append(theta)
    values=[b[0,2]*np.cos(theta)+b[0,3]*np.sin(theta) for theta in angles]
    collective_max=float(max(0.,b[0,0])*maximum[1]+max(0.,b[0,1])*maximum[2]+maximum[0]*max(0.,max(values)))
    collective_min=float(min(0.,b[0,0])*maximum[1]+min(0.,b[0,1])*maximum[2]+maximum[0]*min(0.,min(values)))
    high={'status':'unknown','collective_lower_bound':collective_min,'collective_upper_bound':collective_max}
    if r[0]>collective_max+tolerance or r[0]<collective_min-tolerance:
        high.update(status='proven_infeasible',proof='independent_collective_linear_support_bound')
    else:
        a=np.column_stack((b[:3,0],b[:3,1],b[:3,2]))
        if np.linalg.matrix_rank(a)==3:
            witness=np.linalg.solve(a,r[:3]);witness_effort=np.asarray([witness[2],witness[0],witness[1]])
            if np.all(witness_effort>=-tolerance) and np.all(witness_effort<=maximum+tolerance):
                high.update(status='proven_feasible',proof='zero_mathematical_direction_inverse_witness',
                            witness_controls=[float(witness[0]),float(witness[1]),float(witness[2]),0.],
                            witness_effort_logical=witness_effort.tolist(),maximum_high_response_error=float(np.max(abs(a@witness-r[:3]))))
    return {'scope':'Raw requested actuator-only response under the effective declared/controller B; separate reaction set to zero. Not actual total-response feasibility, plant-map mismatch or motor/servo horizon reachability.',
            'controller_matrix':b.tolist(),'requested_actuator_response':r.tolist(),'unique_full4_inverse':True,
            'full4_actuator_feasible':not violations,'required_controls':controls.tolist(),'required_effort_logical':effort.tolist(),
            'required_mathematical_direction_rad':direction,'maximum_effort_logical':maximum.tolist(),
            'minimum_direction_rad':float(minimum_direction),'maximum_direction_rad':float(maximum_direction),
            'effort_constraint_tolerance':tolerance,'direction_tolerance_rad':angle_tolerance,
            'violated_constraints':violations,'maximum_inverse_response_error':float(np.max(abs(b@controls-r))),
            'high3_static_feasibility':high}
