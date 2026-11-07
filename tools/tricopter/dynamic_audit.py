"""Independent face/SVD, motor-forward and global-priority checks of C output."""
import numpy as np
from dynamic_reference import box_primary_oracle,box_certificate,lexicographic_oracle,analytic_motor_endpoint


def canonical(values):return np.asarray(values,np.float32).astype(np.float64)


def parameters(controller,dynamic):
    c,d=canonical(controller),canonical(dynamic)
    return {'effectiveness_matrix':c[:16].reshape(4,4),'response_scales':c[21:25],'maximum_effort':c[16:19],
            'effort_per_omega_squared':c[62:65],'horizon_us':int(dynamic[0]),'tau_increasing_s':d[2:5],
            'tau_decreasing_s':d[5:8],'slew_increasing_rpm_s':d[8:11],'slew_decreasing_rpm_s':d[11:14],
            'high_cost_tolerance':d[33],'yaw_cost_tolerance':d[34],'movement_cost_tolerance':d[35]}


def motor_forward(p,rpm,command):
    return np.asarray([analytic_motor_endpoint(rpm[i],command[i],p['horizon_us']/1e6,p['tau_increasing_s'][i],
        p['tau_decreasing_s'][i],p['slew_increasing_rpm_s'][i],p['slew_decreasing_rpm_s'][i]) for i in range(3)])


def validate_priority_every_row(result,dynamic):
    d=canonical(dynamic);summary=result['dynamic_numeric'];state=result['dynamic_integer'];c=result['candidate_numeric'];ci=result['candidate_integer']
    used=state[:,8].astype(bool);available=state[:,5].astype(bool)
    if np.any(used&~available):raise ValueError('Dynamic commands used without a priority-available certificate')
    for row in np.flatnonzero(available):
        count=int(state[row,1]);valid=ci[row,:count,2].astype(bool)
        oracle=lexicographic_oracle(c[row,:count,27],c[row,:count,28],c[row,:count,29],valid,d[33],d[34],d[35])
        selected=int(state[row,3])
        if not oracle['available'] or selected!=int(oracle['selected_indices'][0]):raise ValueError('Published selection violates the global lexicographic tolerance bands')
        if summary[row,2]!=oracle['minimum_high_cost'] or summary[row,3]!=oracle['minimum_yaw_cost']:raise ValueError('Published global minima differ from accepted candidate costs')
        if not np.array_equal(result['numeric'][row,26:29],summary[row,33:36]) or result['numeric'][row,32]!=summary[row,0]:raise ValueError('Issued command differs from authoritative dynamic published command')
    return {'rows':len(used),'priority_available_rows':int(available.sum()),'dynamic_used_rows':int(used.sum()),
            'static_fallback_rows':int((~used).sum()),'global_staged_selection_pass':True}


def snapshot(result,row,time_us,controller,dynamic,identity,full=True):
    p=parameters(controller,dynamic);n=result['numeric'];s=result['dynamic_numeric'][row];u=result['dynamic_integer'][row]
    record={'identity':identity,'time_us':int(time_us),'dynamic_used':bool(u[8]),'priority_available':bool(u[5]),
            'status':int(u[0]),'candidate_count':int(u[1]),'selected_index':int(u[3]),'net_request':s[37:41].tolist(),
            'current_rpm':n[row,69:72].tolist(),'current_pwm_us':float(n[row-1,32] if row else controller[72]),
            'minimum_pwm_us':float(s[41]),'maximum_pwm_us':float(s[42]),'minimum_high_cost':float(s[2]),'minimum_yaw_cost':float(s[3]),
            **{key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in p.items()},
            'reachable_min_effort':s[7:10].tolist(),'reachable_max_effort':s[10:13].tolist(),
            'minimum_command_rpm':s[13:16].tolist(),'maximum_command_rpm':s[16:19].tolist(),'candidates':[]}
    if not record['priority_available']:return record
    current=np.asarray(record['current_rpm']);coef=p['effort_per_omega_squared'];maximum=p['maximum_effort']
    lower=coef*(motor_forward(p,current,s[13:16])*np.pi/30)**2
    upper=np.minimum(maximum,coef*(motor_forward(p,current,s[16:19])*np.pi/30)**2)
    record['reachable_min_effort_exact_oracle']=lower.tolist();record['reachable_max_effort_exact_oracle']=upper.tolist()
    np.testing.assert_allclose(lower,s[7:10],rtol=2e-6,atol=1e-7);np.testing.assert_allclose(upper,s[10:13],rtol=2e-6,atol=1e-7)
    net=s[37:41];b=p['effectiveness_matrix'];scales=p['response_scales'];span=s[42]-s[41]
    for index in range(int(u[1])):
        c=result['candidate_numeric'][row,index];ci=result['candidate_integer'][row,index]
        candidate={'index':index,'valid':bool(ci[2]),'status':int(ci[0]),'evaluated_faces':int(ci[1]),'pwm_us':float(c[0]),
            'horizon_corrected_counts':float(c[1]),'theta_h':float(c[2]),'continuous_optimal_effort':c[3:6].tolist(),
            'published_command_rpm':c[6:9].tolist(),'predicted_rpm':c[9:12].tolist(),'predicted_effort':c[12:15].tolist(),
            'predicted_response':c[19:23].tolist(),'published_high_cost':float(c[27]),'published_yaw_cost':float(c[28]),
            'published_movement_cost':float(c[29]),'high_matrix_condition':float(c[30])}
        if candidate['valid']:
            theta=c[2];a=np.column_stack((b[:,2]*np.cos(theta)+b[:,3]*np.sin(theta),b[:,0],b[:,1]))
            rpm_h=motor_forward(p,current,c[6:9]);effort_h=coef*(rpm_h*np.pi/30)**2
            response=a@effort_h;residual=(response-net)/scales
            high=.5*float(residual[:3]@residual[:3]);yaw=.5*float(residual[3]**2)
            move=float(np.sum(((effort_h-coef*(current*np.pi/30)**2)/maximum)**2)+((c[0]-record['current_pwm_us'])/span)**2)
            np.testing.assert_allclose(rpm_h,c[9:12],rtol=2e-7,atol=1e-5)
            np.testing.assert_allclose(effort_h,c[12:15],rtol=2e-7,atol=1e-7)
            np.testing.assert_allclose([high,yaw,move],c[27:30],rtol=2e-9,atol=2e-10)
            if np.any(c[6:9]<s[13:16]) or np.any(c[6:9]>s[16:19]):raise ValueError('Published motor command is outside its configured bounds')
            primary=box_primary_oracle(a[:3],net[:3],lower,upper,scales[:3])
            certificate=box_certificate(a[:3],net[:3],lower,upper,scales[:3],c[3:6])
            cost_tolerance=1e-6*max(1.,primary['cost'])
            if abs(certificate['cost']-primary['cost'])>cost_tolerance:raise ValueError('C continuous primary solution disagrees with independent 27-face SVD oracle')
            if certificate['global_primary_gap_upper_bound']>1e-5*max(1.,primary['cost']):raise ValueError('C continuous optimum fails independent primary box gap certificate')
            candidate['independent']={'primary_oracle_effort':primary['effort'].tolist(),'primary_oracle_cost':primary['cost'],
                'primary_cost_tolerance':cost_tolerance,'continuous_primary_certificate':certificate,
                'motor_forward_rpm':rpm_h.tolist(),'forward_high_cost':high,'forward_yaw_cost':yaw,'forward_movement_cost':move,
                'maximum_response_publication_error':float(np.max(abs(response-c[19:23])))}
        record['candidates'].append(candidate)
    return record
