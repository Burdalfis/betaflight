#!/usr/bin/env python3
"""Generate an explicitly approximate motor/prop electrical-load model.

Only the full-command XING2 2207 1855KV/GF51466 benchmark point calibrates the
propeller coefficients. Motor RPM is inferred from KV, resistance and current,
not measured. The target XING-E Pro1800KV motor is then modeled with its own
published resistance and the operator-supplied 1.2 A at 10 V no-load point. Partial throttle labels
in the manufacturer's table are deliberately not treated as identified ESC duty.
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
G = 9.80665
REFERENCE = {
    'motor': 'iFlight XING2 2207 1855KV', 'propeller': 'Gemfan GF51466',
    'voltage_v': 24.0, 'kv_rpm_v': 1855.0,
    'interphase_resistance_ohm': .050,
    'no_load_current_a_assumed': 1.0, 'friction_smoothing_rad_s': 20.0,
    'full_command_current_a': 35.08, 'full_command_thrust_g': 1685.0,
}
TARGET = {
    'motor': 'iFlight XING-E Pro2207 1800KV', 'propeller': 'Gemfan GF51466',
    'voltage_v': 22.2, 'kv_rpm_v': 1800.0,
    'interphase_resistance_ohm': .0815,
    'no_load_current_a': 1.2, 'no_load_voltage_v': 10.0,
    'no_load_current_status': 'Operator-supplied target no-load measurement/specification',
    'friction_smoothing_rad_s': 20.0,
    'winding_connection': 'delta',
    'delta_branch_resistance_ohm': .12225,
    'equivalent_wye_phase_resistance_ohm': .04075,
    'dc_equivalent_terminal_resistance_ohm': .0815,
    'model_current_limit_a': 35.22,
    'model_brake_current_limit_a': 35.22,
    'model_brake_current_limit_status': 'Unmeasured symmetric default proxy, equal to the assumed drive-current bound; not an identified ESC braking-current limit',
    'model_brake_on_stop': False,
}
# V, I, thrust and throttle labels are transcribed from the primary datasheet.
BENCH = [
    {'reported_throttle_percent': 50, 'voltage_v': 24., 'current_a': 8.23, 'thrust_g': 864., 'electrical_power_w': 197.5},
    {'reported_throttle_percent': 60, 'voltage_v': 24., 'current_a': 11.86, 'thrust_g': 1021., 'electrical_power_w': 284.6},
    {'reported_throttle_percent': 70, 'voltage_v': 24., 'current_a': 17.63, 'thrust_g': 1208., 'electrical_power_w': 423.1},
    {'reported_throttle_percent': 80, 'voltage_v': 24., 'current_a': 22.48, 'thrust_g': 1339., 'electrical_power_w': 539.5},
    {'reported_throttle_percent': 90, 'voltage_v': 24., 'current_a': 27.38, 'thrust_g': 1509., 'electrical_power_w': 657.1},
    {'reported_throttle_percent': 100, 'voltage_v': 24., 'current_a': 35.08, 'thrust_g': 1685., 'electrical_power_w': 841.9},
]
SOURCES = [
    {'url': 'https://onlinedocs.microchip.com/oxy/GUID-AAAF8F97-D139-4A93-A154-AA3C6EE6BB22-en-US-1/GUID-56373C50-12A7-4EBC-8E8A-00AFCBF80C83.html',
     'use': 'Delta branch resistance is 1.5 times terminal resistance; equivalent wye phase resistance is delta branch/3.'},
    {'url': 'https://www.ti.com/document-viewer/lit/html/SSZTBP2/GUID-E16F6FD4-B74F-4AB0-990E-C89D493A2398',
     'use': 'Terminal DC-equivalent trapezoidal drive equation uses line-to-line resistance and back-EMF.'},
    {'url': 'https://iflight-public.oss-cn-hongkong.aliyuncs.com/Support/lnstructions/XING2-MOTOR/XING2-2207-1855KV-EN.pdf',
     'sha256': '274ccdd415685fe86bbdd42e10f61dfb36e315b403d426e707fcf2c94cc13e88',
     'use': 'Reference 1855 KV, 50 mOhm interphase resistance and complete GF51466 static table; no RPM or no-load current published.'},
    {'url': 'https://iflight-public.oss-cn-hongkong.aliyuncs.com/Support/lnstructions/XING-E-MOTOR/XING-E-Pro-2207-1800KV-EN-1.pdf',
     'sha256': '9c9b0e2523209d7f5a8bb7593c3c04b237d1ee7bac9a73c71424bf57b8c47072',
     'use': 'Target 1800 KV, 81.5 mOhm interphase resistance and 35.22 A peak rating; its prop tables are different props. Target delta topology and 1.2 A at 10 V no-load point were supplied by the operator.'},
    {'url': 'https://www.ecalc.ch/calcinclude/help/propcalchelp.htm',
     'use': 'PConst is an empirical power-absorption factor; TConst is thrust fine tuning. Their pair alone does not identify physical kT/kQ.'},
]


def motor_constant(kv):
    """Equivalent SI back-EMF/torque constant using mechanical rad/s."""
    return 60/(2*math.pi*kv)



def no_load_loss_current_asymptote(motor, override_current=None):
    """Calibrate the loss curve to a known no-load test when one is supplied.

    With no aerodynamic load, omega0=(V0-R*I0)/Ke. Choose the asymptotic
    Coulomb-current parameter so its smoothed value at omega0 is exactly I0.
    Reference XING2 has no measured test voltage/current; its 1 A asymptote
    remains an assumption and is not replaced by the target motor's test.
    """
    measured=motor.get('no_load_current_a',motor.get('no_load_current_a_assumed'))
    measured=measured if override_current is None else override_current
    voltage=motor.get('no_load_voltage_v',0.)
    if voltage<=0:return measured
    ke=motor_constant(motor['kv_rpm_v'])
    omega0=(voltage-motor['interphase_resistance_ohm']*measured)/ke
    assert omega0>0
    return measured*(omega0+motor['friction_smoothing_rad_s'])/omega0


def calibrate(i0=1.0):
    r=REFERENCE
    ke=motor_constant(r['kv_rpm_v'])
    omega=(r['voltage_v']-r['interphase_resistance_ohm']*r['full_command_current_a'])/ke
    loss_current=i0*omega/(omega+r['friction_smoothing_rad_s'])
    torque=ke*(r['full_command_current_a']-loss_current)
    return {'reference_omega_rad_s_inferred':omega,
            'reference_rpm_inferred':omega*60/(2*math.pi),
            'reference_prop_torque_nm_inferred':torque,
            'thrust_coefficient_n_per_rad_s_squared':r['full_command_thrust_g']*G/1000/omega**2,
            'torque_coefficient_nm_per_rad_s_squared':torque/omega**2}


def steady(duty, motor, coefficients, i0=None):
    """Solve steady mechanical balance with an independent scalar bisection.

    Positive loaded equilibria draw current; braking current is transient-only
    and does not alter these operating points. Zero duty settles at zero speed
    with either active braking or a disabled coasting drive.
    Load torque is kQ*omega^2. Loss torque is a smoothed Coulomb proxy:
    Ke*I0*omega/(omega+friction_smoothing). No phase inductance is modeled.
    """
    assert math.isfinite(duty) and 0 <= duty <= 1
    v=motor['voltage_v'];r=motor['interphase_resistance_ohm']
    i0=no_load_loss_current_asymptote(motor,i0)
    smooth=motor['friction_smoothing_rad_s']
    ke=motor_constant(motor['kv_rpm_v'])
    kq=coefficients['torque_coefficient_nm_per_rad_s_squared']
    kt=coefficients['thrust_coefficient_n_per_rad_s_squared']
    limit=motor.get('model_current_limit_a',math.inf)

    def balance(omega):
        raw=(duty*v-ke*omega)/r if duty>0 else 0.
        current=min(limit,max(0.,raw))
        loss=ke*i0*omega/(omega+smooth)
        return ke*current-loss-kq*omega*omega

    lower,upper=0.,duty*v/ke
    # The strictly decreasing torque residual brackets the nonnegative root.
    for _ in range(120):
        middle=(lower+upper)/2
        if balance(middle)>0:lower=middle
        else:upper=middle
    omega=(lower+upper)/2
    raw_current=(duty*v-ke*omega)/r if duty>0 else 0.
    current=min(limit,max(0.,raw_current))
    limited=raw_current>limit
    effective_voltage=ke*omega+r*current if current>0 else 0.
    effective_duty=effective_voltage/v
    torque=kq*omega*omega;thrust=kt*omega*omega
    loss_torque=ke*i0*omega/(omega+smooth)
    mech=torque*omega;winding=current*current*r;friction=loss_torque*omega
    power=effective_voltage*current
    assert math.isclose(power,mech+winding+friction,rel_tol=1e-12,abs_tol=1e-12)
    return {'requested_duty':duty,'effective_duty':effective_duty,
            'omega_rad_s':omega,'rpm':omega*60/(2*math.pi),
            'thrust_n':thrust,'thrust_g':thrust/G*1000,
            'equivalent_motor_current_a':current,
            'ideal_bus_current_a':effective_duty*current,
            'prop_torque_nm':torque,'shaft_power_w':mech,
            'input_power_w':power,'winding_loss_w':winding,
            'no_load_loss_w':friction,'current_limited':limited}


def hover(motor,coefficients,total_mass):
    """Ideal equal-thrust level hover; tricopter yaw compensation is excluded."""
    kt=coefficients['thrust_coefficient_n_per_rad_s_squared']
    kq=coefficients['torque_coefficient_nm_per_rad_s_squared']
    ke=motor_constant(motor['kv_rpm_v'])
    thrust=total_mass*G/3
    omega=math.sqrt(thrust/kt)
    current=(kq*omega*omega/ke + no_load_loss_current_asymptote(motor)*
             omega/(omega+motor['friction_smoothing_rad_s']))
    duty=(ke*omega+motor['interphase_resistance_ohm']*current)/motor['voltage_v']
    result=steady(duty,motor,coefficients)
    result['assumed_equal_thrust_per_motor']=True
    result['aircraft_mass_kg']=total_mass
    return result


def calculate():
    coefficients=calibrate(REFERENCE['no_load_current_a_assumed'])
    mass=json.loads((HERE/'mass_properties.json').read_text())['neutral_aircraft']['mass_kg']
    reference_points=[]
    for measured in BENCH:
        row=dict(measured)
        predicted=steady(row['reported_throttle_percent']/100,REFERENCE,coefficients)
        row['prediction_if_label_were_physical_duty']=predicted
        row['thrust_error_percent_if_label_were_duty']=100*(predicted['thrust_g']/row['thrust_g']-1)
        row['bus_current_error_percent_if_label_were_duty']=100*(predicted['ideal_bus_current_a']/row['current_a']-1)
        reference_points.append(row)
    sensitivity={}
    for tag,i0 in [('lower_no_load_current',.6),('higher_no_load_current',1.4)]:
        c=calibrate(i0);target=dict(TARGET)
        sensitivity[tag]={'reference_assumed_no_load_current_a':i0,
                          'target_known_no_load_current_a':TARGET['no_load_current_a'],'calibration':c,
                          'target_full_command':steady(1.,target,c),
                          'ideal_equal_thrust_hover':hover(target,c,mass)}
    return {
        'status':'Approximate coupled motor/prop load model with active-braking proxy; target no-load point and delta resistance known, loaded RPM, ESC duty mapping and braking-current limit not identified',
        'reference_motor':REFERENCE,'target_motor':TARGET,
        'user_prop_constants':{'PConst':1.13,'TConst':.88,
                               'used_as_physical_coefficients':False,
                               'reason':'Empirical calculator factors need their baseline model. Calibrating to the measured GF51466 point already incorporates that prop load; multiplying by these factors again would double-correct it.'},
        'calibration':coefficients,
        'target_motor_constant_v_s_rad_and_nm_a':motor_constant(TARGET['kv_rpm_v']),
        'target_loss_current_asymptote_a':no_load_loss_current_asymptote(TARGET),
        'target_no_load_anchor_check':steady(1.,dict(TARGET,voltage_v=10.),
            dict(coefficients,thrust_coefficient_n_per_rad_s_squared=0.,torque_coefficient_nm_per_rad_s_squared=0.)),
        'target_full_command':steady(1.,TARGET,coefficients),
        'ideal_equal_thrust_hover':hover(TARGET,coefficients,mass),
        'target_duty_sweep':[steady(d,TARGET,coefficients) for d in [0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.]],
        'reference_bench_points':reference_points,'no_load_current_sensitivity':sensitivity,
        'equations':{'back_emf_and_equivalent_current':'I = (u*Vbus - Ke*omega)/R',
                     'enabled_signed_current':'If driveEnabled and (u>0 or brakeOnStop): I = clamp((u*Vbus - Ke*omega)/R, -Ibrake_limit, Idrive_limit)',
                     'disabled_drive_current':'Otherwise I=0; actuator watchdog/invalid drive always coasts, and zero duty coasts by default because brakeOnStop=false',
                     'motor_torque':'Qmotor = Kt*I - Kt*Iloss_asymptote*omega/(omega+20 rad/s)',
                     'known_no_load_anchor':'omega0=(V0-R*I0)/Ke; Iloss_asymptote=I0*(omega0+20)/omega0',
                     'prop_load':'Qprop = kQ*omega^2; T = kT*omega^2',
                     'rotor_acceleration':'J*domega/dt = Qmotor-Qprop',
                     'relative_rotor_momentum':'Hrelative = spin_sign*J*omega*axis',
                     'effective_terminal_voltage':'Veff=Ke*omega+R*I for any nonzero signed current',
                     'ideal_pwm_bus_current':'Ibus = Veff*I/Vbus; negative values represent ideal regeneration into the fixed absorbing bus'},
        'assumptions':[
            'Target delta winding and 81.5 mOhm line-to-line resistance are operator-confirmed. Each physical delta branch is 122.25 mOhm; equivalent wye phase is 40.75 mOhm. The terminal DC-equivalent equation retains 81.5 mOhm with line-to-line back-EMF and terminal current. No branch/phase current is claimed.',
            'Published KV is used as a mechanical speed constant, with KtSI=KeSI=60/(2*pi*KV); ESC timing and speed-dependent losses are not identified.',
            'Target no-load current is operator-supplied 1.2 A at 10 V. The smoothed loss curve is calibrated to pass exactly through that point. Reference XING2 no-load current remains a separate 1 A asymptotic-loss assumption, with sensitivity 0.6–1.4 A; its winding topology is not asserted. The loss speed curve remains a proxy beyond the target anchor.',
            'Only the 100% full-command reference benchmark calibrates kT/kQ. Reference RPM and torque are inferred, not measured.',
            'The different XING2 motor is a GF51466 prop-load calibration proxy for the target XING-E Pro; this is not an exact target-motor identification.',
            'All lower benchmark throttle labels are retained with duty-assumption errors; their mapping to physical PWM duty is unknown and is not fitted.',
            'PConst/TConst are retained as supplied calculator metadata and are not dimensionless physical Ct/Cp or coefficients in T=kT*omega^2.',
            'Target current limit 35.22 A is a chosen model bound based on a motor peak rating, not an actual ESC current regulator specification.',
            'The default braking-current bound is an unmeasured 35.22 A symmetric proxy, equal to the assumed drive-current bound. It does not identify the real ESC braking strength or current limit.',
            'Enabled positive-duty drive permits signed terminal current when rotor back-EMF exceeds commanded voltage. This represents active braking during nonzero throttle reductions. BrakeOnStop is separately disabled by default: disarm/PWM1000 zero duty coasts unless that option is explicitly enabled. Actuator watchdog/invalid drive always disables electrical torque and coasts.',
            'Braking and regeneration affect transient overrun, not the positive loaded steady operating points tabulated here. Signed terminal power is exchanged with an ideal fixed absorbing 22.2 V bus; no battery-voltage evolution, real battery charge acceptance, or phase/PWM braking detail is modeled.',
            'Nominal voltage is fixed 22.2 V; no voltage sag, battery discharge, winding temperature, inductance or ESC switching model is implied.',
            'Ideal equal-thrust hover excludes rear vectoring, aerodynamic torque compensation, body drag and unequal CG lever arms.',
        ],'sources':SOURCES,
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();data=calculate();contents=json.dumps(data,indent=2)+'\n'
    path=HERE/'motor_performance.json'
    if args.check:
        if not path.exists() or path.read_text()!=contents:parser.exit(1,'Regenerate motor_performance.json\n')
    else:path.write_text(contents)
    full=data['target_full_command'];hovering=data['ideal_equal_thrust_hover']
    print(f"Motor load model: full={full['rpm']:.1f}rpm,{full['thrust_n']:.5f}N,{full['equivalent_motor_current_a']:.3f}A; "
          f"ideal hover duty={hovering['requested_duty']:.5f},rpm={hovering['rpm']:.1f}")


if __name__=='__main__':main()
