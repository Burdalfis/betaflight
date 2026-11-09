#!/usr/bin/env python3
"""Exercise the actual Gazebo joint, axis-following wrenches, ADC and watchdog."""
from __future__ import annotations
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from runtime import ARTIFACTS,HERE,RAW_PWM_PACKET,ADC_PACKET,SessionLock,fresh_run_directory,start_child,stop_process
from run import materialize_world


class Commands:
    def __init__(self):
        self.pwm=1509;self.motor_pwm=1400.;self.stop=threading.Event()
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.thread=threading.Thread(target=self.run,daemon=True)
    def run(self):
        while not self.stop.is_set():
            data=RAW_PWM_PACKET.pack(3,*[float(self.motor_pwm)]*3,float(self.pwm),*[0.]*12)
            self.socket.sendto(data,('127.0.0.1',9001));self.stop.wait(.01)
    def start(self):self.thread.start()
    def close(self):
        self.stop.set()
        if self.thread.is_alive():self.thread.join(timeout=2)
        self.socket.close()


class Sensor:
    def __init__(self):
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1',9005));self.socket.setblocking(False)
        self.last=None;self.error=None;self.stop=threading.Event();self.lock=threading.Lock()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        try:
            while not self.stop.is_set():
                readable,_,_=select.select([self.socket],[],[],.05)
                if not readable:continue
                data=self.socket.recv(1024)
                assert len(data)==32,len(data)
                fields=ADC_PACKET.unpack(data)
                assert fields[:3]==(0x31414454,1,32) and fields[-1]==0
                with self.lock:self.last=fields
        except Exception as error:self.error=error
    def latest(self):
        if self.error:raise self.error
        with self.lock:return self.last
    def close(self):
        self.stop.set();self.thread.join(timeout=2);self.socket.close()


def latest_trace(path):
    try:rows=list(csv.DictReader(io.StringIO(path.read_text())))
    except FileNotFoundError:return None
    rows=[row for row in rows if row.get('actuator_valid') is not None]
    return {key:float(value) for key,value in rows[-1].items()} if rows else None


def episode(artifacts,reverse):
    directory=fresh_run_directory(artifacts,'plant','reversed' if reverse else 'normal')
    world=directory/'world.sdf';materialize_world(HERE/'gazebo/worlds/tricopter.sdf',world,reverse)
    tree=ET.parse(world);model=tree.find("./world/model[@name='tricopter']")
    mount=ET.SubElement(model,'joint',{'name':'bench_mount','type':'fixed'})
    ET.SubElement(mount,'parent').text='world';ET.SubElement(mount,'child').text='base_link'
    tree.write(world,encoding='utf-8',xml_declaration=True)
    bridge=model.find("plugin[@name='tricopter::TricopterBridge']")
    max_thrust=[float(v) for v in bridge.findtext('max_thrust_n').split()]
    reaction_ratio=[float(v) for v in bridge.findtext('reaction_ratio_m').split()]
    rotor_j=[float(v) for v in bridge.findtext('rotor_inertia_kgm2').split()]
    coupled=bridge.findtext('motor_model','lag')=='dc_prop'
    expected_thrust=max_thrust[0]*.4**2
    if coupled:
        voltage=float(bridge.findtext('motor_voltage_v'));resistance=float(bridge.findtext('motor_resistance_ohm'))
        constant=60/(2*math.pi*float(bridge.findtext('motor_kv_rpm_v')))
        i0=float(bridge.findtext('motor_no_load_current_a'));limit=float(bridge.findtext('motor_current_limit_a'))
        smooth=float(bridge.findtext('motor_friction_smoothing_rad_s'))
        no_load_voltage=float(bridge.findtext('motor_no_load_voltage_v'))
        no_load_speed=(no_load_voltage-resistance*i0)/constant
        loss_current=i0*(no_load_speed+smooth)/no_load_speed
        kt=float(bridge.findtext('prop_thrust_n_per_rad_s_squared'))
        kq=float(bridge.findtext('prop_torque_nm_per_rad_s_squared'))
        low,high=0.,.4*voltage/constant
        for _ in range(100):
            omega=(low+high)/2
            current=max(0.,min(limit,(.4*voltage-constant*omega)/resistance))
            if constant*(current-loss_current*omega/(omega+smooth))-kq*omega**2>0:low=omega
            else:high=omega
        expected_thrust=kt*((low+high)/2)**2
    plugin=artifacts/'gazebo-build/libTricopterBridge.so'
    if not plugin.exists():raise RuntimeError('Build Gazebo plugin first')
    env=os.environ.copy();env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(plugin.parent)
    env['TRICOPTER_TRACE']=str(directory/'plant.csv');env['GZ_PARTITION']=f'plant-{os.getpid()}'
    commands=Commands();sensor=Sensor();process=None;observations=[]
    try:
        with (directory/'gazebo.log').open('w') as stream:
            process=start_child(['gz','sim','-s','-r','-v','2',str(world)],env=env,cwd=directory,stdout=stream,stderr=subprocess.STDOUT)
        commands.start()
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError(f'Gazebo exited: {directory/"gazebo.log"}')
            row=latest_trace(directory/'plant.csv')
            if row and row['actuator_valid']==1:break
            time.sleep(.1)
        else:raise TimeoutError('No actuator/plant round trip')
        for pwm,expected in [(1509,0),(1700,21.6),(1100,-47.8),(1509,0)]:
            commands.pwm=pwm;time.sleep(.9)
            row=latest_trace(directory/'plant.csv');adc=sensor.latest()
            assert row is not None and adc is not None
            assert abs(row['joint_angle_deg']-expected)<1.0,(pwm,row)
            assert row['pc0']==adc[7]==0 and abs(row['pb0']-adc[6])<=2
            angle=math.radians(row['joint_angle_deg'])
            assert abs(row['rear_axis_world_x'])<.01
            assert abs(row['rear_axis_world_y']-math.sin(angle))<.01
            assert abs(row['rear_axis_world_z']-math.cos(angle))<.01
            # A 1400us command is40% equivalent voltage duty in dc_prop mode;
            # solve its prop-loaded equilibrium rather than assuming40% RPM.
            for i in range(3):
                expected=expected_thrust if coupled else max_thrust[i]*.4**2
                assert abs(row[f'thrust{i}_n']-expected)<.01,row
            signs=[-1,1,-1] if reverse else [1,-1,1]
            for i,sign in enumerate(signs):
                assert abs(row[f'reaction{i}_nm']+sign*reaction_ratio[i]*row[f'thrust{i}_n'])<1e-6
            row['requested_pwm']=pwm;row['adc_wire_pb0']=adc[6];row['adc_wire_pc0']=adc[7]
            observations.append(row)
        # A positive-duty reduction exercises synchronous damping separately
        # from the ESC's Brake On Stop option and the watchdog's open circuit.
        braking_interval=None
        if coupled and float(bridge.findtext('motor_brake_current_limit_a','35.22'))>0:
            commands.motor_pwm=1700.;time.sleep(.4)
            before_brake=latest_trace(directory/'plant.csv')
            commands.motor_pwm=1350.;time.sleep(.35)
            after_brake=latest_trace(directory/'plant.csv')
            braking_interval=(before_brake['sim_time_s'],after_brake['sim_time_s'])
        commands.close();time.sleep(.8)
        coast=latest_trace(directory/'plant.csv')
        assert coast['actuator_valid']==0 and abs(coast['servo_torque_nm'])<1e-9,coast
        if coupled:
            # Drive loss opens the electrical model. It must not teleport RPM
            # to zero; friction and the aerodynamic load dissipate stored energy.
            assert all(abs(coast[f'rotor{i}_terminal_current_a'])<1e-10 for i in range(3)),coast
            time.sleep(1.5)
        row=latest_trace(directory/'plant.csv')
        assert all(row[f'thrust{i}_n']<.001 for i in range(3)),row
        history=list(csv.DictReader(io.StringIO((directory/'plant.csv').read_text())))
        history=[{key:float(value) for key,value in point.items()} for point in history if point.get('actuator_valid') is not None]
        gyro={'peak_rear_precession_nm':max(math.sqrt(sum(point[f'rotor0_precession_{axis}_nm']**2 for axis in 'xyz')) for point in history),
              'peak_rear_spinup_nm':max(math.sqrt(sum(point[f'rotor0_spinup_{axis}_nm']**2 for axis in 'xyz')) for point in history)}
        if rotor_j[0]>0:
            assert gyro['peak_rear_precession_nm']>.001,gyro
            assert gyro['peak_rear_spinup_nm']>.001,gyro
        if coupled:
            errors=[]
            for point in history:
                for i in range(3):
                    current=point[f'rotor{i}_terminal_current_a'];speed=point[f'rotor{i}_omega_rad_s']
                    input_power=point[f'rotor{i}_effective_voltage_v']*current
                    output=constant*current*speed+resistance*current**2
                    errors.append(abs(input_power-output))
            gyro['max_electrical_power_residual_w']=max(errors)
            assert max(errors)<1e-6,gyro
            if braking_interval:
                braking=[point for point in history
                    if braking_interval[0]<point['sim_time_s']<=braking_interval[1]
                    and point['actuator_valid']==1 and point['pwm0_us']==1350]
                assert braking,'No recorded positive-duty reduction'
                gyro['braking_min_terminal_current_a']=min(
                    point[f'rotor{i}_terminal_current_a'] for point in braking for i in range(3))
                gyro['braking_min_battery_current_a']=min(
                    point[f'rotor{i}_battery_current_a'] for point in braking for i in range(3))
                assert gyro['braking_min_terminal_current_a']<0,gyro
                assert gyro['braking_min_battery_current_a']<0,gyro
                gyro['positive_duty_braking_interval_s']=list(braking_interval)
        report={'reverse_spin':reverse,'plateaus':observations,'watchdog_after_input_stop':row,
                'watchdog_coasting_observation':coast,'rotor_diagnostics':gyro,
                'actual_gazebo_physics':True,'body_fixed_joint_mount':True,'flight_test':False,'pass':True}
        (directory/'plant-check.json').write_text(json.dumps(report,indent=2)+'\n')
        print(f'Actual Gazebo plant passed reverse_spin={reverse}: {directory}',flush=True)
        return report,directory
    finally:
        commands.close();sensor.close();stop_process(process)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifacts',type=Path,default=ARTIFACTS)
    args=parser.parse_args();args.artifacts=args.artifacts.resolve()
    with SessionLock():
        normal,first=episode(args.artifacts,False)
        reversed_,second=episode(args.artifacts,True)
    for a,b in zip(normal['plateaus'],reversed_['plateaus']):
        assert abs(a['joint_angle_deg']-b['joint_angle_deg'])<.2
        for i in range(3):
            assert abs(a[f'thrust{i}_n']-b[f'thrust{i}_n'])<.02
            assert abs(a[f'reaction{i}_nm']+b[f'reaction{i}_nm'])<.001
    summary={'pass':True,'normal':str(first),'reversed':str(second),'opposite_reaction_same_thrust_and_joint_map':True}
    (args.artifacts/'plant-check-summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':main()
