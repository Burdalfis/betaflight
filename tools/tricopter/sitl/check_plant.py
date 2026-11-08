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
        self.pwm=1509;self.stop=threading.Event()
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.thread=threading.Thread(target=self.run,daemon=True)
    def run(self):
        while not self.stop.is_set():
            data=RAW_PWM_PACKET.pack(3,1400.,1400.,1400.,float(self.pwm),*[0.]*12)
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
            assert row['thrust0_n']>2.3
            signs=[-1,1,-1] if reverse else [1,-1,1]
            for i,sign in enumerate(signs):
                assert abs(row[f'reaction{i}_nm']+sign*.02*row[f'thrust{i}_n'])<1e-6
            row['requested_pwm']=pwm;row['adc_wire_pb0']=adc[6];row['adc_wire_pc0']=adc[7]
            observations.append(row)
        commands.close();time.sleep(.8)
        row=latest_trace(directory/'plant.csv')
        assert row['actuator_valid']==0 and abs(row['servo_torque_nm'])<1e-9,row
        assert all(row[f'thrust{i}_n']<.001 for i in range(3)),row
        report={'reverse_spin':reverse,'plateaus':observations,'watchdog_after_input_stop':row,
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
