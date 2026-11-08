#!/usr/bin/env python3
"""Run one stock/modified Betaflight with the same articulated Gazebo tricopter."""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

from runtime import (ARTIFACTS, HERE, ROOT, Msp, SessionLock, fresh_run_directory,
                     provision, start_child, start_firmware, stop_process)


def materialize_world(source: Path, destination: Path, reverse_spin=False):
    tree = ET.parse(source)
    bridge = tree.find("./world/model/plugin[@name='tricopter::TricopterBridge']")
    if bridge is None: raise ValueError('World has no tricopter::TricopterBridge plugin')
    signs = bridge.find('spin_sign')
    if signs is None: raise ValueError('World has no explicit rotor spin signs')
    values = [int(s) for s in signs.text.split()]
    if len(values) != 3 or any(abs(s) != 1 for s in values): raise ValueError('Expected three +/-1 rotor signs')
    if reverse_spin:
        signs.text = ' '.join(str(-s) for s in values)
    # Add the visual-only heading target to older/custom world copies too.
    heading = bridge.find('camera_heading_model')
    if heading is None:
        heading = ET.SubElement(bridge, 'camera_heading_model')
        heading.text = 'tricopter_camera_heading'
    name = (heading.text or '').strip()
    if not name: raise ValueError('camera_heading_model must name a separate camera target')
    world = tree.find('./world')
    anchor = next((model for model in world.findall('model') if model.get('name') == name), None)
    if anchor is None:
        anchor = ET.SubElement(world, 'model', name=name)
        ET.SubElement(anchor, 'link', name='camera_anchor')
    if (len(anchor.findall('link')) != 1 or anchor.findall('.//collision') or
            anchor.findall('.//inertial') or anchor.findall('.//visual') or anchor.findall('joint')):
        raise ValueError('camera_heading_model must be a separate empty model without aircraft geometry or inertia')
    # Upgrade older static proxies: normal model pose commands keep the GUI's
    # scene state synchronized. The detached kinematic link never loads the craft.
    for element, tag, value in [(anchor,'static','false'),
                                (anchor.find('link'),'kinematic','true'),
                                (anchor.find('link'),'gravity','false')]:
        child=element.find(tag)
        if child is None: child=ET.SubElement(element,tag)
        child.text=value
    tree.write(destination, encoding='utf-8', xml_declaration=True)


def build_plugin(artifacts: Path, jobs: int) -> Path:
    directory = artifacts/'gazebo-build'
    directory.mkdir(parents=True, exist_ok=True)
    commands = [
        ['cmake','--fresh','-S',str(HERE/'gazebo'),'-B',str(directory),'-DCMAKE_BUILD_TYPE=RelWithDebInfo',
         '-DCMAKE_FIND_USE_PACKAGE_ROOT_PATH=FALSE','-DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=FALSE'],
        ['cmake','--build',str(directory),'--parallel',str(jobs)],
        ['ctest','--test-dir',str(directory),'--output-on-failure'],
    ]
    with (directory/'build.log').open('w') as output:
        for command in commands:
            result = subprocess.run(command, env=native_environment(), stdout=output, stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError(f'Gazebo plugin build/test failed; inspect {directory/"build.log"}')
    plugin = directory/'libTricopterBridge.so'
    if not plugin.exists(): raise RuntimeError('Build did not produce libTricopterBridge.so')
    sources = [path for path in (HERE/'gazebo').rglob('*')
               if path.is_file() and '__pycache__' not in path.parts]
    manifest = {'plugin':str(plugin),'plugin_sha256':hashlib.sha256(plugin.read_bytes()).hexdigest(),
                'build_commands':commands,'ctest_passed':True,
                'source_sha256':{str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in sorted(sources)}}
    (directory/'build-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return plugin


def native_environment():
    """Use the apt-installed toolchain/dependencies, even from a Conda shell.

    Gazebo's generated messages must match its packaged Protobuf ABI. Mixing
    Conda CONFIG packages with native Gazebo headers is not supported. Keep
    display, input and Gazebo settings, but scope build-path changes to children.
    """
    environment = os.environ.copy()
    environment['PATH'] = '/usr/bin:/bin:/usr/sbin:/sbin'
    for name in ('CMAKE_PREFIX_PATH', 'CMAKE_MODULE_PATH', 'CMAKE_INCLUDE_PATH',
                 'CMAKE_LIBRARY_PATH', 'CMAKE_PROGRAM_PATH', 'CMAKE_TOOLCHAIN_FILE',
                 'PKG_CONFIG_PATH',
                 'PKG_CONFIG_LIBDIR', 'PKG_CONFIG_SYSROOT_DIR', 'LD_LIBRARY_PATH',
                 'LIBRARY_PATH', 'CPATH', 'C_INCLUDE_PATH', 'CPLUS_INCLUDE_PATH',
                 'CC', 'CXX', 'CFLAGS', 'CXXFLAGS', 'CPPFLAGS', 'LDFLAGS',
                 'PYTHONPATH', 'PYTHONHOME'):
        environment.pop(name, None)
    return environment


def verified_binary(artifacts: Path, variant: str) -> Path:
    directory = artifacts/variant
    manifest = directory/'build-manifest.json'
    if not manifest.exists():
        raise RuntimeError(f'Build {variant} first: python3 tools/tricopter/sitl/build.py --variant {variant}')
    metadata = json.loads(manifest.read_text())
    binary = Path(metadata['binary'])
    if not binary.exists() or hashlib.sha256(binary.read_bytes()).hexdigest() != metadata['binary_sha256']:
        raise RuntimeError(f'Binary differs from its build manifest: {binary}')
    source = Path(metadata['source'])
    for name, expected in metadata['source_sha256'].items():
        path = source/name
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Source changed after build: {path}; rebuild {variant}')
    return binary


def wait_for_plant(directory: Path, firmware, gazebo, client, variant, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if firmware.poll() is not None or gazebo.poll() is not None:
            raise RuntimeError(f'Simulation exited during startup; inspect logs in {directory}')
        try:
            rows = list(csv.DictReader(io.StringIO((directory/'plant.csv').read_text())))
            valid = any(row.get('actuator_valid') == '1' and float(row['sim_time_s']) > 0 for row in rows)
            if valid:
                if variant == 'stock': return
                debug = struct.unpack('<8h', client.request(254))
                if debug[6] & 3 == 3: return  # config and measured feedback valid
        except (OSError, ValueError, csv.Error):
            pass  # The first trace write may still be buffered or incomplete.
        time.sleep(.1)
    raise RuntimeError(f'No valid actuator/FDM/ADC plant round trip within {timeout}s; inspect {directory/"gazebo.log"}')


def frame_viewer(environment, directory: Path, follow=True):
    """Use a centered, level-horizon chase view without camera spring lag."""
    heading = None
    if follow:
        heading = ET.parse(directory/'world.sdf').findtext(
            "./world/model/plugin[@name='tricopter::TricopterBridge']/camera_heading_model")
        if not heading: raise ValueError('Materialized world has no camera heading target')
    deadline = time.monotonic() + 15
    with (directory/'viewer-setup.log').open('w') as log:
        while time.monotonic() < deadline:
            result = subprocess.run(['gz','service','-s','/gui/move_to',
                '--reqtype','gz.msgs.StringMsg','--reptype','gz.msgs.Boolean',
                '--timeout','500','--req','data:"tricopter"'],env=environment,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=2)
            log.write(result.stdout);log.flush()
            if result.returncode == 0 and 'data: true' in result.stdout:
                time.sleep(.7)
                if follow:
                    # The visual-only proxy follows position and yaw, so the
                    # camera does not swing around the aircraft as it banks.
                    request = (f'track_mode: FOLLOW_LOOK_AT follow_target:{{name:{json.dumps(heading)}}} '
                               'track_target:{name:"tricopter"} follow_offset:{x:-2.4 y:0 z:1.0} '
                               'track_offset:{x:0 y:0 z:0} follow_pgain:1 track_pgain:1')
                    result = subprocess.run(['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack',
                        '-p',request],env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=3)
                    if result.returncode: raise RuntimeError('Camera follow setup failed')
                (directory/'viewer-setup.json').write_text(json.dumps({
                    'framed_model':'tricopter','follow':follow,
                    'follow_target':heading,
                    'follow_offset_m':[-2.4,0,1.0] if follow else None,
                    'follow_pgain':1 if follow else None,'track_pgain':1 if follow else None,
                    'horizon_level':follow})+'\n')
                return
            time.sleep(.1)
    raise RuntimeError(f'Gazebo viewer did not initialize; inspect {directory/"gazebo.log"}')


def execute(args):
    if not shutil.which('gz'):
        raise RuntimeError('Gazebo is not installed. Run: sudo bash tools/tricopter/sitl/install_gazebo_jetty.sh')
    binary = verified_binary(args.artifacts, args.variant)
    plugin = build_plugin(args.artifacts, args.jobs)
    directory = fresh_run_directory(args.artifacts, args.variant)
    world = directory/'world.sdf'
    materialize_world(args.world, world, args.reverse_spin)
    config = provision(binary, directory, args.variant, args.config)
    environment = native_environment()
    environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(plugin.parent) + os.pathsep + environment.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    environment['TRICOPTER_TRACE'] = str(directory/'plant.csv')
    environment['GZ_PARTITION'] = f'tricopter-{os.getpid()}'
    if not args.headless and environment.get('DISPLAY'):
        # Child-only XWayland selection; Ogre1 avoids the observed Ogre2
        # hybrid-GPU EGL initialization hang on this host.
        environment.setdefault('QT_QPA_PLATFORM', 'xcb')
        if environment['QT_QPA_PLATFORM'] == 'xcb': environment.pop('WAYLAND_DISPLAY', None)
    command = ['gz','sim','-r','-v','2']
    if args.headless: command.append('-s')
    else: command += ['--render-engine-gui', args.gui_engine]
    command.append(str(world))
    record = {'variant':args.variant, 'binary':str(binary), 'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
              'plugin_sha256':hashlib.sha256(plugin.read_bytes()).hexdigest(), 'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest(),
              'firmware_config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(), 'command':command,
              'reverse_spin':args.reverse_spin, 'rc_device':args.device, 'rc_profile':str(args.profile) if args.device else None,
              'gui_engine':None if args.headless else args.gui_engine,
              'qt_platform':environment.get('QT_QPA_PLATFORM'), 'gz_partition':environment['GZ_PARTITION']}
    (directory/'run.json').write_text(json.dumps(record,indent=2)+'\n')
    firmware = gazebo = controller = None
    client = None
    try:
        print(f'{args.variant} session: {directory}', flush=True)
        firmware = start_firmware(binary, directory)
        client = Msp.connect(port=5762)
        # The stock UDP worker starts before the virtual IMU devices exist.
        # A successful main-loop MSP reply establishes readiness before FDM.
        client.request(1)
        with (directory/'gazebo.log').open('w') as log:
            gazebo = start_child(command, cwd=directory, env=environment, stdout=log, stderr=subprocess.STDOUT)
        print('Waiting for a valid plant/firmware round trip...', flush=True)
        wait_for_plant(directory, firmware, gazebo, client, args.variant)
        if not args.headless: frame_viewer(environment, directory, not args.no_follow)
        if args.device:
            # python3-evdev is installed for the native interpreter, not Conda.
            rc_command = ['/usr/bin/python3',str(HERE/'rc_input.py'),'--device',args.device,'--profile',str(args.profile)]
            controller = start_child(rc_command, cwd=ROOT, env=environment)
        else:
            print('No RC device selected; model remains disarmed. Use --device NAME_OR_EVENT_PATH for joystick input.',flush=True)
        print('MSP/configurator: TCP 127.0.0.1:5761; Ctrl-C stops this session.',flush=True)
        started = time.monotonic()
        with (directory/'flight.jsonl').open('w') as telemetry:
            while args.duration <= 0 or time.monotonic()-started < args.duration:
                for process,name in [(firmware,'Betaflight'),(gazebo,'Gazebo'),(controller,'controller bridge')]:
                    if process is not None and process.poll() is not None:
                        raise RuntimeError(f'{name} exited ({process.returncode}); inspect session logs')
                status = client.status()
                attitude = struct.unpack('<hhh',client.request(108))
                data = {'wall_elapsed_s':time.monotonic()-started, **status,
                        'roll_deg':attitude[0]/10,'pitch_deg':attitude[1]/10,'yaw_deg':attitude[2]}
                if args.variant=='modified':data['debug']=struct.unpack('<8h',client.request(254))
                telemetry.write(json.dumps(data)+'\n');telemetry.flush()
                print(f"armed={status['armed']} flags=0x{status['arming_disable_flags']:08x} "
                      f"roll={data['roll_deg']:.1f} pitch={data['pitch_deg']:.1f} yaw={data['yaw_deg']:.0f}",flush=True)
                time.sleep(.5)
    finally:
        # Stop input first so the same native RX-loss path is exercised if the
        # firmware is given time to continue during shutdown.
        stop_process(controller)
        if client:client.close()
        stop_process(gazebo)
        stop_process(firmware)
    return directory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant',choices=['stock','modified'],default='modified')
    parser.add_argument('--artifacts',type=Path,default=ARTIFACTS)
    parser.add_argument('--world',type=Path,default=HERE/'gazebo/worlds/tricopter.sdf')
    parser.add_argument('--config',type=Path,help='Additional simulator CLI settings, appended after common settings')
    parser.add_argument('--profile',type=Path,default=HERE/'profiles/xbox.json')
    parser.add_argument('--device',help='Unique HID name or /dev/input/eventN; omit for no RC feed')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--gui-engine',choices=['ogre','ogre2'],default='ogre',help='GUI renderer; Ogre1 is the verified hybrid-GPU fallback')
    parser.add_argument('--no-follow',action='store_true',help='Frame the aircraft without following it')
    parser.add_argument('--duration',type=float,default=0,help='Seconds; zero runs until Ctrl-C')
    parser.add_argument('--reverse-spin',action='store_true',help='Reverse aerodynamic/inertial reaction signs for all rotors')
    parser.add_argument('--jobs',type=int,default=min(8,os.cpu_count() or 1))
    args=parser.parse_args()
    args.artifacts=args.artifacts.resolve();args.world=args.world.resolve();args.profile=args.profile.resolve()
    if args.config:args.config=args.config.resolve()
    if args.duration<0 or args.jobs<1:parser.error('duration must be nonnegative and jobs positive')
    try:
        with SessionLock():execute(args)
    except KeyboardInterrupt:print('\nSITL session stopped.',flush=True)
    except (RuntimeError,ValueError,FileNotFoundError,FileExistsError,TimeoutError,ConnectionError) as error:
        parser.exit(1,f'{error}\n')


if __name__=='__main__':main()
