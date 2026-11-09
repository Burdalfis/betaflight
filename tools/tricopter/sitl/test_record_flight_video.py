"""Capture scope and explicit hardware-encoder provenance checks."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from record_flight_video import (FlightVideo, encoder_command, gui_config,
                                 inspect_video, owned_window,configure_capture_camera,keep_capture_visible)
from record_flight_video import camera_matches,hardware_environment,overlay_line_path


class RecorderTests(unittest.TestCase):
    def test_encoder_is_explicit_nvenc_and_capture_is_window_scoped(self):
        command=encoder_command({'window_id':0x123,'width':1280,'height':721},
            Path('/tmp/flight.mp4'),Path('/tmp/overlay.txt'))
        self.assertEqual(command[command.index('-window_id')+1],str(0x123))
        self.assertEqual(command[command.index('-c:v')+1],'h264_nvenc')
        self.assertEqual(command[command.index('-gpu')+1],'0')
        self.assertEqual(command[command.index('-framerate')+1],'60')
        self.assertEqual(command[command.index('-draw_mouse')+1],'0')
        self.assertNotIn('libx264',' '.join(command))
        self.assertNotIn('-grab_x',command)
        self.assertIn('expansion=none',command[command.index('-vf')+1])
        self.assertIn('pad=ceil(iw/2)*2:ceil(ih/2)*2',command[command.index('-vf')+1])

    def test_root_capture_and_invalid_video_settings_are_rejected(self):
        for xid in (0,-1,2**32,'123',True):
            with self.subTest(xid=xid),self.assertRaises(ValueError):
                encoder_command({'window_id':xid,'width':1280,'height':720},'x','overlay')
        for kwargs in ({'gpu':-1},{'gpu':True},{'fps':0}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                encoder_command({'window_id':123,'width':1280,'height':720},'x','overlay',**kwargs)

    def test_only_owned_gazebo_process_group_can_be_selected(self):
        def run(command,**kwargs):
            if '-tree' in command:
                return SimpleNamespace(returncode=0,stdout='  0x101 "Gazebo": ()\n  0x202 "Gazebo": ()\n  0x303 "Private window": ()\n',stderr='')
            xid=command[command.index('-id')+1]
            if command[0]=='xprop':
                return SimpleNamespace(stdout='_NET_WM_PID(CARDINAL) = '+('10' if xid=='0x101' else '20'))
            return SimpleNamespace(stdout='  Width: 1280\n  Height: 720\n')
        with patch('record_flight_video.subprocess.run',side_effect=run), \
             patch('record_flight_video.os.getpgid',side_effect=lambda pid:100 if pid==10 else 200):
            chosen=owned_window(200)
            self.assertEqual(chosen['window_id'],0x202)
            self.assertEqual(chosen['pid'],20)
            self.assertIsNone(owned_window(300))

    def test_gui_configuration_is_child_local_and_uses_ogre1(self):
        plugins=''.join(f'<plugin filename="{name}">'+('<engine>ogre2</engine>' if name=='MinimalScene' else '')+'</plugin>'
            for name in ('MinimalScene','GzSceneManager','InteractiveViewControl','CameraTracking','MarkerManager','WorldControl'))
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'gui.config'
            with patch('record_flight_video.Path.read_text',return_value=plugins):gui_config(path)
            text=path.read_text()
            self.assertIn('<engine>ogre</engine>',text)
            self.assertIn('<width>2560</width>',text)
            self.assertIn('<height>1440</height>',text)
            self.assertNotIn('WorldControl',text)
            self.assertIn('show_again="false"',text)

    def test_encoded_output_must_retain_nvenc_provenance(self):
        for encoder,success in [('Lavc62 h264_nvenc',True),('Lavc62 libx264',False)]:
            result={'streams':[{'tags':{'encoder':encoder}}],'format':{'duration':'1.0'}}
            with patch('record_flight_video.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(result))):
                if success:self.assertEqual(inspect_video('video.mp4'),result)
                else:
                    with self.assertRaises(RuntimeError):inspect_video('video.mp4')

    def test_overlay_is_literal_and_atomically_replaced(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary);video=FlightVideo(directory,'test-partition')
            video.update('Throttle 70%\nYaw +60 deg/s')
            self.assertEqual((directory/'video-overlay.txt').read_text(),'Throttle 70%\nYaw +60 deg/s\n')
            self.assertFalse((directory/'video-overlay.tmp').exists())
            self.assertEqual(overlay_line_path(directory/'video-overlay.txt',0).read_text(),'Throttle 70%')
            self.assertEqual(overlay_line_path(directory/'video-overlay.txt',1).read_text(),'Yaw +60 deg/s')
            self.assertEqual(overlay_line_path(directory/'video-overlay.txt',2).read_text(),'')

    def test_default_is_physical_hardware_rendered_sixty_fps(self):
        with patch.dict('record_flight_video.os.environ',{'DISPLAY':':0'}):
            video=FlightVideo('/tmp','partition')
        self.assertEqual(video.display,':0');self.assertEqual(video.fps,60)
        self.assertEqual((video.width,video.height),(2560,1440))
        with patch('record_flight_video.native_environment',return_value={'LIBGL_ALWAYS_SOFTWARE':'1','WAYLAND_DISPLAY':'wayland-0'}):
            env=hardware_environment('partition',':0')
        self.assertNotIn('LIBGL_ALWAYS_SOFTWARE',env);self.assertNotIn('WAYLAND_DISPLAY',env)
        self.assertEqual(env['__GLX_VENDOR_LIBRARY_NAME'],'nvidia')
        self.assertEqual(env['__NV_PRIME_RENDER_OFFLOAD'],'1')
        self.assertEqual(env['QSG_NO_VSYNC'],'1')

    def test_resolution_can_be_selected_and_hud_scales_with_capture(self):
        video=FlightVideo('/tmp','partition',width=1920,height=1080)
        self.assertEqual((video.width,video.height),(1920,1080))
        for width,height in ((0,1440),(2560,0),(True,1440),(2561,1440)):
            with self.subTest(width=width,height=height),self.assertRaises(ValueError):
                FlightVideo('/tmp','partition',width=width,height=height)
        command=encoder_command({'window_id':123,'width':2560,'height':1440},'x','overlay')
        self.assertEqual(command[command.index('-video_size')+1],'2560x1440')
        self.assertIn('fontsize=40',command[command.index('-vf')+1])
        self.assertIn('x=24:y=24',command[command.index('-vf')+1])

    def test_clean_encoder_eof_before_stop_is_still_a_capture_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary);video=FlightVideo(directory,'test-partition')
            video.encoder=SimpleNamespace(poll=lambda:0,returncode=0)
            with patch('record_flight_video.stop_process'),self.assertRaisesRegex(RuntimeError,'ended before'):
                video.close()
            record=json.loads((directory/'video-record.json').read_text())
            self.assertFalse(record['pass'])
            self.assertIn('ended before',record['error'])

    def test_camera_override_requires_explicit_targets_and_actual_pose(self):
        with tempfile.TemporaryDirectory() as temporary,patch('record_flight_video.subprocess.run',
                return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run:
            status={'trackMode':'FOLLOW_LOOK_AT','followTarget':{'name':'tricopter_camera_heading'},
                'trackTarget':{'name':'tricopter'},'followOffset':{'x':-1.2,'z':.5},'followPgain':1,'trackPgain':1}
            pose={'position':{'x':-1.2,'z':.5}}
            (Path(temporary)/'plant.csv').write_text('x_m,y_m,z_m,yaw_deg\n0,0,0,0\n')
            with patch('record_flight_video.read_gui_message',side_effect=[status,pose]):
                record=configure_capture_camera({},Path(temporary))
            self.assertEqual(record['follow_offset_m'],[-1.2,0,.5])
            self.assertTrue(record['pass'])
            self.assertIn('track_mode: FOLLOW_LOOK_AT',run.call_args.args[0][-1])
            self.assertFalse(camera_matches(status,{'position':{'z':10}},{'x_m':0,'y_m':0,'z_m':0,'yaw_deg':0},'tricopter_camera_heading'))
            self.assertTrue((Path(temporary)/'video-camera.json').exists())

    def test_window_exposure_refuses_another_process_group(self):
        with patch('record_flight_video.os.getpgid',return_value=200),patch('record_flight_video.subprocess.run') as run:
            with self.assertRaisesRegex(RuntimeError,'no longer belongs'):
                keep_capture_visible({'pid':20,'process_group':100,'window_id':123},{})
            run.assert_not_called()

    def test_viewable_owned_window_does_not_require_desktop_focus(self):
        window={'pid':20,'process_group':100,'window_id':123,'width':2560,'height':1440}
        def run(command,**kwargs):
            if command[0]=='xwininfo':
                return SimpleNamespace(returncode=0,stdout='  Map State: IsViewable\n',stderr='')
            if command[0]=='xprop':
                text=('_NET_ACTIVE_WINDOW(WINDOW): window id # 0x456' if '-root' in command
                      else '_NET_WM_STATE(ATOM) = _NET_WM_STATE_ABOVE')
                return SimpleNamespace(returncode=0,stdout=text,stderr='')
            return SimpleNamespace(returncode=int('-ia' in command),stdout='',stderr='activation refused')
        with patch('record_flight_video.os.getpgid',return_value=100), \
             patch('record_flight_video.owned_window',return_value=window.copy()), \
             patch('record_flight_video.time.sleep'), \
             patch('record_flight_video.subprocess.run',side_effect=run):
            record=keep_capture_visible(window,{'DISPLAY':':0'})
        self.assertTrue(record['viewable'])
        self.assertFalse(record['active']);self.assertFalse(record['focused'])
        self.assertFalse(record['focus_required'])
        self.assertEqual(record['actions'][-1]['returncode'],1)

    def test_unmapped_or_wrong_size_owned_window_still_fails(self):
        window={'pid':20,'process_group':100,'window_id':123,'width':2560,'height':1440}
        for actual,message in ((window,'not viewable'),({**window,'width':1920},'geometry')):
            with self.subTest(actual=actual), \
                 patch('record_flight_video.os.getpgid',return_value=100), \
                 patch('record_flight_video.owned_window',return_value=actual), \
                 patch('record_flight_video.time.sleep'), \
                 patch('record_flight_video.subprocess.run',return_value=SimpleNamespace(
                     returncode=0,stdout='  Map State: IsUnMapped\n',stderr='')):
                with self.assertRaisesRegex(RuntimeError,message):
                    keep_capture_visible(window.copy(),{'DISPLAY':':0'})


if __name__=='__main__':unittest.main()
