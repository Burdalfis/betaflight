"""A launcher exiting must not leave its long-lived child behind."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from runtime import start_child,stop_process


class ProcessCleanup(unittest.TestCase):
    def test_exited_launcher_still_cleans_its_child_group(self):
        code = "import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(60)"
        wrapper = 'import subprocess,sys; p=subprocess.Popen([sys.executable,"-c",'+repr(code)+']); print(p.pid,flush=True)'
        parent = start_child([sys.executable,'-c',wrapper],stdout=subprocess.PIPE,text=True)
        child = int(parent.stdout.readline())
        try:
            parent.wait(timeout=3)
            self.assertNotEqual(os.getpgid(0), child)
            stop_process(parent)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                stat=Path(f'/proc/{child}/stat')
                if not stat.exists() or stat.read_text().split(') ',1)[1].startswith('Z '):break
                time.sleep(.05)
            else:self.fail('Launcher child survived group shutdown')
        finally:
            parent.stdout.close()
            try:os.kill(child,signal.SIGKILL)
            except ProcessLookupError:pass


if __name__=='__main__':unittest.main()
