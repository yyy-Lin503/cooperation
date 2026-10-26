"""Real Linux child-process cleanup tests. No ROS or physical devices."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'功能包/my_radar_follower'))
from my_radar_follower.gui_processes import (OwnedProcess, process_identity, same_process,
    capture_tree, signal_tree, atomic_json, cleanup_registry)


@unittest.skipUnless(sys.platform.startswith('linux'),'Linux /proc ownership tests')
class ProcessTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)

    def stop(self,child):
        child.refresh();signal_tree(child.identities,signal.SIGKILL)
        try:child.process.wait(timeout=3)
        except subprocess.TimeoutExpired:pass
        child.process.stdout.close()

    def test_force_cleanup_includes_detached_child_and_preserves_external_process(self):
        code=('import subprocess,sys,signal,time; '
              'signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);'
              'child=subprocess.Popen([sys.executable,"-u","-c",'
              '"import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);'
              'signal.signal(signal.SIGTERM,signal.SIG_IGN);print(1,flush=True);time.sleep(60)"],'
              'start_new_session=True,stdout=subprocess.PIPE,text=True);'
              'child.stdout.readline();print(child.pid,flush=True);time.sleep(60)')
        external=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
        child=OwnedProcess('follow',[sys.executable,'-u','-c',code],self.temp.name,lambda *a,**k:None)
        try:
            detached=int(child.process.stdout.readline())
            child.refresh();self.assertIn(detached,[p['pid'] for p in child.identities])
            self.assertNotIn(external.pid,[p['pid'] for p in child.identities])
            now=time.monotonic();child.begin_stop(now)
            child.tick_stop(now+3);child.tick_stop(now+6)
            until=time.monotonic()+3
            while not child.tick_stop(now+6) and time.monotonic()<until:time.sleep(.02)
            self.assertFalse(child.identities)
            self.assertIsNone(external.poll())
        finally:
            self.stop(child);external.terminate();external.wait(timeout=3)

    def test_parent_exit_does_not_hide_its_remaining_process_group(self):
        code=('import subprocess,sys;child=subprocess.Popen([sys.executable,"-c",'
              '"import time;time.sleep(60)"]);print(child.pid,flush=True)')
        child=OwnedProcess('chassis',[sys.executable,'-u','-c',code],self.temp.name,lambda *a,**k:None)
        try:
            orphan=int(child.process.stdout.readline());child.process.wait(timeout=3)
            child.refresh();self.assertIn(orphan,[p['pid'] for p in child.identities])
        finally:self.stop(child)

    def test_registry_cleanup_revokes_lease_and_only_stops_registered_tree(self):
        child=OwnedProcess('follow',[sys.executable,'-c','import time;time.sleep(60)'],self.temp.name,lambda *a,**k:None)
        try:
            folder=Path(self.temp.name);registry=folder/'owned.json'
            atomic_json(registry,dict(manager=dict(pid=99999999,start='0'),owned={'follow':child.identities}))
            cleanup_registry(registry)
            self.assertFalse(json.loads((folder/'context.json').read_text())['permit'])
            self.assertFalse(json.loads(registry.read_text())['owned'])
            self.assertFalse([p for p in child.identities if same_process(p)])
        finally:self.stop(child)

    def test_live_manager_cannot_be_cleaned_and_reused_pid_is_not_signalled(self):
        registry=Path(self.temp.name)/'owned.json'
        atomic_json(registry,dict(manager=process_identity(os.getpid()),owned={}))
        with self.assertRaises(RuntimeError):cleanup_registry(registry)
        wrong=dict(process_identity(os.getpid()),start='not-the-same-process')
        with patch('my_radar_follower.gui_processes.os.kill') as kill:
            signal_tree([wrong],signal.SIGKILL)
            kill.assert_not_called()


if __name__=='__main__':unittest.main()
