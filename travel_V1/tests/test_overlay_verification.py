"""Run the real verifier against fake ROS module resolution, with no ROS install."""
from pathlib import Path
import contextlib
import importlib.util
import io
import os
import runpy
import sys
from types import ModuleType
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'功能包/my_radar_follower/my_radar_follower'
def support():
    spec=importlib.util.spec_from_file_location('support_under_test',PKG/'runtime_support.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
class OverlayTest(unittest.TestCase):
    def run_verifier(self,wrong_source=False,wrong_share=False,wrong_version=False):
        helper=support()
        if wrong_version:helper.VERSION='not-this-version'
        modules={name:ModuleType(name) for name in ('my_radar_follower','my_radar_follower.odom_receiver',
            'my_radar_follower.txt_tracker_bridge','ament_index_python','ament_index_python.packages')}
        modules['my_radar_follower'].__path__=[];modules['ament_index_python'].__path__=[]
        modules['my_radar_follower'].__file__=str(PKG/'__init__.py')
        modules['my_radar_follower.runtime_support']=helper
        for name,file in [('odom_receiver','odom_receiver.py'),('txt_tracker_bridge','txt_tracker_bridge.py')]:
            modules['my_radar_follower.'+name].__file__=str(PKG/file)
        if wrong_source:modules['my_radar_follower.txt_tracker_bridge'].__file__=str(ROOT.parent/'V4/功能包/my_radar_follower/my_radar_follower/txt_tracker_bridge.py')
        correct=ROOT/'ros2_ws'/f'{support().VERSION.lower()}-install/my_radar_follower/share/my_radar_follower'
        modules['ament_index_python.packages'].get_package_share_directory=lambda name:str(ROOT.parent/'V3/ros2_ws/install' if wrong_share else correct)
        with patch.dict(sys.modules,modules),patch.dict(os.environ,{'RADAR_PROJECT_ROOT':str(ROOT)}),contextlib.redirect_stdout(io.StringIO()):
            runpy.run_path(str(ROOT/'tools/verify_ros_overlay.py'),run_name='__main__')
    def test_correct_overlay(self):self.run_verifier()
    def test_old_source_refused(self):
        with self.assertRaises(RuntimeError):self.run_verifier(wrong_source=True)
    def test_old_share_refused(self):
        with self.assertRaises(SystemExit):self.run_verifier(wrong_share=True)
    def test_version_marker_mismatch_refused(self):
        with self.assertRaises(RuntimeError):self.run_verifier(wrong_version=True)
if __name__=='__main__':unittest.main()
