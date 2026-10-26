"""Path contracts shared by GUI, bridge, odometry and runtime."""
from pathlib import Path
import os
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/my_radar_follower'))
from my_radar_follower.runtime_support import runtime_paths, project_root


class PathsTest(unittest.TestCase):
    def test_defaults_and_project_discovery(self):
        with patch.dict(os.environ, {}, clear=True):
            paths = runtime_paths()
            self.assertEqual(project_root(), ROOT)
            self.assertEqual(paths['data'], ROOT / 'var')
            self.assertEqual(paths['track'], ROOT / 'var/tracking/txt')
            self.assertEqual(paths['ros_install'], ROOT / 'ros2_ws/install')

    def test_relative_overrides_are_independent_of_cwd(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.dict(os.environ, dict(RADAR_PROJECT_ROOT=str(root),
                    RADAR_DATA_ROOT='data with spaces', RADAR_TRACK_DIR='custom tracks'), clear=True):
                paths = runtime_paths()
                self.assertEqual(paths['data'], root / 'data with spaces')
                self.assertEqual(paths['track'], root / 'custom tracks')

    def test_absolute_track_override_wins(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.dict(os.environ, dict(RADAR_DATA_ROOT=str(root / 'data'),
                    RADAR_TRACK_DIR=str(root / 'tracks')), clear=True):
                self.assertEqual(runtime_paths(root)['track'], root / 'tracks')


if __name__ == '__main__':
    unittest.main()
