from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/replay'))
from replay_inputs import basename, resolve_input

class ReplayInputsTest(unittest.TestCase):
    def test_windows_names_and_relative_moved_dataset(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'bin').mkdir();(root/'bin/recording.bin').touch()
            manifest=root/'audit/recordings.json'
            self.assertEqual(basename(r'D:\old\recording.bin'), 'recording.bin')
            self.assertEqual(resolve_input(r'D:\old\recording.bin',manifest,root,'bin'),root/'bin/recording.bin')
            self.assertEqual(resolve_input('../bin/recording.bin',manifest,None,'bin'),root/'bin/recording.bin')
    def test_missing_inputs_are_explicit(self):
        with self.assertRaisesRegex(FileNotFoundError,'--data-root'):
            resolve_input(r'D:\missing\does-not-exist.bin',Path('/tmp/manifest.json'),None,'bin')
