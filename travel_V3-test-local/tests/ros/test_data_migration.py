from pathlib import Path
import importlib.util
import os
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('migrate_data', ROOT / 'scripts/runtime/migrate_data.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MigrationTest(unittest.TestCase):
    def test_pairs_and_preserved_settings(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            root = Path(directory)
            (root / 'bin').mkdir()
            (root / 'bin/capture.bin').write_bytes(b'ADC')
            (root / 'bin/capture.bin.frames.csv').write_text('index')
            (root / 'gui_runtime.ini').write_text('old settings')
            (root / 'var/config').mkdir(parents=True)
            (root / 'var/config/gui_runtime.ini').write_text('new settings')
            self.assertEqual(len(module.migrate(root)), 3)
            self.assertTrue((root / 'bin/capture.bin').exists())
            module.migrate(root, True)
            self.assertEqual((root / 'var/recordings/radar/capture.bin').read_bytes(), b'ADC')
            self.assertEqual((root / 'var/recordings/radar/capture.bin.frames.csv').read_text(), 'index')
            self.assertEqual((root / 'var/config/gui_runtime.ini').read_text(), 'new settings')
            self.assertEqual((root / 'var/config/gui_runtime.legacy.ini').read_text(), 'old settings')
            self.assertFalse((root / 'bin').exists())
            self.assertEqual(module.migrate(root, True), [])

    def test_conflict_prevents_partial_migration(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            root = Path(directory)
            (root / 'bin').mkdir()
            (root / 'bin/a.bin').write_bytes(b'original')
            (root / 'bin/b.bin').write_bytes(b'new')
            (root / 'var/recordings/radar').mkdir(parents=True)
            (root / 'var/recordings/radar/b.bin').write_bytes(b'existing')
            with self.assertRaises(FileExistsError):
                module.migrate(root, True)
            self.assertTrue((root / 'bin/a.bin').exists())
            self.assertFalse((root / 'var/recordings/radar/a.bin').exists())


if __name__ == '__main__':
    unittest.main()
