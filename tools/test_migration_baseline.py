"""Baseline must detect changes, deletions and new source files."""
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from migration_baseline import differences, fingerprint


class BaselineTests(unittest.TestCase):
    def test_source_and_assets_drift_while_build_outputs_are_ignored(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'assets').mkdir()
            (root/'assets/voice.wav').write_bytes(b'original')
            (root/'config.py').write_text('value = 1')
            snapshot = {'files_sha256': fingerprint(root)[0]}
            (root/'build').mkdir()
            (root/'build/ignored.py').write_text('build output')
            self.assertEqual(differences(root, snapshot), [])
            (root/'config.py').write_text('value = 2')
            (root/'assets/voice.wav').unlink()
            (root/'new.py').write_text('new module')
            self.assertEqual(differences(root, snapshot),
                             ['assets/voice.wav', 'config.py', 'new.py'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
