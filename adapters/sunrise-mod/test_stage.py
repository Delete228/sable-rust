#!/usr/bin/env python3
"""Filesystem staging guards only: no SDK generation, compilation or runtime proof."""
import importlib.util
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
SPEC = importlib.util.spec_from_file_location('sable_adapter_stage', Path(__file__).with_name('stage.py'))
stage = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stage)
REAL_ROOT, REAL_HERE = stage.ROOT, stage.HERE


class StagingGuards(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='sable-staging-guards-', dir=REAL_ROOT / '.local/tmp')
        self.root = Path(self.temporary.name).resolve()
        here = self.root / 'adapters/sunrise-mod'
        shutil.copytree(REAL_HERE, here, ignore=shutil.ignore_patterns('__pycache__'))
        for name in ['sable-bridge', 'sable-core', 'sable-sunrise']:
            source, target = REAL_ROOT / 'crates' / name, self.root / 'crates' / name
            target.mkdir(parents=True)
            shutil.copyfile(source / 'Cargo.toml', target / 'Cargo.toml')
            shutil.copytree(source / 'src', target / 'src')
            for path in source.glob('LICENSE*'):
                shutil.copyfile(path, target / path.name)
            if (source / 'provenance').exists():
                shutil.copytree(source / 'provenance', target / 'provenance')
        stage.ROOT, stage.HERE = self.root, here
        self.project = self.root / '.local/phase-3-continuation/sable_port'
        (self.project / 'src').mkdir(parents=True)
        (self.project / '.sunrise/sdk').mkdir(parents=True)
        # A minimal template-shaped test fixture. It is NOT an installed SDK.
        self.manifest = ('id = "sable_port"\nname = "Staging guard fixture"\nversion = "0.1.0"\n'
                         '[native]\ngame = "26.3"\nsunrise = "' + '0' * 40 + '"\n')
        self.cargo = ('[package]\nname="mod-sable_port"\nversion="0.1.0"\nedition="2024"\n'
                      '[dependencies]\nsunrise-mod-api = { path = ".sunrise/sdk/api/crates/sunrise-mod-api" }\n')
        (self.project / 'sunrise-mod.toml').write_text(self.manifest)
        (self.project / 'Cargo.toml').write_text(self.cargo)
        (self.project / 'src/lib.rs').write_text('// owned template sentinel\n')

    def tearDown(self):
        stage.ROOT, stage.HERE = REAL_ROOT, REAL_HERE
        self.temporary.cleanup()

    def refused_without_copy(self):
        with self.assertRaises(ValueError):
            stage.stage(self.project)
        self.assertEqual((self.project / 'sunrise-mod.toml').read_text(), self.manifest)
        self.assertEqual((self.project / 'Cargo.toml').read_text(), self.cargo)
        self.assertFalse((self.project / 'port').exists())
        self.assertFalse((self.project / 'src/port').exists())
        self.assertFalse((self.project / 'source-ledger.json').exists())
        self.assertFalse((self.project / 'src/frames.rs').exists())

    def test_child_file_symlink_refuses_before_any_copy(self):
        victim = self.root / 'outside.rs'
        victim.write_text('outside-project sentinel\n')
        (self.project / 'src/lib.rs').unlink()
        (self.project / 'src/lib.rs').symlink_to(victim)
        self.refused_without_copy()
        self.assertEqual(victim.read_text(), 'outside-project sentinel\n')

    def test_cargo_symlink_refuses_before_any_copy(self):
        victim = self.root / 'outside.toml'
        victim.write_text(self.cargo)
        (self.project / 'Cargo.toml').unlink()
        (self.project / 'Cargo.toml').symlink_to(victim)
        self.refused_without_copy()
        self.assertEqual(victim.read_text(), self.cargo)

    def test_parent_directory_symlink_refuses_before_any_copy(self):
        victim = self.root / 'outside-src'
        victim.mkdir()
        (victim / 'sentinel').write_text('unchanged')
        shutil.rmtree(self.project / 'src')
        (self.project / 'src').symlink_to(victim, target_is_directory=True)
        self.refused_without_copy()
        self.assertEqual(sorted(path.name for path in victim.iterdir()), ['sentinel'])

    def test_noncoordinated_project_path_refuses(self):
        other = self.project.with_name('different_project')
        other.mkdir()
        with self.assertRaises(ValueError):
            stage.stage(other)
        self.assertEqual(list(other.iterdir()), [])

    def test_readonly_sdk_link_is_preserved_during_real_source_copy(self):
        sdk = self.root / 'readonly-sdk-fixture'
        sdk.mkdir()
        (sdk / 'sentinel').write_text('not a real SDK')
        (self.project / '.sunrise/sdk').rmdir()
        (self.project / '.sunrise/sdk').symlink_to(sdk, target_is_directory=True)
        result = stage.stage(self.project)
        self.assertEqual((sdk / 'sentinel').read_text(), 'not a real SDK')
        self.assertEqual((self.project / 'sunrise-mod.toml').read_text(), self.manifest)
        self.assertTrue((self.project / 'src/frames.rs').is_file())
        self.assertTrue((self.project / 'src/port/sable-core/src/lib.rs').is_file())
        self.assertFalse((self.project / 'port').exists())
        self.assertTrue((self.project / 'NOTICE-sunrise-mod-origin.json').is_file())
        self.assertTrue((self.project / 'LICENSE-sable-core-LICENSE-SABLE.md').is_file())
        self.assertIn('src/port/sable-core', (self.project / 'Cargo.toml').read_text())
        self.assertFalse(result['sdk_verified_by_this_script'])
        self.assertFalse(result['package_built'])
        self.assertFalse(result['runtime_loaded'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
