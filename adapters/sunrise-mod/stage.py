#!/usr/bin/env python3
"""Stage exact personal adapter inputs for Sunrise's ordinary native package path.

Populate a fresh project made by `cargo sunrise-mod new --template native`.
Preserve its generated manifest/pins and SDK dependency verbatim. Copy authored
port source only, never game/SDK binaries or resources. This script neither
builds a mod nor claims native runtime loading or upstream acceptance.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_path(path, project):
    """Refuse existing symlink components before any staging mutation."""
    if not path.is_relative_to(project):
        raise ValueError('staging write is outside the coordinated project')
    for component in (path, *path.parents):
        if component.is_symlink():
            raise ValueError('staging write path contains a symlink: ' + str(component))
        if component == ROOT:
            break
        if component != path and component.exists() and not component.is_dir():
            raise ValueError('staging write parent is not a directory: ' + str(component))
    if not path.resolve().is_relative_to(project):
        raise ValueError('staging write resolves outside the coordinated project')
    if path.exists() and not path.is_file():
        raise ValueError('staging file destination is not a regular file: ' + str(path))


def stage(output):
    output = output.absolute()
    expected = ROOT / '.local/phase-3-continuation/sable_port'
    if output != expected:
        raise ValueError('project must be the exact coordinated .local/phase-3-continuation/sable_port path')
    write_path(output / 'Cargo.toml', output)
    write_path(output / 'sunrise-mod.toml', output)
    if not output.is_dir():
        raise ValueError('run normal cargo sunrise-mod new at the coordinated path first')
    if ((output / 'source-ledger.json').exists() or (output / 'port').exists()
            or (output / 'src/port').exists()):
        raise ValueError('project already populated; preserve its prior receipt and use an explicit review update')
    manifest = output / 'sunrise-mod.toml'
    cargo = output / 'Cargo.toml'
    manifest_hash = digest(manifest)
    manifest_text = manifest.read_text()
    if re.findall(r'^id\s*=\s*"([a-z0-9_]+)"\s*$', manifest_text, re.MULTILINE) != ['sable_port']:
        raise ValueError('generate the project at the coordinated sable_port path so its id matches the authored namespaces')
    pins = re.findall(r'^sunrise\s*=\s*"([0-9a-f]{40})"\s*$', manifest_text, re.MULTILINE)
    if len(pins) != 1 or '[native]' not in manifest_text or not (output / '.sunrise/sdk').is_dir():
        raise ValueError('run the normal cargo sunrise-mod new command with the experimental local SDK first')
    build = pins[0]
    original_cargo = cargo.read_text()
    api_line = 'sunrise-mod-api = { path = ".sunrise/sdk/api/crates/sunrise-mod-api" }'
    if api_line not in original_cargo or original_cargo.count('[dependencies]') != 1:
        raise ValueError('expected the unchanged normal native template SDK dependency')
    # Validate the COMPLETE destination set before copying even its first file.
    # Read-only SDK links are intentionally not in this write plan.
    targets = [cargo, output / 'source-ledger.json']
    targets.extend(output / path.relative_to(HERE) for path in (HERE / 'src').rglob('*.rs'))
    targets.extend(output / path.name for path in HERE.glob('LICENSE*'))
    targets.extend(output / ('NOTICE-sunrise-mod-' + path.relative_to(HERE / 'provenance').as_posix().replace('/', '__'))
                   for path in (HERE / 'provenance').rglob('*.json'))
    for name in ['sable-bridge', 'sable-core', 'sable-sunrise']:
        source = ROOT / 'crates' / name
        target = output / 'src/port' / name
        targets.append(target / 'Cargo.toml')
        targets.extend(target / path.relative_to(source) for path in (source / 'src').rglob('*.rs')
                       if not (name == 'sable-core' and path.name == 'physics.rs'))
        targets.extend(output / ('LICENSE-' + name + '-' + path.name) for path in source.glob('LICENSE*'))
        targets.extend(output / ('NOTICE-' + name + '-' + path.relative_to(source / 'provenance').as_posix().replace('/', '__'))
                       for path in (source / 'provenance').rglob('*.json'))
    for target in targets:
        write_path(target, output)
    ledger = []

    def copy(source, target):
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        ledger.append({'source': str(source.relative_to(ROOT)), 'source_sha256': digest(source),
                       'staged': str(target.relative_to(output)), 'staged_sha256': digest(target),
                       'transformation': 'byte-identical'})

    for source in sorted((HERE / 'src').rglob('*.rs')):
        copy(source, output / source.relative_to(HERE))
    for source in sorted(HERE.glob('LICENSE*')):
        copy(source, output / source.name)
    for source in sorted((HERE / 'provenance').rglob('*.json')):
        copy(source, output / ('NOTICE-sunrise-mod-' + source.relative_to(HERE / 'provenance').as_posix().replace('/', '__')))
    dependencies = ('sable-bridge = { path = "src/port/sable-bridge" }\n'
                    'sable-core = { path = "src/port/sable-core", default-features = false }\n'
                    'sable-sunrise = { path = "src/port/sable-sunrise" }\n')
    cargo.write_text(original_cargo.replace('[dependencies]', '[dependencies]\n' + dependencies, 1))
    ledger.append({'source': 'cargo sunrise-mod new generated Cargo.toml',
                   'source_sha256': hashlib.sha256(original_cargo.encode()).hexdigest(),
                   'staged': 'Cargo.toml', 'staged_sha256': digest(cargo),
                   'transformation': 'append three contained personal dependencies; preserve generated SDK dependency'})
    for name in ['sable-bridge', 'sable-core', 'sable-sunrise']:
        source = ROOT / 'crates' / name
        target = output / 'src/port' / name
        for path in sorted((source / 'src').rglob('*.rs')):
            if name == 'sable-core' and path.name == 'physics.rs':
                continue
            copy(path, target / path.relative_to(source))
        for path in sorted(source.glob('LICENSE*')):
            copy(path, output / ('LICENSE-' + name + '-' + path.name))
        for path in sorted((source / 'provenance').rglob('*.json')):
            copy(path, output / ('NOTICE-' + name + '-' + path.relative_to(source / 'provenance').as_posix().replace('/', '__')))
        original = source / 'Cargo.toml'
        text = original.read_text()
        if name == 'sable-bridge':
            # Reference-only tools are not part of the shipped client dependency.
            text = text.split('[dev-dependencies]', 1)[0]
        elif name == 'sable-core':
            # Cargo resolves optional path manifests even when the feature is
            # disabled. The staged SDK package intentionally cannot enable the
            # native realization; that implementation stays in the C-ABI dylib.
            text = ('[package]\nname="sable-core"\nversion="0.1.0"\nedition="2024"\n'
                    'license-file="../../../LICENSE-sable-core-LICENSE-SABLE.md"\n[features]\ndefault=[]\nnative-physics=[]\n')
        (target / 'Cargo.toml').write_text(text)
        ledger.append({'source': str(original.relative_to(ROOT)), 'source_sha256': digest(original),
                       'staged': str((target / 'Cargo.toml').relative_to(output)),
                       'staged_sha256': digest(target / 'Cargo.toml'),
                       'transformation': 'strip bridge dev-tools; remove disabled native-core realization dependency; point core license at contained root notice; preserve runtime source'})
    if digest(manifest) != manifest_hash:
        raise ValueError('generated Sunrise manifest changed while staging')
    result = {'schema': 'sable.phase3.native-package-inputs', 'version': 3,
              'runtime': 'experimental_local_modified', 'runtime_commit': build,
              'generated_manifest_sha256': manifest_hash,
              'generated_manifest_preserved': True, 'sdk_directory_present': True,
              'sdk_verified_by_this_script': False, 'package_built': False, 'runtime_loaded': False,
              'native_physics_in_sdk_dependency_graph': False, 'files': ledger}
    (output / 'source-ledger.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', required=True, type=Path,
                        help='fresh normal-tool generated native project, with its SDK installed')
    args = parser.parse_args()
    result = stage(args.project.absolute())
    print(json.dumps({'project': str(args.project.absolute()), 'source_files': len(result['files']),
                      'package_built': False, 'runtime_loaded': False}))


if __name__ == '__main__':
    main()
