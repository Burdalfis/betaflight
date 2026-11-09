#!/usr/bin/env python3
"""Build stock/modified controllers with the same explicit SITL adapter."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ARTIFACTS = ROOT / '.local-artifacts/tricopter/gazebo-sitl'
STOCK_REF = 'e0b7bb01b17b21351057e9ead2d1ab39dd44fa16'
ADAPTER_PREIMAGES = {
    'src/platform/SIMULATOR/sitl.c': '630d629bc97e3c74de1de83b22b38378a82d5778c8779729809fc7244f276541',
    'src/platform/SIMULATOR/sitl_gyro.h': '6d906ee9e15cbcec7f1fe4d3997f183051f63db5054daad40bfa3f03c193132f',
    'src/main/flight/imu.c': 'b4ea5d546ab605cb9515d34c1d1b4461b06b9eacd552bd282c3835a503f24584',
    'src/main/flight/pid.c': 'd1065b12de3936c8bbc0385e8fee81928bb7f08779f3d0f915ab99f69b75e013',
}
ADAPTER_PATCH_SHA256 = '7ce82ec3438216c6bc8046dab3ea5995918849a2ca278c7af854fc7589ad2807'
ADAPTER_POSTIMAGES = {
    'src/platform/SIMULATOR/sitl.c': '27702226bc66063d41f063d521331ce999fa71c2018dd8a30d42f3f761236891',
    'src/platform/SIMULATOR/sitl_gyro.h': 'db5bc903783cd5ac8c97522e28fbd6df7c6be23eff6aac3d0bd987d560beebf0',
    'src/main/flight/imu.c': '8b2eae8d29b7285e797d7f7dbd7e88167cda938272e7029e661bcd5e3a92699a',
    'src/main/flight/pid.c': 'fd741ea280d725fb07673e9116770e558ace930ef5f5fda81963033abb1428e0',
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def verify_stock(directory: Path) -> dict:
    """Check every regular file/symlink against the pinned Git tree, excluding gitlinks."""
    listing = subprocess.check_output(['git', '-C', str(ROOT), 'ls-tree', '-rz', STOCK_REF])
    checked = 0
    expected_paths = set()
    for item in listing.split(b'\0'):
        if not item:
            continue
        metadata, name = item.split(b'\t', 1)
        mode, kind, blob = metadata.decode().split()
        if kind != 'blob':
            continue
        path = directory / os.fsdecode(name)
        expected_paths.add(path)
        data = os.fsencode(os.readlink(path)) if mode == '120000' else path.read_bytes()
        actual = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
        if actual != blob:
            raise RuntimeError(f'Stock source was changed: {path}')
        checked += 1
    extras = [str(path.relative_to(directory)) for path in directory.rglob('*')
              if (path.is_file() or path.is_symlink()) and path not in expected_paths]
    if extras:
        raise RuntimeError(f'Unexpected files in pristine stock source: {extras[:10]}')
    return {'commit': STOCK_REF, 'git_tree': git('rev-parse', STOCK_REF + '^{tree}'),
            'verified_git_blobs': checked, 'source_files_unchanged': True}


def stock_source(artifacts: Path) -> tuple[Path, dict]:
    directory = artifacts / 'stock-source'
    if not directory.exists():
        directory.mkdir(parents=True)
        proc = subprocess.Popen(['git', '-C', str(ROOT), 'archive', '--format=tar', STOCK_REF], stdout=subprocess.PIPE)
        try:
            with tarfile.open(fileobj=proc.stdout, mode='r|') as archive:
                archive.extractall(directory, filter='data')
        finally:
            proc.stdout.close()
        if proc.wait() != 0:
            raise RuntimeError('Could not export pinned stock source')
    return directory, verify_stock(directory)


def fingerprint(value: dict) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def source_tree(directory: Path, paths=None) -> dict:
    """Record bytes, executable modes and link targets without following links."""
    candidates = directory.rglob('*') if paths is None else (directory/name for name in paths)
    result = {}
    for path in sorted(candidates):
        if path.is_symlink():
            result[str(path.relative_to(directory))] = {'kind': 'symlink', 'target': os.readlink(path)}
        elif path.is_file():
            result[str(path.relative_to(directory))] = {'kind': 'file', 'sha256': digest(path),
                                                       'mode': stat.S_IMODE(path.stat().st_mode)}
    return result


def adapter_definition(name: str, patch_path: Path | None = None) -> dict:
    if name != 'corrected-frame':
        raise ValueError(f'Unknown SITL adapter: {name}')
    patch = patch_path or ROOT/'tools/tricopter/sitl/patches/corrected-frame.patch'
    data = patch.read_bytes()
    text = data.decode('utf-8')
    old = re.findall(r'^--- a/(.+)$', text, re.MULTILINE)
    new = re.findall(r'^\+\+\+ b/(.+)$', text, re.MULTILINE)
    headers = re.findall(r'^diff --git a/(.+) b/(.+)$', text, re.MULTILINE)
    expected = set(ADAPTER_PREIMAGES)
    if (len(old) != len(expected) or len(new) != len(expected) or len(headers) != len(expected)
            or set(old) != expected or set(new) != expected
            or any(a != b or a not in expected for a, b in headers)
            or re.search(r'^(?:old mode|new mode|new file mode|deleted file mode|rename |copy |GIT binary patch)', text, re.MULTILINE)):
        raise RuntimeError('SITL adapter patch must change only the four allowlisted existing file contents')
    if hashlib.sha256(data).hexdigest() != ADAPTER_PATCH_SHA256:
        raise RuntimeError('SITL adapter patch differs from its approved exact bytes')
    return {'name': name, 'patch_file': str(patch.resolve()), 'patch_sha256': hashlib.sha256(data).hexdigest(),
            'allowed_preimage_sha256': dict(ADAPTER_PREIMAGES)}


def apply_adapter(directory: Path, definition: dict) -> dict:
    """Fail closed on changed preimages, patch bytes, modes, or extra paths."""
    before = source_tree(directory)
    for name, expected in definition['allowed_preimage_sha256'].items():
        if before.get(name, {}).get('kind') != 'file' or before[name]['sha256'] != expected:
            raise RuntimeError(f'SITL adapter preimage mismatch: {name}')
    patch = Path(definition['patch_file']).read_bytes()
    if hashlib.sha256(patch).hexdigest() != definition['patch_sha256']:
        raise RuntimeError('SITL adapter patch changed after its identity was recorded')
    environment = os.environ.copy()
    for name in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE', 'GIT_COMMON_DIR'):
        environment.pop(name, None)
    environment['GIT_CEILING_DIRECTORIES'] = str(directory.parent.resolve())
    for arguments in (['--check'], []):
        result = subprocess.run(['git', 'apply', *arguments, '-'], input=patch, cwd=directory,
                                env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode:
            raise RuntimeError('Cannot apply common SITL adapter: '+result.stderr.decode(errors='replace'))
    # git apply may replace 0644 files with 0664 under umask 0002, even for a
    # content-only patch. Keep the exported input modes as well as its bytes.
    for name in definition['allowed_preimage_sha256']:
        path = directory/name
        if not stat.S_ISREG(path.lstat().st_mode):
            raise RuntimeError('SITL adapter changed a file kind')
        path.chmod(before[name]['mode'])
    after = source_tree(directory)
    changed = sorted(name for name in set(before)|set(after) if before.get(name) != after.get(name))
    if changed != sorted(definition['allowed_preimage_sha256']):
        raise RuntimeError(f'SITL adapter changed unexpected paths: {changed}')
    if any(after[name]['kind'] != 'file' or before[name]['mode'] != after[name]['mode'] for name in changed):
        raise RuntimeError('SITL adapter changed a file kind or mode')
    if any(after[name]['sha256'] != expected for name, expected in ADAPTER_POSTIMAGES.items()):
        raise RuntimeError('SITL adapter produced an unexpected postimage')
    return {**definition, 'changed_paths': changed,
            'applied_postimage_sha256': {name: after[name]['sha256'] for name in changed},
            'unadapted_tree_sha256': fingerprint(before), 'adapted_tree_sha256': fingerprint(after),
            'source_file_count': len(after)}


def snapshot_source(source: Path, artifacts: Path, variant: str, adapter: dict, paths=None) -> tuple[Path, dict]:
    """Keep content-addressed copies immutable; never edit either input tree."""
    original = source_tree(source, paths)
    identity = fingerprint(original)
    cache = artifacts/'source-snapshots'; cache.mkdir(parents=True, exist_ok=True)
    destination = cache/f'{variant}-{identity[:20]}-{adapter["patch_sha256"][:20]}'
    if destination.exists():
        metadata = json.loads((destination/'snapshot-manifest.json').read_text())
        expected_tree = {name:dict(entry) for name, entry in original.items()}
        for name, expected in ADAPTER_POSTIMAGES.items():
            expected_tree[name] = {**expected_tree.get(name, {}), 'sha256': expected}
        if (metadata['adapter']['unadapted_tree_sha256'] != identity
                or metadata['adapter']['patch_sha256'] != adapter['patch_sha256']
                or metadata['unadapted_tree'] != original
                or metadata['adapted_tree'] != expected_tree
                or source_tree(destination/'source') != expected_tree):
            raise RuntimeError(f'Existing adapted source snapshot changed: {destination}')
        return destination/'source', metadata['adapter']
    temporary = Path(tempfile.mkdtemp(prefix='.preparing-', dir=cache))
    try:
        copied = temporary/'source'; copied.mkdir()
        for name, entry in original.items():
            target = copied/name; target.parent.mkdir(parents=True, exist_ok=True)
            if entry['kind'] == 'symlink':
                target.symlink_to(entry['target'])
            else:
                shutil.copy2(source/name, target)
        if source_tree(copied) != original or source_tree(source, original) != original:
            raise RuntimeError('Source changed while making the adapted build snapshot')
        applied = apply_adapter(copied, adapter)
        metadata = {'variant': variant, 'original_source': str(source.resolve()),
                    'unadapted_tree': original, 'adapted_tree': source_tree(copied), 'adapter': applied}
        (temporary/'snapshot-manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
        temporary.rename(destination)
    finally:
        if temporary.exists(): shutil.rmtree(temporary)
    return destination/'source', applied


def validate_output_mode(artifacts: Path, variant: str, adapter: str) -> None:
    manifest = artifacts/variant/'build-manifest.json'
    marker = artifacts/variant/'sitl-adapter.json'
    if manifest.exists():
        metadata = json.loads(manifest.read_text())
        existing = metadata.get('provenance', {}).get('sitl_adapter', {}).get('name', 'none')
        if existing != adapter:
            raise RuntimeError(f'Refusing to overwrite {existing} {variant} artifacts with {adapter}; use a separate --artifacts directory')
    elif marker.exists():
        if json.loads(marker.read_text())['name'] != adapter:
            raise RuntimeError(f'Refusing to change the adapter of an unfinished {variant} build')
    elif adapter != 'none' and (artifacts/variant).exists() and any((artifacts/variant).iterdir()):
        raise RuntimeError(f'Unrecognized nonempty build directory: {artifacts/variant}')
    if adapter != 'none' and artifacts.resolve() == DEFAULT_ARTIFACTS.resolve():
        raise RuntimeError('Use --artifacts with a separate corrected-frame directory for the common SITL adapter')


def source_hashes(source: Path) -> dict:
    tracked = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '-z', 'src/platform/SIMULATOR',
                                     'src/main/flight', 'mk/source.mk', 'src/main/config/config.c'])
    untracked = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '--others', '--exclude-standard', '-z',
                                       'src/platform/SIMULATOR'])
    paths = sorted(set(os.fsdecode(name) for name in (tracked+untracked).split(b'\0') if name))
    return {name: digest(source/name) for name in paths if (source/name).is_file()}


def adapted_provenance(variant: str, original: Path, provenance: dict, adapter: dict) -> dict:
    """Pristine verification describes the input, never the patched binary."""
    result = {'pristine_input_verification': provenance} if variant == 'stock' else dict(provenance)
    return {**result, 'sitl_adapter': adapter, 'unadapted_source': str(original),
            'controller_variant': variant, 'built_source_is_pristine_stock': False,
            'built_source_has_common_sitl_adapter': True}


def build(variant: str, artifacts: Path, jobs: int, sitl_adapter: str = 'none') -> dict:
    validate_output_mode(artifacts, variant, sitl_adapter)
    if variant == 'stock':
        original, provenance = stock_source(artifacts)
        revision = STOCK_REF[:9]
    else:
        original = ROOT
        provenance = {'base_commit': git('rev-parse', 'HEAD'), 'working_tree_snapshot': True}
        revision = 'sitl-dev'
    source = original
    live_hashes = source_hashes(original) if variant == 'modified' and sitl_adapter != 'none' else None
    if sitl_adapter != 'none':
        adapter = adapter_definition(sitl_adapter)
        paths = None
        if variant == 'modified':
            listing = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '--cached', '--others', '--exclude-standard', '-z'])
            paths = sorted(set(os.fsdecode(name) for name in listing.split(b'\0') if name))
        source, applied = snapshot_source(original, artifacts, variant, adapter, paths)
        provenance = adapted_provenance(variant, original, provenance, applied)
    output = artifacts / variant
    output.mkdir(parents=True, exist_ok=True)
    if sitl_adapter != 'none':
        (output/'sitl-adapter.json').write_text(json.dumps({'name': sitl_adapter})+'\n')
    command = ['make', f'-j{jobs}', 'TARGET=SITL', 'CCACHE=', 'AUTOHYDRATE_SUBMODULES=', 'EXTRA_FLAGS=-DUSE_SERVOS',
               f'OBJECT_DIR={output / "obj/main"}', f'BIN_DIR={output / "obj"}', f'REVISION={revision}']
    environment = os.environ.copy()
    environment['SOURCE_DATE_EPOCH'] = git('show', '-s', '--format=%ct', STOCK_REF)
    environment['GIT_CEILING_DIRECTORIES'] = str(source.parent)
    log = output / 'build.log'
    print(f'Building {variant}: {source}; log {log}', flush=True)
    with log.open('w') as stream:
        result = subprocess.run(command, cwd=source, env=environment, stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        print(log.read_text()[-10000:], flush=True)
        raise RuntimeError(f'{variant} build failed ({result.returncode}); see {log}')
    binary = output / 'betaflight_SITL.elf'
    shutil.copy2(output / 'obj/main/betaflight_SITL.elf', binary)
    binary.chmod(0o755)
    if variant == 'stock':
        checked = verify_stock(original)
        if sitl_adapter == 'none':
            provenance = checked
        else:
            provenance['pristine_input_verification'] = checked
    if sitl_adapter != 'none':
        snapshot = source.parent/'snapshot-manifest.json'
        metadata = json.loads(snapshot.read_text())
        if source_tree(source) != metadata['adapted_tree']:
            raise RuntimeError('Adapted source changed during build')
        if live_hashes is not None and source_hashes(original) != live_hashes:
            raise RuntimeError('Modified working-tree source changed during build')
    hashes = source_hashes(source)
    report = {'variant': variant, 'source': str(source), 'provenance': provenance, 'command': command,
              'source_date_epoch': environment['SOURCE_DATE_EPOCH'], 'binary': str(binary),
              'binary_sha256': digest(binary), 'build_log_sha256': digest(log), 'source_sha256': hashes}
    if sitl_adapter != 'none':
        report['source_snapshot_manifest'] = str(snapshot)
        report['source_snapshot_manifest_sha256'] = digest(snapshot)
        if live_hashes is not None:
            report['live_source_root'] = str(original)
            report['live_source_sha256'] = live_hashes
    (output / 'build-manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Built {variant}: {binary} SHA256 {report["binary_sha256"]}', flush=True)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['stock', 'modified', 'both'], default='both')
    parser.add_argument('--artifacts', type=Path, default=DEFAULT_ARTIFACTS/'corrected-frame')
    parser.add_argument('--jobs', type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument('--sitl-adapter', choices=['none', 'corrected-frame'], default='corrected-frame',
                        help='Identical simulator-only compatibility patch on isolated source copies (default: corrected-frame); none reproduces the historical plain build')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    artifacts = args.artifacts.resolve()
    variants = ['stock', 'modified'] if args.variant == 'both' else [args.variant]
    for variant in variants:
        validate_output_mode(artifacts, variant, args.sitl_adapter)
    for variant in variants:
        build(variant, artifacts, args.jobs, args.sitl_adapter)


if __name__ == '__main__':
    main()
