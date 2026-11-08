#!/usr/bin/env python3
"""Build genuine stock and working-tree Betaflight SITL in separate directories."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ARTIFACTS = ROOT / '.local-artifacts/tricopter/gazebo-sitl'
STOCK_REF = 'e0b7bb01b17b21351057e9ead2d1ab39dd44fa16'


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


def build(variant: str, artifacts: Path, jobs: int) -> dict:
    if variant == 'stock':
        source, provenance = stock_source(artifacts)
        revision = STOCK_REF[:9]
    else:
        source = ROOT
        provenance = {'base_commit': git('rev-parse', 'HEAD'), 'working_tree_snapshot': True}
        revision = 'sitl-dev'
    output = artifacts / variant
    output.mkdir(parents=True, exist_ok=True)
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
        provenance = verify_stock(source)
    tracked = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '-z', 'src/platform/SIMULATOR',
                                     'src/main/flight', 'mk/source.mk', 'src/main/config/config.c'])
    untracked = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '--others', '--exclude-standard', '-z',
                                       'src/platform/SIMULATOR'])
    paths = sorted(set(os.fsdecode(name) for name in (tracked + untracked).split(b'\0') if name))
    hashes = {name: digest(source / name) for name in paths if (source / name).is_file()}
    report = {'variant': variant, 'source': str(source), 'provenance': provenance, 'command': command,
              'source_date_epoch': environment['SOURCE_DATE_EPOCH'], 'binary': str(binary),
              'binary_sha256': digest(binary), 'build_log_sha256': digest(log), 'source_sha256': hashes}
    (output / 'build-manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'Built {variant}: {binary} SHA256 {report["binary_sha256"]}', flush=True)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['stock', 'modified', 'both'], default='both')
    parser.add_argument('--artifacts', type=Path, default=DEFAULT_ARTIFACTS)
    parser.add_argument('--jobs', type=int, default=min(8, os.cpu_count() or 1))
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    artifacts = args.artifacts.resolve()
    variants = ['stock', 'modified'] if args.variant == 'both' else [args.variant]
    for variant in variants:
        build(variant, artifacts, args.jobs)


if __name__ == '__main__':
    main()
