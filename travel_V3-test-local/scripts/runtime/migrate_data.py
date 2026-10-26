"""Move legacy runtime files in verified groups; never overwrite existing data."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/my_radar_follower'))
from my_radar_follower.runtime_support import runtime_paths


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def migrate(root, apply=False):
    root = Path(root).resolve()
    paths = runtime_paths(root)
    mappings = dict(bin=paths['data'] / 'recordings/radar',
                    txt=paths['data'] / 'pointcloud', track_txt=paths['track'],
                    track_csv=paths['data'] / 'tracking/csv',
                    tracking_diagnostics=paths['data'] / 'diagnostics',
                    camera_recordings=paths['data'] / 'recordings/camera',
                    reports=paths['data'] / 'reports')
    pending = []
    for legacy, destination in mappings.items():
        source = root / legacy
        if not source.exists():
            continue
        if source.is_symlink() or not source.resolve().is_relative_to(root):
            raise ValueError(f'Legacy source must be inside this project: {source}')
        if destination.resolve().is_relative_to(source.resolve()):
            raise ValueError(f'Destination overlaps legacy source: {destination}')
        for item in source.rglob('*'):
            if item.is_symlink():
                raise ValueError(f'Symlink requires explicit manual migration: {item}')
            if item.is_file():
                pending.append((item, destination / item.relative_to(source)))
    for name, destination in (
        ('gui_runtime.ini', paths['settings']),
        ('tracking_window_stats.csv', paths['data'] / 'reports/tracking_window_stats.csv'),
        ('CFAR_T_matrix.txt', paths['data'] / 'diagnostics/CFAR_T_matrix.txt'),
    ):
        source = root / name
        if source.is_file():
            # A saved new configuration wins. Keep the old values as migration evidence.
            if name == 'gui_runtime.ini' and destination.exists() and digest(source) != digest(destination):
                destination = paths['settings'].with_name('gui_runtime.legacy.ini')
            pending.append((source, destination))
    # Validate the complete mapping before changing any file.
    for source, destination in pending:
        if source.resolve() == destination.resolve():
            raise ValueError(f'Source and destination are the same: {source}')
        if destination.exists() and digest(source) != digest(destination):
            raise FileExistsError(f'Refusing to overwrite different data: {destination}')
    records = []
    for source, destination in pending:
        checksum = digest(source)
        records.append(dict(source=str(source), destination=str(destination), sha256=checksum))
        if apply:
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                # Exclusive creation also protects against a concurrently created destination.
                with source.open('rb') as src, destination.open('xb') as dst:
                    shutil.copyfileobj(src, dst)
            if digest(destination) != checksum:
                raise IOError(f'Migration checksum mismatch: {destination}')
    # Delete originals only after every member of every group was copied and verified.
    if apply:
        for source, _ in pending:
            source.unlink()
        for legacy in mappings:
            source = root / legacy
            if source.is_dir():
                for folder in sorted((p for p in source.rglob('*') if p.is_dir()),
                                     key=lambda p: len(p.parts), reverse=True):
                    if not any(folder.iterdir()):
                        folder.rmdir()
                if not any(source.iterdir()):
                    source.rmdir()
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Copy, verify all files, then remove originals')
    args = parser.parse_args()
    records = migrate(ROOT, args.apply)
    output = dict(applied=args.apply, files=records)
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if args.apply:
        folder = runtime_paths(ROOT)['data'] / 'reports'
        folder.mkdir(parents=True, exist_ok=True)
        (folder / 'data-migration.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
