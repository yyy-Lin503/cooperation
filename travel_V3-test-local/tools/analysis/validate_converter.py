"""Check real-data cfg scaling, frame offsets and unaligned-offset rejection."""
from _paths import ROOT, PATHS

import csv
import argparse
from replay_inputs import resolve_input
import json
import math
import os
from pathlib import Path
import subprocess


def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=PATHS['data']/'reports/audit/recordings.json')
    parser.add_argument('--data-root',type=Path)
    parser.add_argument('--converter',type=Path,required=True,help='Explicit bin2pointcloud executable')
    parser.add_argument('--output-dir',type=Path,default=PATHS['data']/'reports/converter_checks')
    args = parser.parse_args()
    audit = json.loads(args.manifest.read_text(encoding='utf-8'))
    entry = dict(audit['recordings'][0])
    entry['bin_path'] = str(resolve_input(entry['bin_path'],args.manifest,args.data_root,'bin'))
    entry['diagnostic_csv'] = str(resolve_input(entry['diagnostic_csv'],args.manifest,args.data_root,'diagnostics'))
    executable = args.converter.resolve()
    directory = args.output_dir
    directory.mkdir(parents=True, exist_ok=True)
    results = {}
    cases = [
        ('correct_cfg', 'IWR6843_260714-15m-10Hz-9.8cm.cfg', 0, 5, 0),
        ('wrong_cfg', 'IWR6843_260520-25m-10Hz-9.8cm.cfg', 0, 5, 0),
        ('offset_frame3', 'IWR6843_260714-15m-10Hz-9.8cm.cfg', 1048576, 1, 0),
        ('unaligned_offset', 'IWR6843_260714-15m-10Hz-9.8cm.cfg', 1, 1, 1),
    ]
    for name, cfg, offset, count, expected in cases:
        command = [str(executable), entry['bin_path'], str(root / 'config/radar' / cfg),
                   '--diagnostic-csv', entry['diagnostic_csv'],
                   '--output', str(directory / (name + '.csv')),
                   '--offset-bytes', str(offset), '--max-frames', str(count)]
        result = subprocess.run(command, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        output = result.stdout.decode('utf-8', errors='replace')
        results[name] = dict(exit_code=result.returncode, command=command, output=output)
        if result.returncode != expected:
            raise AssertionError(f'{name}: expected {expected}, got {result.returncode}: {output}')
    good = rows(directory / 'correct_cfg.csv')
    wrong = {(r['Frame'], r['RangeBin'], r['DopplerBin']): r for r in rows(directory / 'wrong_cfg.csv')}
    ratios = []
    for point in good:
        other = wrong.get((point['Frame'], point['RangeBin'], point['DopplerBin']))
        if other:
            ratios.append(math.hypot(float(other['X']), float(other['Y'])) /
                          math.hypot(float(point['X']), float(point['Y'])))
    if not ratios or not all(abs(r - 1.5) < .001 for r in ratios):
        raise AssertionError('Wrong-cfg distance scaling is not reproduced')
    offset_rows = rows(directory / 'offset_frame3.csv')
    if not offset_rows or {r['Frame'] for r in offset_rows} != {'3'}:
        raise AssertionError('Offset output did not keep source frame 3')
    timestamps = sorted({r['Timestamp'] for r in offset_rows if r['Timestamp']})
    with Path(entry['diagnostic_csv']).open(encoding='utf-8-sig', newline='') as stream:
        recorded = next(csv.DictReader(stream))
    if timestamps != [recorded['radar_timestamp']]:
        raise AssertionError(f'Offset output time does not match recorded frame 3: {timestamps}')
    if 'must be aligned' not in results['unaligned_offset']['output']:
        raise AssertionError('Unaligned offset was not explicitly rejected')
    results['checks'] = dict(status='PASS', same_rd_cell_points=len(ratios),
                            distance_ratio_min=min(ratios), distance_ratio_max=max(ratios),
                            distance_ratio_mean=sum(ratios) / len(ratios),
                            offset_source_frame=3, offset_timestamp=timestamps[0],
                            frame_bytes=audit['dimensions']['frame_bytes'])
    (directory / 'validation.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(results['checks'], ensure_ascii=False))


if __name__ == '__main__':
    main()
