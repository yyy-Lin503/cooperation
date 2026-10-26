"""Summarize replay observations without treating recorded IDs/positions as truth."""

import csv
import argparse
from replay_inputs import resolve_input
from datetime import datetime
import json
import math
from pathlib import Path
import collections

ROOT = Path(__file__).resolve().parents[1]


def read_csv(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def summarize(rows, selection_frame):
    active = [r for r in rows if int(r['frame']) > selection_frame]
    valid = [r for r in active if r['lock_valid'] == '1']
    measured = [r for r in active if r['lock_valid'] == '1' and r['measured'] == '1'
                and r['association_source'] not in ('reacquire_pending', 'manual_lock_pending')]
    changes = sum(a['uid'] != b['uid'] for a, b in zip(valid, valid[1:]))
    episodes, run = [], []
    for row in active + [None]:
        if row and row['lock_valid'] == '1' and int(row['lost_frames']) > 0:
            run.append(row)
        elif run:
            maximum = max(int(r['lost_frames']) for r in run)
            if maximum >= 20:
                episodes.append(dict(start_frame=int(run[0]['frame']), end_frame=int(run[-1]['frame']),
                                     max_lost=maximum, duration_s=len(run) * .1,
                                     frames_with_candidates=sum(int(r['candidates']) > 0 for r in run),
                                     reasons=dict(collections.Counter(r['reject_reason'] for r in run if r['reject_reason']))))
            run = []
    return dict(frames=len(rows), evaluated_frames=len(active), locked_frames=len(valid),
                accepted_frames=len(measured), accepted_fraction=len(measured) / len(active),
                uid_changes=changes, max_lost=max((int(r['lost_frames']) for r in active), default=0),
                states=dict(collections.Counter(r['control_state'] for r in active)),
                selection_success=all(r['selection_success'] == '1' for r in rows
                                      if r['selection_success'] != '-1'),
                failed_frames=sum(r['dsp_ok'] != '1' for r in rows),
                reject_reasons=dict(collections.Counter(r['reject_reason'] for r in active if r['reject_reason'])),
                missing_odom=sum(r['motion_present'] == '0' for r in rows), long_loss_episodes=episodes)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=ROOT/'reports/audit/recordings.json')
    parser.add_argument('--data-root',type=Path)
    args=parser.parse_args()
    audit=json.loads(args.manifest.read_text(encoding='utf-8'))
    for entry in audit['recordings']:
        entry['diagnostic_csv']=str(resolve_input(entry['diagnostic_csv'],args.manifest,args.data_root,'diagnostics'))
    results = []
    for entry in audit['recordings']:
        stem = Path(entry['bin']).stem
        selection = entry['selection_events'][0]['after_frame']
        result = dict(bin=entry['bin'], selection_frame=selection)
        for version in ('v1', 'v2'):
            rows = read_csv(ROOT / 'reports/replay' / version / (stem + '_frames.csv'))
            result[version] = summarize(rows, selection)
            reference = {r['frame']: r for r in read_csv(entry['diagnostic_csv'])}
            result[version]['cfar_match_recorded'] = sum(
                r['cfar_targets'] == reference[r['frame']]['cfar_targets']
                for r in rows if r['frame'] in reference)
            result[version]['reference_comparable_frames'] = len(reference)
            # Frames with virtually stationary platform and the FFT DC cell.
            stationary = {r['frame'] for r in reference.values()
                          if abs(float(r['dsp_odom_vx_mps'])) < .02
                          and abs(float(r['dsp_odom_wz_radps'])) < .02}
            dc_values = [float(p['relative_velocity']) for p in read_csv(
                ROOT / 'reports/replay' / version / (stem + '_points.csv'))
                if p['frame'] in stationary and p['doppler_bin'] == '32']
            result[version]['stationary_dc_points'] = len(dc_values)
            result[version]['stationary_dc_mean_mps'] = sum(dc_values) / len(dc_values) if dc_values else None
        results.append(result)
        print(f"{stem}: accepted {result['v1']['accepted_fraction']:.1%} -> "
              f"{result['v2']['accepted_fraction']:.1%}; UID {result['v1']['uid_changes']} -> "
              f"{result['v2']['uid_changes']}; lost {result['v1']['max_lost']} -> {result['v2']['max_lost']}")
    total = {version: dict(
        frames=sum(r[version]['frames'] for r in results),
        evaluated_frames=sum(r[version]['evaluated_frames'] for r in results),
        accepted_frames=sum(r[version]['accepted_frames'] for r in results),
        uid_changes=sum(r[version]['uid_changes'] for r in results),
        failed_frames=sum(r[version]['failed_frames'] for r in results),
        cfar_matches=sum(r[version]['cfar_match_recorded'] for r in results),
        cfar_comparable=sum(r[version]['reference_comparable_frames'] for r in results),
        dc_points=sum(r[version]['stationary_dc_points'] for r in results),
    ) for version in ('v1', 'v2')}
    summary = dict(recordings=results, totals=total,
                   limitations=['No independent video/human identity ground truth.',
                                'Historical CSV uses V11 parameters, not current V1 defaults.',
                                'Replay uses CPU FFT; timing is not Jetson GPU performance.',
                                'V1 comparator uses correct frame sizing so undefined memory reads are not executed.'])
    (ROOT / 'reports/comparison.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(total, ensure_ascii=False))


if __name__ == '__main__':
    main()
