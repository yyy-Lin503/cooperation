"""Summarize replay observations without treating recorded IDs/positions as truth."""
from _paths import ROOT, PATHS

import csv
import argparse
from replay_inputs import resolve_input
from datetime import datetime
import json
import math
from pathlib import Path
import collections

ROOT = Path(__file__).resolve().parents[2]


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
                accepted_frames=len(measured), accepted_fraction=len(measured) / len(active) if active else 0.0,
                uid_changes=changes, max_lost=max((int(r['lost_frames']) for r in active), default=0),
                states=dict(collections.Counter(r['control_state'] for r in active)),
                selection_success=all(r['selection_success'] == '1' for r in rows
                                      if r['selection_success'] != '-1'),
                failed_frames=sum(r['dsp_ok'] != '1' for r in rows),
                reject_reasons=dict(collections.Counter(r['reject_reason'] for r in active if r['reject_reason'])),
                missing_odom=sum(r['motion_present'] == '0' for r in rows), long_loss_episodes=episodes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True, help='Current replay frame CSV')
    parser.add_argument('--selection-frame', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(read_csv(args.frames), args.selection_frame)
    result['source'] = str(args.frames.resolve())
    result['limitations'] = ['Replay observations and UID continuity are not identity ground truth.',
                              'CPU replay timings do not measure Jetson GPU performance.']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
