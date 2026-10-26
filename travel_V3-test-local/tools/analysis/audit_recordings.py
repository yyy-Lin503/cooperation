"""Audit the supplied recordings and recover reproducible manual selections."""
from _paths import ROOT, PATHS

import argparse
import bisect
import collections
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re
import os


def percentile(values, p):
    values = sorted(values)
    return values[min(len(values) - 1, int((len(values) - 1) * p))] if values else None


def snapshots(text):
    return [dict(part.split('=', 1) for part in item.split('|') if '=' in part)
            for item in text.split(';') if item]


def cfg_dimensions(path):
    commands = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        tokens = line.split()
        if tokens:
            commands[tokens[0]] = tokens
    profile, frame, channel = (commands[k] for k in ('profileCfg', 'frameCfg', 'channelCfg'))
    adc, loops = int(profile[10]), int(frame[3])
    rx, tx = int(channel[1]).bit_count(), int(channel[2]).bit_count()
    return dict(adc=adc, loops=loops, rx=rx, tx=tx,
                frame_bytes=adc * loops * rx * tx * 4, frame_period_ms=float(frame[5]),
                sampling_ksps=float(profile[11]), slope_mhz_us=float(profile[8]))


def audit(data_root, output, cfg):
    output.mkdir(parents=True, exist_ok=True)
    dimensions = cfg_dimensions(cfg)
    result = []
    diagnostics = data_root / 'diagnostics'
    if not diagnostics.is_dir():
        diagnostics = data_root / 'tracking'  # Explicitly supplied legacy datasets remain readable.
    for path in sorted(diagnostics.glob('*.csv')):
        with path.open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise ValueError(f'Empty CSV: {path}')
        bin_name = Path(rows[0]['input_path'].replace('\\', '/')).name
        binary = data_root / 'recordings/radar' / bin_name
        if not binary.is_file(): binary = data_root / 'bin' / bin_name
        size = binary.stat().st_size
        frames = [int(r['frame']) for r in rows]
        locked = [r for r in rows if r['lock_valid'] == '1']
        measured = [r for r in locked if r['has_measurement'] == '1'
                    and r['association_source'] not in ('reacquire_pending', 'manual_lock_pending')]
        changes = sum(a['matched_internal_uid'] != b['matched_internal_uid']
                      for a, b in zip(locked, locked[1:]))
        stats = dict(
            bin=bin_name, bin_path=str(binary.resolve()), diagnostic_csv=str(path.resolve()),
            bytes=size, bin_frames=size // dimensions['frame_bytes'],
            incomplete_bytes=size % dimensions['frame_bytes'], csv_rows=len(rows),
            first_csv_frame=frames[0], last_csv_frame=frames[-1],
            duplicate_frames=len(frames) - len(set(frames)),
            missing_csv_frames=[i for i in range(1, size // dimensions['frame_bytes'] + 1)
                                if i not in set(frames)],
            recorded_state_counts=dict(collections.Counter(r['control_state'] for r in rows)),
            locked_frames=len(locked), accepted_measurement_frames=len(measured),
            uid_changes=changes,
            max_lost_frames=max((int(r['lost_frames']) for r in locked), default=0),
            recorded_dsp_ms_p95=percentile([float(r['dsp_processing_ms']) for r in rows], .95),
            recorded_dsp_failed=sum(r['dsp_processing_ok'] != '1' for r in rows),
            odom_invalid=sum(r['dsp_odom_valid'] != '1' for r in rows),
            max_abs_wz=max(abs(float(r['dsp_odom_wz_radps'])) for r in rows),
            max_abs_vx=max(abs(float(r['dsp_odom_vx_mps'])) for r in rows),
            rejection_reasons=dict(collections.Counter(r['reject_reason'] for r in locked if r['reject_reason'])),
            recorded_parameters={key: sorted({r[key] for r in rows}) for key in (
                'radar_config_file', 'min_range_m', 'window_frames',
                'new_uid_reacquire_confirm_frames', 'new_uid_reacquire_window_frames',
                'candidate_track_max_gate_m', 'candidate_track_ambiguity_cost_margin')},
        )
        wall_times = [datetime.fromisoformat(r['wall_timestamp']) for r in rows]
        events = []
        log_path = path.with_suffix('.log')
        pattern = re.compile(r'^(\S+) .*已选择人员 P(\d+) / uid=(\d+).*range=([\d.]+)m, angle=([-\d.]+)deg')
        for line in log_path.read_text(encoding='utf-8-sig').splitlines():
            match = pattern.search(line)
            if not match:
                continue
            stamp, display, uid, distance, angle = match.groups()
            index = max(0, bisect.bisect_right(wall_times, datetime.fromisoformat(stamp)) - 1)
            row = rows[index]
            x = float(distance) * math.sin(math.radians(float(angle)))
            y = float(distance) * math.cos(math.radians(float(angle)))
            snap = next((s for s in snapshots(row['candidate_snapshot'])
                         if s.get('uid') == uid), None)
            if snap:
                x, y = float(snap['x']), float(snap['y'])
            events.append(dict(after_frame=int(row['frame']), x=x, y=y,
                               recorded_uid=int(uid), recorded_display=int(display),
                               timestamp=stamp, position_source='candidate_snapshot' if snap else 'log_range_angle'))
        stats['selection_events'] = events
        stem = binary.stem
        with (output / (stem + '_selections.csv')).open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['after_frame', 'x', 'y', 'recorded_uid',
                                                        'recorded_display', 'timestamp', 'position_source'])
            writer.writeheader()
            writer.writerows(events)
        result.append(stats)
    # Paths are relative to this manifest, so moving the whole dataset retains links.
    for entry in result:
        for key in ('bin_path', 'diagnostic_csv'):
            entry[key] = Path(os.path.relpath(entry[key], output.resolve())).as_posix()
    manifest = dict(dimensions=dimensions, cfg=Path(os.path.relpath(cfg.resolve(), output.resolve())).as_posix(), recordings=result,
                    total_frames=sum(s['bin_frames'] for s in result),
                    total_bytes=sum(s['bytes'] for s in result),
                    note='Recorded diagnostics are comparison evidence, not human identity ground truth.')
    (output / 'recordings.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    for entry in result:
        print(f"{entry['bin']}: BIN={entry['bin_frames']} CSV={entry['first_csv_frame']}..{entry['last_csv_frame']} "
              f"missing={len(entry['missing_csv_frames'])} UID changes={entry['uid_changes']} "
              f"lost_max={entry['max_lost_frames']} selections={len(entry['selection_events'])}")
    print(f"TOTAL frames={manifest['total_frames']} frame_bytes={dimensions['frame_bytes']}")
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[2]
    parser.add_argument('--data-root', type=Path, default=PATHS['data'])
    parser.add_argument('--output', type=Path, default=PATHS['data'] / 'reports' / 'audit')
    parser.add_argument('--cfg', type=Path, default=root / 'config/radar/IWR6843_260714-15m-10Hz-9.8cm.cfg')
    args = parser.parse_args()
    audit(args.data_root, args.output, args.cfg)
