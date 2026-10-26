"""Plot measured replay observations and animate a genuine loss episode."""

import csv
import os
from replay_inputs import resolve_input
import collections
import json
from pathlib import Path
try:
    import matplotlib
except ModuleNotFoundError:
    from plot_replays_pillow import main
    main()
    raise SystemExit(0)
import numpy as np
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = Path(__file__).resolve().parents[1]


def load(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    dest = ROOT / 'reports/figures'
    dest.mkdir(parents=True, exist_ok=True)
    comparison = json.loads((ROOT / 'reports/comparison.json').read_text(encoding='utf-8'))
    audit = json.loads((ROOT / 'reports/audit/recordings.json').read_text(encoding='utf-8'))
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    entries = comparison['recordings']
    x = np.arange(len(entries))
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for version, offset, color in [('v1', -.17, '#ba6a26'), ('v2', .17, '#16827b')]:
        axes[0].bar(x + offset, [100 * e[version]['accepted_fraction'] for e in entries], .34,
                    label=version.upper(), color=color)
    axes[0].set_xticks(x, [e['bin'][9:13] for e in entries], rotation=45)
    axes[0].set_ylim(0, 100)
    axes[0].set_ylabel('Accepted observations / post-selection frames (%)')
    axes[0].set_xlabel('Recording start time (HHMM)')
    axes[0].set_title('Observation coverage did not materially improve')
    axes[0].legend()
    axes[1].bar(['V1-compatible', 'V2 corrected'], [-.279017836, 0.0], color=['#ba6a26', '#16827b'])
    axes[1].axhline(0, color='#444444', linewidth=1)
    axes[1].set_ylim(-.36, .10)
    axes[1].set_ylabel('Mean reported relative speed of FFT DC cells (m/s)')
    axes[1].set_title('7490 DC points on nearly stationary-platform frames')
    fig.suptitle('Full raw-BIN CPU replay: 10 recordings, 6155 frames per version', fontsize=13)
    fig.tight_layout()
    fig.savefig(dest / 'all_recordings_comparison.png', dpi=160)
    plt.close(fig)

    stem = '20260814_1638_52_564_raw'
    v1 = load(ROOT / 'reports/replay/v1' / (stem + '_frames.csv'))
    v2 = load(ROOT / 'reports/replay/v2' / (stem + '_frames.csv'))
    reference_entry = next(e for e in audit['recordings'] if Path(e['bin']).stem == stem)
    reference = load(resolve_input(reference_entry['diagnostic_csv'],ROOT/'reports/audit/recordings.json',os.environ.get('RADAR_REPLAY_DATA_ROOT'),'diagnostics'))
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for rows, tag, color in [(v1, 'V1', '#ba6a26'), (v2, 'V2', '#16827b')]:
        time = np.array([int(r['frame']) * .1 for r in rows])
        valid = np.array([r['lock_valid'] == '1' for r in rows])
        axes[0].plot(time, np.where(valid, np.hypot([float(r['x']) for r in rows],
                                                  [float(r['y']) for r in rows]), np.nan),
                     label=tag, color=color, linewidth=1.1)
        axes[1].plot(time, [int(r['lost_frames']) for r in rows], label=tag, color=color, linewidth=1.1)
        axes[2].plot(time, [int(r['candidates']) for r in rows], label=tag, color=color, linewidth=1.0)
    rt = [int(r['frame']) * .1 for r in reference]
    axes[0].plot(rt, [float(r['filtered_range_m']) for r in reference], color='#777777',
                 linestyle=':', linewidth=1, label='Recorded V11 estimate (not ground truth)')
    axes[0].set_ylabel('Filtered range (m)')
    axes[1].set_ylabel('Frames since acceptance')
    axes[1].axhline(30, color='#a83030', linestyle='--', linewidth=.8, label='Prediction motion limit')
    axes[2].set_ylabel('Visible candidates')
    axes[2].set_xlabel('Recording time (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(loc='upper right', fontsize=8)
    fig.suptitle('16:38:52 recording: candidate observations exist while lock recovery fails', fontsize=13)
    fig.tight_layout()
    fig.savefig(dest / '163852_loss_timeline.png', dpi=160)
    plt.close(fig)

    start_frame = next(int(r['frame']) for r in v2 if int(r['lost_frames']) >= 20)
    start_frame = max(1, start_frame - 25)
    frames = list(range(start_frame, min(start_frame + 100, len(v2) + 1)))
    points = collections.defaultdict(list)
    for row in load(ROOT / 'reports/replay/v2' / (stem + '_points.csv')):
        if int(row['frame']) in frames:
            points[int(row['frame'])].append((float(row['x']), float(row['y']), int(row['is_moving'])))
    by_frame = {int(r['frame']): r for r in v2}
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(-7, 7)
    ax.set_ylim(0, 15)
    ax.set_aspect('equal')
    ax.set_xlabel('Radar lateral x (m)')
    ax.set_ylabel('Radar forward y (m)')
    ax.grid(alpha=.2)
    scatter = ax.scatter([], [], s=12, alpha=.7)
    target, = ax.plot([], [], 'o', markersize=9, markerfacecolor='none', markeredgecolor='#dd5426',
                      markeredgewidth=2, label='Filtered lock (may be predicted)')
    trail, = ax.plot([], [], color='#dd5426', linewidth=1, alpha=.55)
    ax.plot([0], [0], '^', color='#333333', markersize=10)
    ax.legend(loc='upper right', fontsize=8)

    def update(index):
        frame = frames[index]
        values = points[frame]
        scatter.set_offsets(np.array([(x, y) for x, y, _ in values]).reshape(-1, 2))
        scatter.set_color(['#16827b' if moving else '#929292' for _, _, moving in values])
        row = by_frame[frame]
        if row['lock_valid'] == '1':
            target.set_data([float(row['x'])], [float(row['y'])])
        else:
            target.set_data([], [])
        history = [by_frame[f] for f in frames[max(0, index-20):index+1] if by_frame[f]['lock_valid'] == '1']
        trail.set_data([float(r['x']) for r in history], [float(r['y']) for r in history])
        ax.set_title(f"V2 raw-BIN replay | frame {frame} | {row['control_state']}\n"
                     f"lost={row['lost_frames']} | candidates={row['candidates']} | UID={row['uid']}")
        return scatter, target, trail

    animation = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    animation.save(dest / '163852_loss_replay.gif', writer=PillowWriter(fps=10), dpi=90)
    update(len(frames)//2)
    fig.savefig(dest / '163852_replay_preview.png', dpi=150)
    plt.close(fig)
    print(f'Created comparison, timeline and {len(frames)}-frame raw replay animation in {dest}')


if __name__ == '__main__':
    main()
