"""Plot a current replay timeline and optional point-cloud animation from explicit inputs."""
from _paths import ROOT, PATHS
import argparse
import collections
import csv
from pathlib import Path


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--points', type=Path, help='Optional replay point CSV for animation')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--frame-period', type=float, default=.1)
    parser.add_argument('--start-frame', type=int, default=0)
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    if args.frame_period <= 0 or args.max_frames <= 0:
        parser.error('frame-period and max-frames must be positive')
    return args


def load(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    args = arguments()
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.animation import FuncAnimation, PillowWriter
    except ImportError:
        from plot_replays_pillow import render
        render(args)
        return
    rows = load(args.frames)
    if not rows:
        raise ValueError('Replay CSV contains no frames')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    time = [int(row['frame']) * args.frame_period for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    distance = [np.hypot(float(r['x']), float(r['y'])) if r['lock_valid'] == '1' else np.nan for r in rows]
    for ax, values, title in zip(axes, (distance, [int(r['lost_frames']) for r in rows],
                                     [int(r['candidates']) for r in rows]),
                                ('Filtered range (m)', 'Frames since acceptance', 'Candidates')):
        ax.plot(time, values, color='#16827b')
        ax.set_ylabel(title)
        ax.grid(alpha=.2)
    axes[-1].set_xlabel('Source frame time (s)')
    fig.suptitle(args.frames.stem + ' — replay observations')
    fig.tight_layout()
    fig.savefig(args.output_dir / 'timeline.png', dpi=160)
    plt.close(fig)
    if args.points:
        selected = [r for r in rows if int(r['frame']) >= args.start_frame][:args.max_frames]
        if not selected:
            raise ValueError('No frames selected for animation')
        points = collections.defaultdict(list)
        selected_ids = {r['frame'] for r in selected}
        for point in load(args.points):
            if point['frame'] in selected_ids:
                points[point['frame']].append((float(point['x']), float(point['y']), point['is_moving'] == '1'))
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.set(xlim=(-18, 18), ylim=(-4, 18), xlabel='Radar lateral x (m)', ylabel='Radar forward y (m)')
        ax.grid(alpha=.2)
        scatter = ax.scatter([], [], s=12)
        target, = ax.plot([], [], 'o', markerfacecolor='none', markeredgecolor='#dd5426', markersize=10)
        def update(index):
            row = selected[index]
            values = points[row['frame']]
            scatter.set_offsets(np.array([(x,y) for x,y,_ in values]).reshape(-1,2))
            scatter.set_color(['#16827b' if moving else '#929292' for _,_,moving in values])
            target.set_data([float(row['x'])] if row['lock_valid'] == '1' else [],
                            [float(row['y'])] if row['lock_valid'] == '1' else [])
            ax.set_title(f"frame {row['frame']} | {row['control_state']} | lost={row['lost_frames']} | UID={row['uid']}")
            return scatter, target
        animation = FuncAnimation(fig, update, frames=len(selected), interval=args.frame_period*1000)
        animation.save(args.output_dir / 'replay.gif', writer=PillowWriter(fps=1/args.frame_period))
        plt.close(fig)
    print(f'Plots saved to {args.output_dir}')


if __name__ == '__main__':
    main()
