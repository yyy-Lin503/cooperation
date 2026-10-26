"""Dependency-light numeric plots and point-cloud animation using Pillow."""

import csv
import os
from replay_inputs import resolve_input
import collections
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OLD, NEW, GREY = '#b66b29', '#16827b', '#888888'


def font(size):
    path = Path('C:/Windows/Fonts/arial.ttf')
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default(size=size)


def text(draw, xy, value, size=18, fill='#222222'):
    draw.text(xy, str(value), fill=fill, font=font(size))


def load(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def panel(draw, box, xmin, xmax, ymin, ymax):
    x0, y0, x1, y1 = box
    draw.line((x0, y0, x0, y1, x1, y1), fill='#333333', width=2)
    def project(x, y):
        return (x0 + (x - xmin) / (xmax - xmin) * (x1 - x0),
                y1 - (y - ymin) / (ymax - ymin) * (y1 - y0))
    return project


def curve(draw, rows, project, key, color, valid=None):
    segment = []
    for row in rows:
        value = float(row[key])
        if not math.isfinite(value) or (valid and not valid(row)):
            if len(segment) > 1: draw.line(segment, fill=color, width=2)
            segment = []
        else:
            segment.append(project(int(row['frame']) * .1, value))
    if len(segment) > 1: draw.line(segment, fill=color, width=2)


def main():
    dest = ROOT / 'reports/figures'
    dest.mkdir(parents=True, exist_ok=True)
    comparison = json.loads((ROOT / 'reports/comparison.json').read_text(encoding='utf-8'))
    audit = json.loads((ROOT / 'reports/audit/recordings.json').read_text(encoding='utf-8'))
    entries = comparison['recordings']
    image = Image.new('RGB', (1700, 690), 'white')
    draw = ImageDraw.Draw(image)
    text(draw, (65, 24), 'Full raw-BIN CPU replay: 10 recordings, 6155 frames per version', 27)
    text(draw, (100, 80), 'Accepted observations after selection (%)', 21)
    p = panel(draw, (100, 130, 800, 545), -.5, 9.5, 0, 100)
    for tick in range(0, 101, 20):
        y = p(0, tick)[1]
        draw.line((100, y, 800, y), fill='#dddddd')
        text(draw, (55, y-10), tick, 17)
    for index, entry in enumerate(entries):
        for version, delta, color in [('v1', -.19, OLD), ('v2', .19, NEW)]:
            centre, y = p(index+delta, 100 * entry[version]['accepted_fraction'])
            draw.rectangle((centre-11, y, centre+11, 545), fill=color)
        text(draw, (p(index, 0)[0]-22, 555), entry['bin'][9:13], 16)
    text(draw, (110, 598), 'Orange: V1-compatible     Green: V2 corrected', 18)
    text(draw, (110, 629), 'Observation coverage is not human identity accuracy.', 17)
    text(draw, (950, 80), 'FFT DC speed on stationary-platform frames', 21)
    q = panel(draw, (960, 130, 1595, 545), -.5, 1.5, -.35, .10)
    for tick in [-.3, -.2, -.1, 0.0, .1]:
        y = q(0, tick)[1]
        draw.line((960, y, 1595, y), fill='#dddddd')
        text(draw, (885, y-10), f'{tick:.1f}', 17)
    zero = q(0, 0)[1]
    draw.line((960, zero, 1595, zero), fill='#333333', width=2)
    for index, value, color, label in [(0, -.279017836, OLD, 'V1-compatible'), (1, 0, NEW, 'V2 corrected')]:
        centre, y = q(index, value)
        if value:
            draw.rectangle((centre-55, zero, centre+55, y), fill=color)
        else:
            draw.line((centre-55, zero, centre+55, zero), fill=color, width=8)
        text(draw, (centre-55, y+12 if value else y-34), f'{value:.3f} m/s', 19)
        text(draw, (centre-66, 555), label, 17)
    text(draw, (975, 598), '7490 DC points; no synthetic data used in this chart.', 17)
    image.save(dest / 'all_recordings_comparison.png')

    stem = '20260814_1638_52_564_raw'
    v1 = load(ROOT / 'reports/replay/v1' / (stem + '_frames.csv'))
    v2 = load(ROOT / 'reports/replay/v2' / (stem + '_frames.csv'))
    record = next(r for r in audit['recordings'] if Path(r['bin']).stem == stem)
    reference = load(resolve_input(record['diagnostic_csv'],ROOT/'reports/audit/recordings.json',os.environ.get('RADAR_REPLAY_DATA_ROOT'),'diagnostics'))
    for rows in (v1, v2):
        for row in rows: row['range'] = math.hypot(float(row['x']), float(row['y']))
    image = Image.new('RGB', (1600, 1060), 'white')
    draw = ImageDraw.Draw(image)
    text(draw, (70, 20), '16:38:52 recording: long loss persists after the Doppler correction', 27)
    text(draw, (100, 65), 'Orange V1 | Green V2 | Grey recorded V11 estimate (not ground truth)', 19)
    for idx, key, title, maximum in [(0, 'range', 'Filtered range (m)', 15),
                                     (1, 'lost_frames', 'Frames since an accepted observation', 50),
                                     (2, 'candidates', 'Visible candidates', 20)]:
        top = 135 + idx*285
        project = panel(draw, (100, top, 1530, top+205), 0, 116, 0, maximum)
        text(draw, (110, top-32), title, 20)
        for tick in [0, maximum/2, maximum]:
            y = project(0, tick)[1]
            draw.line((100, y, 1530, y), fill='#dddddd')
            text(draw, (50, y-10), f'{tick:g}', 16)
        for tick in range(0, 121, 20):
            if tick > 116: continue
            x, _ = project(tick, 0)
            text(draw, (x-10, top+215), tick, 16)
        if idx == 0:
            curve(draw, reference, project, 'filtered_range_m', GREY, lambda r:r['lock_valid']=='1')
        curve(draw, v1, project, key, OLD, (lambda r:r['lock_valid']=='1') if idx==0 else None)
        curve(draw, v2, project, key, NEW, (lambda r:r['lock_valid']=='1') if idx==0 else None)
    text(draw, (690, 1030), 'Recording time (seconds)', 18)
    image.save(dest / '163852_loss_timeline.png')

    start = max(1, next(int(r['frame']) for r in v2 if int(r['lost_frames']) >= 20)-25)
    frames = list(range(start, min(start+100, len(v2)+1)))
    points = collections.defaultdict(list)
    for row in load(ROOT / 'reports/replay/v2' / (stem + '_points.csv')):
        frame = int(row['frame'])
        if frame in frames:
            points[frame].append((float(row['x']), float(row['y']), int(row['is_moving'])))
    rows = {int(r['frame']):r for r in v2}
    images = []
    for frame in frames:
        row = rows[frame]
        picture = Image.new('RGB', (720, 780), 'white')
        draw = ImageDraw.Draw(picture)
        text(draw, (40, 20), f"V2 raw replay | frame {frame} | {row['control_state']}", 23)
        text(draw, (40, 54), f"lost={row['lost_frames']}  candidates={row['candidates']}  UID={row['uid']}", 20)
        project = panel(draw, (80, 115, 640, 715), -7, 7, 0, 15)
        for x in range(-6, 7, 2):
            px, _ = project(x, 0)
            draw.line((px, 115, px, 715), fill='#eeeeee')
            text(draw, (px-8, 719), x, 16)
        for y in range(0, 16, 3):
            _, py = project(0, y)
            draw.line((80, py, 640, py), fill='#eeeeee')
            text(draw, (43, py-8), y, 16)
        for x, y, moving in points[frame]:
            if not (-7 <= x <= 7 and 0 <= y <= 15): continue
            px, py = project(x, y)
            draw.ellipse((px-2, py-2, px+2, py+2), fill=NEW if moving else '#777777')
        if row['lock_valid'] == '1':
            x, y = float(row['x']), float(row['y'])
            if -7 <= x <= 7 and 0 <= y <= 15:
                px, py = project(x, y)
                draw.ellipse((px-9, py-9, px+9, py+9), outline='#d74b26', width=3)
        text(draw, (155, 754), 'Lateral x (m) | ring = filtered/predicted lock', 17)
        images.append(picture)
    images[0].save(dest / '163852_loss_replay.gif', save_all=True, append_images=images[1:],
                   duration=100, loop=0, optimize=False)
    images[len(images)//2].save(dest / '163852_replay_preview.png')
    print(f'Created 2 numeric figures and a {len(images)}-frame real point-cloud animation.')


if __name__ == '__main__':
    main()
