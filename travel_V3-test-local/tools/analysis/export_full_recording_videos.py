"""Export every replay frame as seekable H.264 MP4, from explicit current replay inputs."""
from _paths import ROOT, PATHS

import argparse
import collections
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
from urllib.parse import quote

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
SIZE = (1440, 900)
FULL_BOUNDS = (-18, 18, -4, 18)
ZOOM_BOUNDS = (-6, 6, -3, 9)
BG, GRID, WHITE = '#0b1322', '#243247', '#e5edf7'
MOVING, OTHER, TARGET = '#2dd4bf', '#8797aa', '#ffae48'
STATES = {'TRACKING': '接受观测', 'PREDICTED': '预测 / 待确认',
          'STOPPED': '停止运动建议', 'UNLOCKED': '未锁定'}


def csv_rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_replay(entry, version):
    stem = Path(entry['bin']).stem
    source = REPLAY_ROOT
    frame_path, point_path = source / (stem + '_frames.csv'), source / (stem + '_points.csv')
    rows = csv_rows(frame_path)
    expected = entry['bin_frames']
    frame_ids = [int(row['frame']) for row in rows]
    if len(frame_ids) != expected or frame_ids != sorted(set(frame_ids)):
        raise ValueError(f'Full sequential frame data required: {frame_path}')
    valid_frames = set(frame_ids)
    if any(row['dsp_ok'] != '1' for row in rows):
        raise ValueError(f'DSP failure present: {frame_path}')
    points = collections.defaultdict(list)
    for point in csv_rows(point_path):
        frame = int(point['frame'])
        x, y = float(point['x']), float(point['y'])
        if not (frame in valid_frames and math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'Invalid point: {point_path}, {point}')
        if not inside((x, y), FULL_BOUNDS):
            raise ValueError(f'Point outside full view; enlarge FULL_BOUNDS before export: {x}, {y}')
        points[frame].append((x, y, point['is_moving'] == '1'))
    for row in rows:
        if len(points[int(row['frame'])]) != int(row['points']):
            raise ValueError(f'Point count mismatch at {frame_path}:{row["frame"]}')
        if row['lock_valid'] == '1' and not inside((float(row['x']), float(row['y'])), FULL_BOUNDS):
            raise ValueError('Lock estimate outside full view; enlarge FULL_BOUNDS before export')
    return rows, points, frame_path, point_path


def inside(point, bounds):
    x, y = point
    xmin, xmax, ymin, ymax = bounds
    return xmin <= x <= xmax and ymin <= y <= ymax


class Renderer:
    def __init__(self, version):
        choices = [Path('C:/Windows/Fonts/msyh.ttc'),
                   Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')]
        font_path = next((path for path in choices if path.is_file()), None)
        if font_path is None:
            raise FileNotFoundError('A Chinese-capable font is needed (Microsoft YaHei or Noto Sans CJK)')
        self.fonts = {size: ImageFont.truetype(str(font_path), size) for size in (17, 20, 22, 25, 30)}
        self.background = Image.new('RGB', SIZE, BG)
        draw = ImageDraw.Draw(self.background)
        self.label(draw, (38, 20), f'毫米波雷达全量离线回放   |   {version.upper()} CPU', 30)
        self.label(draw, (42, 133), '完整雷达视野：保留所有导出点及锁定估计', 22)
        self.label(draw, (916, 300), '近场放大：超出此框的点仍显示在左图', 20)
        self.full = self.panel(draw, (65, 192, 867, 725), FULL_BOUNDS, 4)
        self.zoom = self.panel(draw, (920, 338, 1342, 760), ZOOM_BOUNDS, 3)
        self.label(draw, (100, 748), '横向 x（m）；纵向 y（m）为雷达前方；两轴等比例', 20)
        self.label(draw, (38, 807), '绿色：判为运动   灰色：其他点   橙色实圈：接受观测   橙色虚圈：预测 / 待确认', 20)
        self.label(draw, (38, 841), '雷达坐标，不是世界轨迹；锁定估计不等于身份真值；算法状态不代表实车动作。', 17, '#acbbce')

    def label(self, draw, xy, value, size=20, color=WHITE):
        draw.text(xy, str(value), font=self.fonts[size], fill=color)

    def panel(self, draw, box, bounds, tick):
        xmin, xmax, ymin, ymax = bounds
        left, top, right, bottom = box
        scale = min((right - left) / (xmax - xmin), (bottom - top) / (ymax - ymin))
        width, height = scale * (xmax - xmin), scale * (ymax - ymin)
        left += ((right - left) - width) / 2
        top += ((bottom - top) - height) / 2
        right, bottom = left + width, top + height
        def project(x, y):
            return left + (x - xmin) * scale, bottom - (y - ymin) * scale
        draw.rectangle((left, top, right, bottom), fill='#101d30', outline='#566780', width=2)
        for x in range(math.ceil(xmin / tick) * tick, int(xmax) + 1, tick):
            px, _ = project(x, 0)
            draw.line((px, top, px, bottom), fill=GRID, width=1)
            self.label(draw, (px - 12, bottom + 4), x, 17, '#acbbce')
        for y in range(math.ceil(ymin / tick) * tick, int(ymax) + 1, tick):
            _, py = project(0, y)
            draw.line((left, py, right, py), fill=GRID, width=1)
            self.label(draw, (left - 34, py - 12), y, 17, '#acbbce')
        px, py = project(0, 0)
        draw.polygon(((px, py - 9), (px - 7, py + 7), (px + 7, py + 7)), fill=WHITE)
        return project

    def render(self, row, points, history, entry, index, count, fps):
        picture = self.background.copy()
        draw = ImageDraw.Draw(picture)
        frame, total = int(row['frame']), entry['bin_frames']
        self.label(draw, (42, 65), f'录制 {index}/{count}：{entry["bin"]}', 22)
        estimate = '  [边界时间估计]' if row['motion_present'] == '0' else ''
        self.label(draw, (42, 100), f'原始帧 {frame}/{total}   播放 {(frame-1)/fps:.1f}/{total/fps:.1f} s   时间 {row["timestamp"]}{estimate}', 20)
        self.label(draw, (916, 148), f'算法状态：{STATES[row["control_state"]]}', 25,
                   MOVING if row['control_state'] == 'TRACKING' else TARGET)
        self.label(draw, (916, 190), f'点云 {len(points)}   CFAR {row["cfar_targets"]}   候选 {row["candidates"]}', 20)
        uid = row['uid'] if row['lock_valid'] == '1' else '—'
        self.label(draw, (916, 225), f'锁定 UID {uid}   连续丢失 {row["lost_frames"]} 帧', 20)
        self.label(draw, (916, 260), f'置信度 {float(row["confidence"]):.2f}   配套 CSV：' +
                   ('有记录' if row['motion_present'] == '1' else '此帧缺失'), 20)
        accepted = row['measured'] == '1' and row['association_source'] not in ('manual_lock_pending', 'reacquire_pending')
        for bounds, project, radius in ((FULL_BOUNDS, self.full, 2), (ZOOM_BOUNDS, self.zoom, 3)):
            segment = []
            for x, y in history:
                if inside((x, y), bounds):
                    segment.append(project(x, y))
                else:
                    if len(segment) >= 2:
                        draw.line(segment, fill='#805833', width=2)
                    segment = []
            if len(segment) >= 2:
                draw.line(segment, fill='#805833', width=2)
            for x, y, moving in points:
                if inside((x, y), bounds):
                    px, py = project(x, y)
                    draw.ellipse((px-radius, py-radius, px+radius, py+radius), fill=MOVING if moving else OTHER)
            target = float(row['x']), float(row['y'])
            if row['lock_valid'] == '1' and inside(target, bounds):
                px, py = project(*target)
                box = (px-10, py-10, px+10, py+10)
                if accepted:
                    draw.ellipse(box, outline=TARGET, width=3)
                else:
                    for angle in range(0, 360, 60):
                        draw.arc(box, angle, angle+35, fill=TARGET, width=3)
        draw.rectangle((38, 881, 1402, 888), fill=GRID)
        draw.rectangle((38, 881, 38 + 1364 * frame / total, 888), fill=MOVING)
        return picture


def validate_video(ffmpeg, path, expected_frames, expected_seconds):
    command = [ffmpeg, '-hide_banner', '-v', 'error', '-nostats', '-progress', 'pipe:1',
               '-i', str(path), '-map', '0:v:0', '-f', 'null', '-']
    result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8',
                            errors='replace', creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
    if result.returncode:
        raise RuntimeError(f'Video decode failed: {path}: {result.stderr[-2000:]}')
    progress = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    frames, seconds = int(progress['frame']), int(progress['out_time_us']) / 1_000_000
    if frames != expected_frames or abs(seconds - expected_seconds) > .011:
        raise ValueError(f'Decoded frame/time mismatch: {path}: {frames}, {seconds}')
    return dict(status='PASS', decoded_frames=frames, duration_seconds=seconds, bytes=path.stat().st_size)


def export_one(entry, index, count, version, fps, ffmpeg, destination):
    rows, points, frame_path, point_path = read_replay(entry, version)
    name = Path(entry['bin']).stem + '_' + version.upper() + '_full.mp4'
    output = destination / name
    partial = destination / (Path(name).stem + '.partial.mp4')
    command = [ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y', '-f', 'rawvideo',
               '-pix_fmt', 'rgb24', '-s', f'{SIZE[0]}x{SIZE[1]}', '-r', str(fps),
               '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'veryfast',
               '-crf', '20', '-pix_fmt', 'yuv420p', '-threads', '2',
               '-movflags', '+faststart', str(partial)]
    renderer, history = Renderer(version), collections.deque(maxlen=20)
    with (destination / (Path(name).stem + '_encode.log')).open('wb') as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log,
                                   creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        try:
            for row in rows:
                if row['lock_valid'] == '1':
                    history.append((float(row['x']), float(row['y'])))
                else:
                    history.clear()
                image = renderer.render(row, points[int(row['frame'])], history, entry, index, count, fps)
                process.stdin.write(image.tobytes())
                if int(row['frame']) % 250 == 0:
                    print(f'ENCODE {index}/{count} {entry["bin"]}: {row["frame"]}/{len(rows)}', flush=True)
            process.stdin.close()
            if process.wait() != 0:
                raise RuntimeError(f'Encoding failed; see {log.name}')
        except BaseException:
            if process.poll() is None:
                process.terminate()
                process.wait()
            raise
    validation = validate_video(ffmpeg, partial, len(rows), len(rows) / fps)
    partial.replace(output)
    preview = renderer.render(rows[len(rows)//2], points[int(rows[len(rows)//2]['frame'])], [], entry, index, count, fps)
    preview.save(destination / (Path(name).stem + '_preview.png'))
    return dict(index=index, recording=entry['bin'], video=name, fps=fps, frames=len(rows),
                empty_point_frames=sum(not points[int(row['frame'])] for row in rows),
                points=sum(len(values) for values in points.values()),
                first_timestamp=rows[0]['timestamp'], last_timestamp=rows[-1]['timestamp'],
                frame_csv_sha256=sha256(frame_path), points_csv_sha256=sha256(point_path),
                video_sha256=sha256(output), validation=validation)


def write_catalog(destination, records, merged, fps, version):
    offset = 0.0
    chapters = []
    lines = ['全部雷达录制回放', '', f'打开“{merged}”可连续查看，或打开每段单独 MP4。',
             '可用 Windows 媒体播放器、VLC，或浏览器打开“播放目录.html”。',
             f'播放速度：{fps:g} 帧/秒（原始采集速度）；无删帧，无音频。',
             f'来源：{version.upper()} 离线算法的真实雷达点云；不是摄像头录像。',
             '左图保留所有导出点和锁定估计，右图为近场放大。',
             '橙色虚圈包含预测或等待确认，不能当成真实身份或位置标注。',
             '片段间按录制顺序拼接，不补录制间隔，也不增加片头帧。', '', '合并版章节：']
    cards = []
    for record in records:
        seconds = record['validation']['duration_seconds']
        chapter = dict(start_seconds=offset, frames=record['frames'], recording=record['recording'])
        chapters.append(chapter)
        stamp = f'{int(offset)//60:02d}:{offset%60:04.1f}'
        lines.append(f'{stamp}  {record["recording"]}  {record["frames"]} 帧  {seconds:.1f} 秒')
        cards.append(f'<article><h2>{record["index"]:02d} · {html.escape(record["recording"])}</h2>'
                     f'<p>{record["frames"]} 帧 · {seconds:.1f} 秒 · 原速 {fps:g} Hz '
                     f'<button onclick="jump({offset})">在合并版跳至 {stamp}</button></p>'
                     f'<video controls preload="none" poster="{quote(Path(record["video"]).stem + "_preview.png")}" '
                     f'src="{quote(record["video"])}"></video></article>')
        offset += seconds
    (destination / '查看说明.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8-sig')
    page = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>全部雷达录制回放</title><style>body{background:#0b1322;color:#e5edf7;font:16px "Microsoft YaHei",sans-serif;'
            'margin:24px auto;max-width:1200px;padding:0 20px}h1{font-size:28px}h2{font-size:18px}video{width:100%;background:#000;'
            'max-height:75vh}article{padding:18px;margin:20px 0;background:#142136;border-radius:10px}p{line-height:1.7;color:#bccbdf}'
            'button,select{padding:6px 10px;margin-left:12px;cursor:pointer}a{color:#2dd4bf}</style>'
            '<h1>全部雷达录制回放</h1><p>10 组录制全部帧，按 10 Hz 原速播放。左图为完整视野，右图为近场放大。'
            f'这是 {version.upper()} 雷达点云回放，不是摄像头视频；预测圈与 UID 不是身份真值。</p>'
            '<article id="combined"><h2>连续合并版</h2><p>片段间不补时间间隔。播放速度'
            '<select onchange="document.getElementById(\'merged\').playbackRate=Number(this.value)">'
            '<option value="0.5">0.5 倍</option><option value="1" selected>1 倍</option>'
            '<option value="2">2 倍</option><option value="4">4 倍</option></select></p>'
            f'<video id="merged" controls preload="metadata" src="{quote(merged)}"></video></article>'
            + ''.join(cards) + '<script>function jump(seconds){const video=document.getElementById("merged");'
            'const seek=()=>{video.currentTime=seconds;document.getElementById("combined").scrollIntoView({behavior:"smooth"});};'
            'if(video.readyState>=1){seek();}else{video.addEventListener("loadedmetadata",seek,{once:true});video.load();}}'
            '</script></html>')
    (destination / '播放目录.html').write_text(page, encoding='utf-8')
    return chapters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=PATHS['data'] / 'reports/videos')
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--replay-dir', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--ffmpeg')
    parser.add_argument('--version', default='current', help='Display label for this replay')
    args = parser.parse_args()
    global REPLAY_ROOT
    REPLAY_ROOT = args.replay_dir.resolve()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    ffmpeg = args.ffmpeg or shutil.which('ffmpeg')
    if not ffmpeg:
        import imageio_ffmpeg
        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    audit = json.loads(args.manifest.read_text(encoding='utf-8'))
    entries, fps = audit['recordings'], 1000 / audit['dimensions']['frame_period_ms']
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    merged_name = f'全部录制_{args.version.upper()}_完整回放.mp4'
    outputs = [destination / (Path(e['bin']).stem + '_' + args.version.upper() + '_full.mp4') for e in entries] + [destination / merged_name]
    if any(path.exists() for path in outputs):
        raise FileExistsError('Export videos already exist; use a fresh --output-dir to preserve them')
    records = []
    with ThreadPoolExecutor(max_workers=min(args.jobs, 4)) as pool:
        tasks = [pool.submit(export_one, entry, index, len(entries), args.version, fps, ffmpeg, destination)
                 for index, entry in enumerate(entries, 1)]
        for task in as_completed(tasks):
            result = task.result()
            records.append(result)
            print(f'COMPLETE {len(records)}/{len(entries)} {result["recording"]} '
                  f'{result["frames"]} frames, {result["validation"]["duration_seconds"]:.1f}s', flush=True)
    records.sort(key=lambda record: record['index'])
    concat = destination / 'concat_inputs.txt'
    concat.write_text(''.join(f"file '{record['video']}'\n" for record in records), encoding='utf-8')
    partial = destination / f'全部录制_{args.version.upper()}_完整回放.partial.mp4'
    subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y', '-f', 'concat', '-safe', '0',
                    '-i', str(concat), '-c', 'copy', '-movflags', '+faststart', str(partial)], check=True,
                   creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
    total = sum(record['frames'] for record in records)
    merged_validation = validate_video(ffmpeg, partial, total, total / fps)
    partial.replace(destination / merged_name)
    chapters = write_catalog(destination, records, merged_name, fps, args.version)
    manifest = dict(version=args.version.upper(), source='real ADC replay outputs, not camera footage',
                    fps=fps, resolution=list(SIZE), frame_skipping=False, full_bounds=FULL_BOUNDS,
                    zoom_bounds=ZOOM_BOUNDS, recordings=records, merged_video=merged_name,
                    merged_validation=merged_validation, merged_sha256=sha256(destination / merged_name),
                    chapters=chapters)
    (destination / '导出验证清单.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'PASS: {len(records)} complete recordings + merged MP4, {total} frames, {total/fps:.1f}s in {destination}', flush=True)


if __name__ == '__main__':
    main()
