"""Pillow fallback for current replay timeline and point animation."""
import collections
import math
from PIL import Image, ImageDraw


def render(args):
    from plot_replays import load
    rows = load(args.frames)
    if not rows:
        raise ValueError('Replay CSV contains no frames')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    image = Image.new('RGB', (1100, 760), 'white')
    draw = ImageDraw.Draw(image)
    draw.text((50, 15), args.frames.stem + ' - replay observations', fill='black')
    frames = [int(r['frame']) for r in rows]
    series = [('Filtered range (m)', [math.hypot(float(r['x']), float(r['y'])) if r['lock_valid']=='1' else None for r in rows]),
              ('Frames since acceptance', [int(r['lost_frames']) for r in rows]),
              ('Candidates', [int(r['candidates']) for r in rows])]
    for panel, (title, values) in enumerate(series):
        top = 55 + panel * 225
        draw.text((50, top), title, fill='black')
        draw.rectangle((60, top+25, 1050, top+190), outline='#cccccc')
        high = max([v for v in values if v is not None] + [1])
        run = []
        for frame, value in zip(frames, values):
            if value is None:
                if len(run)>1: draw.line(run, fill='#16827b', width=2)
                run=[]
            else:
                run.append((60+990*(frame-frames[0])/max(1,frames[-1]-frames[0]),top+190-165*value/high))
        if len(run)>1: draw.line(run, fill='#16827b', width=2)
        draw.text((60,top+194), f'{frames[0]*args.frame_period:g} s', fill='black')
        draw.text((980,top+194), f'{frames[-1]*args.frame_period:g} s', fill='black')
    image.save(args.output_dir/'timeline.png')
    if args.points:
        selected=[r for r in rows if int(r['frame'])>=args.start_frame][:args.max_frames]
        if not selected: raise ValueError('No frames selected for animation')
        points=collections.defaultdict(list)
        selected_ids={r['frame'] for r in selected}
        for point in load(args.points):
            if point['frame'] in selected_ids: points[point['frame']].append(point)
        images=[]
        def location(x,y): return (round(30+(x+18)*640/36),round(670-(y+4)*620/22))
        for row in selected:
            frame=Image.new('RGB',(700,720),'#0b1322');canvas=ImageDraw.Draw(frame)
            canvas.text((20,10),f"frame {row['frame']} | {row['control_state']} | UID={row['uid']}",fill='white')
            canvas.rectangle((30,50,670,670),outline='#8797aa')
            for point in points[row['frame']]:
                x,y=location(float(point['x']),float(point['y']))
                canvas.ellipse((x-2,y-2,x+2,y+2),fill='#2dd4bf' if point['is_moving']=='1' else '#8797aa')
            if row['lock_valid']=='1':
                x,y=location(float(row['x']),float(row['y']))
                canvas.ellipse((x-7,y-7,x+7,y+7),outline='#ffae48',width=2)
            images.append(frame)
        images[0].save(args.output_dir/'replay.gif',save_all=True,append_images=images[1:],duration=max(1,round(args.frame_period*1000)),loop=0)
    print(f'Plots saved to {args.output_dir}')


def main():
    from plot_replays import arguments
    render(arguments())


if __name__ == '__main__':
    main()
