"""Local frame-aligned V2/V3 comparison from the complete validated MP4s."""
from pathlib import Path
import json
import subprocess
import sys
from export_full_recording_videos import validate_video
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'build/video-runtime'))
import imageio_ffmpeg
def main():
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    left=ROOT.parent/'V2-1013/reports/完整录制回放/全部录制_V2_完整回放.mp4'
    right=ROOT/'reports/完整录制回放/全部录制_V3_完整回放.mp4'
    output=ROOT/'reports/V2_V3全量并排对比.mp4'
    if output.exists():raise FileExistsError('Preserve existing comparison video; choose a new output path')
    with (ROOT/'reports/side_by_side_encode.log').open('wb') as log:
        subprocess.run([ffmpeg,'-hide_banner','-loglevel','warning','-y','-i',str(left),'-i',str(right),
            '-filter_complex','[0:v]scale=1280:800[a];[1:v]scale=1280:800[b];[a][b]hstack=inputs=2[v]',
            '-map','[v]','-an','-c:v','libx264','-preset','veryfast','-crf','20','-threads','3',
            '-movflags','+faststart',str(output)],check=True,stdout=subprocess.DEVNULL,stderr=log,
            creationflags=subprocess.CREATE_NO_WINDOW if sys.platform=='win32' else 0)
    result=validate_video(ffmpeg,output,6155,615.5)
    result.update(left='V2',right='V3',video=str(output))
    (ROOT/'reports/side_by_side_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
