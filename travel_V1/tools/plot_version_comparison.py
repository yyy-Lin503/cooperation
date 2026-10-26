"""Draw the actual full-ADC comparison and the geometry rejection example."""
import csv
import json
import math
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1]
import os
FONT=next((p for p in [os.environ.get('RADAR_FONT',''), 'C:/Windows/Fonts/msyh.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'] if p and Path(p).is_file()), None)
COLORS={'v1':'#b87932','v2':'#568ac5','v3':'#16877c'}
def label(draw,xy,text,size=20,color='#263447'):
    draw.text(xy,str(text),font=(ImageFont.truetype(FONT,size) if FONT else ImageFont.load_default()),fill=color)
def load(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def main():
    data=json.loads((ROOT/'reports/comparison_three_versions.json').read_text(encoding='utf-8'))
    dest=ROOT/'reports/figures';dest.mkdir(parents=True,exist_ok=True)
    im=Image.new('RGB',(1560,860),'white');d=ImageDraw.Draw(im)
    label(d,(60,22),'同批原始 ADC 全量回放：V1、V2 与 V3',30)
    label(d,(65,74),'10 组录制，每版 6155 帧；统计人工选择之后的 5765 帧',22)
    x0,y0,x1,y1=100,160,1470,635
    for tick in range(0,101,20):
        y=y1-(y1-y0)*tick/100;d.line((x0,y,x1,y),fill='#dae1e8')
        label(d,(45,y-14),str(tick)+'%',18)
    for i,r in enumerate(data['recordings']):
        x=x0+(i+.5)*(x1-x0)/10
        for v,delta in [('v1',-27),('v2',0),('v3',27)]:
            y=y1-(y1-y0)*r[v]['coverage'];d.rectangle((x+delta-11,y,x+delta+11,y1),fill=COLORS[v])
        label(d,(x-32,y1+12),r['bin'][9:13],20)
    for i,v in enumerate(('v1','v2','v3')):
        total=data['totals'][v];x=100+i*440
        label(d,(x,710),f'{v.upper()}  有效观测 {total["coverage"]:.2%}',25,COLORS[v])
        label(d,(x,755),f'UID 变化 {total["uid_changes"]} 次；解锁帧 {total["unlocked"]}',20)
    label(d,(100,811),'有效观测覆盖率不是身份准确率；V3 默认不扩大距离门限。',22)
    im.save(dest/'三版本全量回放对比.png')
    stem='20260814_1638_52_564_raw'
    im=Image.new('RGB',(1560,980),'white');d=ImageDraw.Draw(im)
    label(d,(60,24),'1638 片段：当前帧尺寸与三帧尺寸的差异',30)
    label(d,(65,80),'第 406 帧的簇中心距 V2 预测约 0.03 m，却因三帧宽度 2.093 m 超过 2 m 被过滤',21)
    labels=[('v2','V2 旧尺寸'),('v3','V3 当前帧尺寸')]
    for i,(v,title) in enumerate(labels):
        rows=load(ROOT/'reports/replay'/v/(stem+'_clusters.csv'))
        visible=[r for r in rows if int(r['frame'])==406 and math.hypot(float(r['x'])+1.03,float(r['y'])-2.18)<.5]
        row=visible[0];x=90+i*740
        label(d,(x,145),title,27,COLORS[v])
        label(d,(x,197),f'宽 {float(row["width"]):.3f} m；深 {float(row["depth"]):.3f} m',23)
        label(d,(x,238),'候选保留' if row['person_like']=='1' else '候选被删除',23,COLORS[v])
    for panel,(key,title,ymax) in enumerate([('y','锁定位置前向 y（m）',5),('lost_frames','连续缺测帧',50)]):
        top=350+panel*270;bottom=top+190
        label(d,(100,top-35),title,21)
        ymin=-1 if key=='y' else 0
        for tick in ([ymin,0,2.5,5] if key=='y' else [0,25,50]):
            y=bottom-(bottom-top)*(tick-ymin)/(ymax-ymin);d.line((100,y,1480,y),fill='#dae1e8');label(d,(45,y-12),f'{tick:g}',18)
        for v,_ in labels:
            rows=load(ROOT/'reports/replay'/v/(stem+'_frames.csv'));segment=[]
            for r in rows:
                frame=int(r['frame'])
                if not 390<=frame<=510:continue
                if r['lock_valid']!='1':
                    if len(segment)>1:d.line(segment,fill=COLORS[v],width=3)
                    segment=[];continue
                x=100+(frame-390)/120*1380;y=bottom-(float(r[key])-ymin)/(ymax-ymin)*(bottom-top)
                segment.append((x,max(top,min(bottom,y))))
            if len(segment)>1:d.line(segment,fill=COLORS[v],width=3)
        for frame in [390,410,430,450,470,490,510]:label(d,(100+(frame-390)/120*1380-20,bottom+7),frame,18)
    label(d,(100,920),'蓝：V2；绿：V3。锁定位置仍是估计，并非独立真值；人物身份需要视频或标签验证。',21)
    im.save(dest/'1638丢锁与尺寸诊断.png')
if __name__=='__main__':main()
