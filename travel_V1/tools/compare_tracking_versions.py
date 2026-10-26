"""Measure observation continuity, not human identity accuracy."""
import argparse
import collections
import csv
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def load(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def summarize(rows,selection):
    active=[r for r in rows if int(r['frame'])>selection]
    measured=[r for r in active if r['lock_valid']=='1' and r['measured']=='1'
              and r['association_source'] not in ('manual_lock_pending','reacquire_pending')]
    locked=[r for r in active if r['lock_valid']=='1']
    longest=run=0
    for r in active:
        run=0 if r in measured else run+1
        longest=max(longest,run)
    return dict(frames=len(rows),evaluated=len(active),accepted=len(measured),
        coverage=len(measured)/len(active),uid_changes=sum(a['uid']!=b['uid'] for a,b in zip(locked,locked[1:])),
        max_lost=max(int(r['lost_frames']) for r in active),max_unobserved_run=longest,
        unlocked=sum(r['lock_valid']=='0' for r in active),
        selection_success=all(r['selection_success']=='1' for r in rows if r['selection_success']!='-1'),
        selection_error=max(float(r['selection_error_m']) for r in rows if r['selection_success']!='-1'),
        rejects=dict(collections.Counter(r['reject_reason'] for r in active if r['reject_reason'])),
        pending_motion_violations=sum(r['association_source'] in ('manual_lock_pending','reacquire_pending','reacquire_pending_gap')
            and (r['allow_translation']=='1' or r['allow_yaw']=='1') for r in active),
        processing_sources=sorted({r['processing_source'] for r in rows}))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--tags',nargs='+',required=True)
    p.add_argument('--output',default='comparison_three_versions.json')
    p.add_argument('--manifest',type=Path,default=ROOT/'reports/audit/recordings.json')
    p.add_argument('--replay-root',type=Path,default=ROOT/'reports/replay')
    a=p.parse_args()
    audit=json.loads(a.manifest.read_text(encoding='utf-8'))
    records=[];totals={}
    for entry in audit['recordings']:
        stem=Path(entry['bin']).stem;record=dict(bin=entry['bin'])
        for tag in a.tags:
            record[tag]=summarize(load(a.replay_root/tag/(stem+'_frames.csv')),entry['selection_events'][0]['after_frame'])
        records.append(record)
    for tag in a.tags:
        totals[tag]={k:sum(r[tag][k] for r in records) for k in ('frames','evaluated','accepted','uid_changes','unlocked','pending_motion_violations')}
        totals[tag]['coverage']=totals[tag]['accepted']/totals[tag]['evaluated']
        totals[tag]['recordings_reaching_49_lost']=sum(r[tag]['max_lost']>=49 for r in records)
        totals[tag]['max_unobserved_run']=max(r[tag]['max_unobserved_run'] for r in records)
        print(tag,json.dumps(totals[tag]),flush=True)
    (ROOT/'reports'/a.output).write_text(json.dumps(dict(recordings=records,totals=totals,
        metric_scope='No video/person identity ground truth; coverage is accepted observations only.'),ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':main()
