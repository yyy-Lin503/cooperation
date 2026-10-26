"""Qt test peer only; never imports ROS or starts a chassis/controller."""
import argparse
import json
from pathlib import Path
import sys
import os
import subprocess

parser=argparse.ArgumentParser()
parser.add_argument('--runtime-dir',required=True)
parser.add_argument('--settings',required=True)
args=parser.parse_args()
folder=Path(args.runtime_dir)
(folder/'received-settings.json').write_text(args.settings,encoding='utf-8')
settings=json.loads(args.settings)
child=None
def identity(pid):
    fields=Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,start=fields[19],parent=int(fields[1]),group=int(fields[2]))

def emit(**value):print(json.dumps(value),flush=True)
emit(type='ready')
for line in sys.stdin:
    value=json.loads(line)
    with (folder/'commands.jsonl').open('a',encoding='utf-8') as stream:stream.write(line)
    if value['action']=='start':
        if settings['chassis_package']=='test_owned_child' and child is None:
            child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
            (folder/'owned.json').write_text(json.dumps(dict(manager=identity(os.getpid()),owned={'follow':[identity(child.pid)]})))
        emit(type='state',phase='following',components={'chassis':'owned','odom':'owned','follow':'owned'},
             odom_fresh=True,control={'reason':'waiting_for_target'})
    elif value['action']=='stop_follow':
        if child is not None:child.terminate();child.wait(timeout=3);child=None
        emit(type='state',phase='idle',components={'chassis':'owned','odom':'owned','follow':'off'},odom_fresh=True)
    elif value['action']=='shutdown':
        if child is not None:child.terminate();child.wait(timeout=3);child=None
        emit(type='state',phase='idle',components={},odom_fresh=False)
        break
