"""Read-only existing IPC inspector; never creates or removes a segment."""
from pathlib import Path
import argparse
import json
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/my_radar_follower'))
from my_radar_follower.odom_transport import decode
from my_radar_follower.runtime_support import shm_key
def main():
    import sysv_ipc
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=10);p.add_argument('--key',type=int,default=shm_key());args=p.parse_args()
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:
        memory=None
        try:
            memory=sysv_ipc.SharedMemory(args.key,flags=0)
            a=memory.read(32);b=memory.read(32)
            result=decode(a,time.time()) if a==b else dict(valid=False,status='inconsistent_snapshot')
            result.update(key=args.key,shmid=memory.id)
        except Exception as exc:result=dict(valid=False,status='ipc_unavailable',error=str(exc),key=args.key)
        finally:
            if memory is not None:memory.detach()
        print(json.dumps(result,ensure_ascii=False),flush=True);time.sleep(.2)
if __name__=='__main__':main()
