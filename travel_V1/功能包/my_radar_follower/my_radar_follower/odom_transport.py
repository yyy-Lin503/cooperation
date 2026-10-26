"""ODM8 shared-memory codec and receiver state.

Timestamps use callback receipt time; ROS header age is diagnostic data.
"""
import math
import struct

MAGIC=0x384D444F
SIZE=32

def checksum(packet):
    value=2166136261
    for i,b in enumerate(packet):
        if i<20 or i>=24:value=((value^b)*16777619)&0xffffffff
    return value

def pack(stamp,vx,wz,seq,epoch):
    packet=bytearray(struct.pack('=dffIIII',stamp,vx,wz,seq,0,MAGIC,epoch))
    struct.pack_into('=I',packet,20,checksum(packet))
    return bytes(packet)

def decode(packet,now):
    if len(packet)!=SIZE:return dict(valid=False,status='wrong_size')
    stamp,vx,wz,seq,check,magic,epoch=struct.unpack('=dffIIII',packet)
    status='fresh'
    if magic!=MAGIC:status='protocol_mismatch'
    elif checksum(packet)!=check:status='checksum_mismatch'
    elif not all(math.isfinite(x) for x in (stamp,vx,wz)) or stamp<=0:status='invalid_value'
    elif now-stamp<-.05:status='future_timestamp'
    elif now-stamp>.5:status='stale'
    return dict(valid=status=='fresh',status=status,seq=seq,age_sec=now-stamp,
                timestamp_sec=stamp,vx_mps=vx,wz_radps=wz,writer_pid=epoch)

class SharedMemoryWriter:
    def __init__(self,key,factory):self.key=key;self.factory=factory;self.memory=None
    def write(self,packet):
        candidate=self.factory(self.key)
        if candidate.size!=SIZE:
            candidate.detach();raise RuntimeError('Existing IPC size mismatch; never deleting a segment')
        if self.memory is not None and self.memory.id==candidate.id:candidate.detach()
        else:
            if self.memory is not None:self.memory.detach()
            self.memory=candidate
        self.memory.write(packet)
    def close(self,invalid_packet=None):
        if self.memory is not None:
            try:
                if invalid_packet is not None:self.memory.write(invalid_packet)
            finally:self.memory.detach();self.memory=None

class ReceiverState:
    def __init__(self,writer,epoch):
        self.writer=writer;self.epoch=epoch;self.seq=0
        self.accepted=0;self.rejected=0;self.status='waiting_for_odom'
        self.last_monotonic=None;self.last_wall=None;self.source_age=None

    def receive(self,vx,wz,wall,monotonic,source_age=None):
        self.source_age=source_age
        try:
            if not all(math.isfinite(x) for x in (vx,wz,wall,monotonic)) or wall<=0:
                raise ValueError('nonfinite_odom_or_clock')
            seq=(self.seq+1)&0xffffffff
            # Count a sample as accepted only after a successful IPC write.
            self.writer(pack(wall,vx,wz,seq,self.epoch))
            self.seq=seq;self.accepted+=1;self.last_wall=wall;self.last_monotonic=monotonic;self.status='fresh'
            return True
        except Exception as exc:
            self.rejected+=1;self.status='write_or_sample_error: '+str(exc)
            return False

    def health(self,monotonic):
        age=None if self.last_monotonic is None else monotonic-self.last_monotonic
        status='stale' if age is not None and age>.5 else self.status
        return dict(status=status,seq=self.seq,accepted=self.accepted,rejected=self.rejected,
                    receipt_age_sec=age,source_header_age_sec=self.source_age,
                    timestamp_semantics='callback_receipt_wall_time',new_odom_motion_stop=False)
