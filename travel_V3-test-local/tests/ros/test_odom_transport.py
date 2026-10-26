"""Pure tests without ROS, Linux IPC, cameras or a cart."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
def module(name):
    p=ROOT/'ros2_ws/src/my_radar_follower/my_radar_follower'/(name+'.py')
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
wire=module('odom_transport');support=module('runtime_support')
class TestOdom(unittest.TestCase):
    def test_roundtrip(self):
        r=wire.decode(wire.pack(100,.25,-.3,8,42),100.1)
        self.assertTrue(r['valid']);self.assertEqual(r['seq'],8);self.assertEqual(r['writer_pid'],42)
        self.assertAlmostEqual(r['wz_radps'],-.3,places=5)
    def test_stale(self):self.assertEqual(wire.decode(wire.pack(100,0,0,1,1),100.501)['status'],'stale')
    def test_future(self):self.assertEqual(wire.decode(wire.pack(100,0,0,1,1),99.9)['status'],'future_timestamp')
    def test_invalid(self):self.assertFalse(wire.decode(wire.pack(0,0,0,1,1),100)['valid'])
    def test_nan(self):self.assertFalse(wire.decode(wire.pack(100,float('nan'),0,1,1),100)['valid'])
    def test_checksum(self):
        b=bytearray(wire.pack(100,0,0,1,1));b[8]^=1;self.assertEqual(wire.decode(b,100)['status'],'checksum_mismatch')
    def test_wrong_protocol(self):
        b=bytearray(wire.pack(100,0,0,1,1));b[24]^=1;self.assertEqual(wire.decode(b,100)['status'],'protocol_mismatch')
    def test_legacy_rejected(self):self.assertFalse(wire.decode(bytes(24),100)['valid'])
    def test_stationary_still_updates(self):
        writes=[];s=wire.ReceiverState(writes.append,42)
        for i in range(20):self.assertTrue(s.receive(0,0,100+i*.05,10+i*.05))
        self.assertEqual(s.seq,20);self.assertTrue(wire.decode(writes[-1],101)['valid'])
    def test_source_header_is_diagnostic_not_new_gate(self):
        s=wire.ReceiverState(lambda p:None,42);self.assertTrue(s.receive(0,0,100,10,999))
        self.assertFalse(s.health(10)['new_odom_motion_stop'])
    def test_write_failure_not_accepted(self):
        def fail(packet):raise OSError('writer unavailable')
        s=wire.ReceiverState(fail,1);self.assertFalse(s.receive(0,0,100,10));self.assertEqual((s.seq,s.accepted,s.rejected),(0,0,1))
    def test_health_stale(self):
        s=wire.ReceiverState(lambda p:None,1);s.receive(0,0,100,10);self.assertEqual(s.health(11)['status'],'stale')
    def test_correct_source_path(self):
        with patch.dict('os.environ',{'RADAR_PROJECT_ROOT':str(ROOT)}):self.assertEqual(support.verify_loaded_package(support.__file__),ROOT)
    def test_wrong_source_path(self):
        with patch.dict('os.environ',{'RADAR_PROJECT_ROOT':str(ROOT)}):
            with self.assertRaises(RuntimeError):support.verify_loaded_package(ROOT.parent/'V4/ros2_ws/src/my_radar_follower/my_radar_follower/txt_tracker_bridge.py')
    def test_reserved_key_refused(self):
        with patch.dict('os.environ',{'RADAR_ODOM_SHM_KEY':'123456'}):
            with self.assertRaises(ValueError):support.shm_key()
    def test_writer_reconnects(self):
        class Segment:
            size=32
            def __init__(self,id):self.id=id;self.detached=False;self.packets=[]
            def detach(self):self.detached=True
            def write(self,p):self.packets.append(p)
        first=Segment(1);same=Segment(1);second=Segment(2)
        queue=iter([first,same,second]);writer=wire.SharedMemoryWriter(123458,lambda key:next(queue))
        writer.write(b'a');writer.write(b'b');self.assertTrue(same.detached)
        writer.write(b'c');self.assertTrue(first.detached);self.assertEqual(second.packets,[b'c'])
        writer.close(b'invalid');self.assertTrue(second.detached);self.assertEqual(second.packets[-1],b'invalid')
    def test_writer_size_mismatch_does_not_write(self):
        class Wrong:
            id=1;size=8
            def detach(self):pass
            def write(self,p):raise AssertionError('must not overwrite legacy IPC')
        writer=wire.SharedMemoryWriter(123458,lambda key:Wrong())
        with self.assertRaises(RuntimeError):writer.write(b'a')
if __name__=='__main__':unittest.main()
