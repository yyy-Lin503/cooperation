"""ROS /odom -> IPC only. This executable cannot publish cmd_vel."""
from pathlib import Path
import csv
import json
import os
import time
from .runtime_support import runtime_paths, VERSION,verify_loaded_package,shm_key
from .odom_transport import SIZE,ReceiverState,SharedMemoryWriter,pack

def main(args=None):
    import fcntl
    import rclpy
    import sysv_ipc
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    root=verify_loaded_package(__file__)
    class Receiver(Node):
        def __init__(self):
            super().__init__('radar_odom_receiver')
            self.declare_parameter('odom_topic','/odom')
            self.declare_parameter('audit_dir',str(runtime_paths(root)['track']))
            self.folder=Path(self.get_parameter('audit_dir').value);self.folder.mkdir(parents=True,exist_ok=True)
            self.key=shm_key()
            self.ipc=SharedMemoryWriter(self.key,lambda key:sysv_ipc.SharedMemory(key,sysv_ipc.IPC_CREAT,size=SIZE))
            # All versions using an explicitly shared key respect the same lock.
            self.lock=Path('/tmp',f'radar_odom_writer_{self.key}.lock').open('a')
            try:fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except OSError:raise RuntimeError(f'Another receiver owns key {self.key}; not overwriting it')
            self.state=ReceiverState(self.write,os.getpid())
            self.audit_file=(self.folder/f'odom_samples_{time.time_ns()}.csv').open('x',encoding='utf-8',newline='')
            self.audit=csv.writer(self.audit_file);self.audit.writerow(['receipt_wall_sec','source_ros_sec','vx_mps','wz_radps','seq','accepted','status'])
            self.topic=self.get_parameter('odom_topic').value
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.subscription=self.create_subscription(Odometry,self.topic,self.callback,qos)
            self.timer=self.create_timer(.2,self.health)
            self.reported=None
            self.get_logger().info(f'{VERSION} ODOM ONLY: {self.topic}, key={self.key}, loaded={Path(__file__).resolve()}, no motion publisher')

        def write(self,packet):
            self.ipc.write(packet) # re-resolves the key, reconnects after recreation

        def callback(self,msg):
            wall=time.time();mono=time.monotonic();source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            source_age=self.get_clock().now().nanoseconds*1e-9-source if source>0 else None
            vx=float(msg.twist.twist.linear.x);wz=float(msg.twist.twist.angular.z)
            ok=self.state.receive(vx,wz,wall,mono,source_age)
            self.audit.writerow([wall,source,vx,wz,self.state.seq,int(ok),self.state.status])

        def health(self):
            h=self.state.health(time.monotonic());h.update(version=VERSION,shm_key=self.key,
                topic=self.topic,resolved_topic=self.resolve_topic_name(self.topic),
                publisher_count=self.count_publishers(self.resolve_topic_name(self.topic)),wall_time_sec=time.time(),
                loaded_module=str(Path(__file__).resolve()),motion_output=False)
            try:
                tmp=self.folder/'odom_health.json.tmp';tmp.write_text(json.dumps(h,ensure_ascii=False,indent=2),encoding='utf-8');tmp.replace(self.folder/'odom_health.json')
                self.audit_file.flush()
            except OSError as exc:self.get_logger().error(f'Odom audit failed: {exc}')
            if h['status']!=self.reported:
                self.get_logger().warning(f"Odom {h['status']}: seq={h['seq']} age={h['receipt_age_sec']}; diagnostic only, no new motion gate")
                self.reported=h['status']

        def close(self):
            self.ipc.close(pack(0,0,0,self.state.seq,os.getpid()))
            self.audit_file.close();self.lock.close()
    rclpy.init(args=args);node=None
    try:node=Receiver();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
