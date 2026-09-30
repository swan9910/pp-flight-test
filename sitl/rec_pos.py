import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
rclpy.init(); n=Node('rec'); q=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,depth=1)
f=open('/tmp/ofb_track.csv','w'); f.write('t,x,y,z,armed\n'); st={'a':0}
n.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',lambda m: st.update(a=int(m.arming_state==2)),q)
last=[0]
def cb(m):
    t=time.time()
    if t-last[0]>=0.2: f.write(f'{t:.2f},{m.x:.3f},{m.y:.3f},{m.z:.3f},{st["a"]}\n'); f.flush(); last[0]=t
n.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position',cb,q)
t0=time.time()
while time.time()-t0<200: rclpy.spin_once(n,timeout_sec=0.1)
