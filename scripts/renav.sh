#!/bin/bash
# Restart Nav2 (new params) keeping the robot's current AMCL pose, then restart patrol. Log: /tmp/renav.log
# Usage: bash renav.sh [map] [full]    full = also restart the robot drivers (needed after ekf.yaml changes)
MAP=${1:-test_map}
source /opt/ros/jazzy/setup.bash
echo "$(date +%T) stopping patrol (cancels its goal)"
pkill -INT -f '[p]atrol.py'; sleep 3
echo "$(date +%T) saving current pose"
python3 - <<'EOF' || exit 1
import json, math, time, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
rclpy.init(); n = rclpy.create_node('save_pose'); got = []
n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', got.append,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
while not got and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.2)
if not got: raise SystemExit('no amcl_pose - cannot keep the pose')
p = got[-1].pose.pose; q = p.orientation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
json.dump(dict(x=p.position.x, y=p.position.y, yaw=yaw), open('/tmp/lastpose.json', 'w'))
print('pose', round(p.position.x, 3), round(p.position.y, 3), round(math.degrees(yaw), 1), 'deg')
EOF
echo "$(date +%T) stopping RViz + Nav2"
pkill -f '[r]viz2'; pkill -f '[n]av2_container'; pkill -f '[b]ringup_launch.py'; sleep 3
if [ "$2" = full ]; then  # also restart the robot drivers (EKF/IMU/base/lidar), e.g. after editing ekf.yaml
  echo "$(date +%T) restarting robot drivers (keep STILL)"
  for p in bringup.launch base_node.py imu_node.py ekf_node rplidar static_transform_publisher; do pkill -f "[${p:0:1}]${p:1}"; done
  sleep 3
  python3 ~/wheelbot/lidar_check.py | tail -2
  setsid nohup ros2 launch ~/wheelbot/bringup.launch.py >/tmp/bring.log 2>&1 </dev/null &
  sleep 10
fi
echo "$(date +%T) starting Nav2 on $MAP"
setsid nohup ros2 launch nav2_bringup bringup_launch.py map:=$HOME/wheelbot/$MAP.yaml \
  params_file:=$HOME/wheelbot/nav2_params.yaml >/tmp/nav.log 2>&1 </dev/null &
sleep 40
python3 - <<'EOF'
import json, math, time, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
p = json.load(open('/tmp/lastpose.json'))
rclpy.init(); n = rclpy.create_node('set_pose')
pub = n.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = p['x'], p['y']
m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(p['yaw']/2), math.cos(p['yaw']/2)
c = [0.0]*36; c[0] = c[7] = 0.02; c[35] = 0.02; m.pose.covariance = c
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.2)
time.sleep(1)
for _ in range(2):
    m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m); rclpy.spin_once(n, timeout_sec=0.5)
print('initialpose sent to', pub.get_subscription_count(), 'subscriber(s)')
EOF
sleep 25
echo "$(date +%T) starting patrol"
cd ~/wheelbot; setsid nohup python3 -u patrol.py >/tmp/patrol.log 2>&1 </dev/null &
echo "$(date +%T) done"
