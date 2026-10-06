#!/bin/bash
# Headless patrol: robot + Nav2 (no RViz) on a saved map, pose from a scan match, then patrol.py.
# Usage: bash patrol_headless.sh [map_name] [last_x last_y radius]   Log: /tmp/headless.log
# The scan match only searches within radius m of (last_x, last_y): long corridors look alike.
MAP=${1:-test_map}
PRIOR="${2:--4.5} ${3:-2.74} ${4:-6}"
source /opt/ros/jazzy/setup.bash
cd ~/wheelbot
pkill -f '[p]atrol.py'
bash ~/wheelbot/stop_all.sh
pkill -f '[n]av2_container'; pkill -f '[c]omponent_container'
sleep 2
echo "$(date +%T) lidar check"; python3 ~/wheelbot/lidar_check.py | tail -2
echo "$(date +%T) starting robot (keep STILL)"
setsid nohup ros2 launch ~/wheelbot/bringup.launch.py >/tmp/bring.log 2>&1 </dev/null &
sleep 10
echo "$(date +%T) starting Nav2 on $MAP"
setsid nohup ros2 launch nav2_bringup bringup_launch.py map:=$HOME/wheelbot/$MAP.yaml \
  params_file:=$HOME/wheelbot/nav2_params.yaml >/tmp/nav.log 2>&1 </dev/null &
sleep 40
echo "$(date +%T) matching scan to map"
python3 ~/wheelbot/plan_patrol.py 4.0 $PRIOR >/tmp/plan.out 2>&1 || { echo "scan match failed"; tail -5 /tmp/plan.out; exit 1; }
python3 - <<'EOF' || exit 1
import json, math, sys, time, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
plan = json.load(open('/tmp/plan.json')); p, r2 = plan['pose'], plan['runner_up']
print('scan match:', p, ' runner-up:', r2)
if p['score'] < 0.75:
    sys.exit('NOT STARTING: match too weak - set 2D Pose Estimate in RViz instead')
if r2 and p['score'] - r2['score'] < 0.05:
    sys.exit('NOT STARTING: two places match about equally - set 2D Pose Estimate in RViz instead')
rclpy.init(); n = rclpy.create_node('set_pose')
pub = n.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = p['x'], p['y']
yaw = math.radians(p['yaw_deg']); m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
c = [0.0]*36; c[0] = c[7] = 0.04; c[35] = 0.03; m.pose.covariance = c
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.2)
time.sleep(1)
for _ in range(2):
    m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m); rclpy.spin_once(n, timeout_sec=0.5)
print('initialpose sent to', pub.get_subscription_count(), 'subscriber(s)')
EOF
sleep 25
echo "$(date +%T) starting patrol"
setsid nohup python3 -u ~/wheelbot/patrol.py >/tmp/patrol.log 2>&1 </dev/null &
echo "$(date +%T) done - follow with: tail -f /tmp/patrol.log"
