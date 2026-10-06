#!/bin/bash
# Resume the patrol after a (re)boot: robot + Nav2 (no RViz) on a saved map, pose from ~/wheelbot/lastpose.json
# (kept up to date by pose_saver.py), then patrol.py. Started at boot by the wheelbot-patrol user service.
# Usage: bash autopatrol.sh [map_name]   Log: /tmp/autopatrol.log
# Boot autostart off/on:  systemctl --user disable wheelbot-patrol  /  systemctl --user enable wheelbot-patrol
# The robot must not be moved while it is off; if it was, use start_nav.sh + 2D Pose Estimate instead.
MAP=${1:-event}
source /opt/ros/jazzy/setup.bash
cd ~/wheelbot
[ -f lastpose.json ] || { echo "no lastpose.json - start by hand with start_nav.sh"; exit 1; }
# No RTC: wait (up to 2 min) for the clock to sync, a later time jump can abort Nav2 startup.
for i in $(seq 60); do [ "$(timedatectl show -p NTPSynchronized --value)" = yes ] && break; sleep 2; done
echo "$(date +%T) clock synced: $(timedatectl show -p NTPSynchronized --value)"
pkill -9 -f '[p]atrol.py'; pkill -f '[p]ose_saver.py'
bash ~/wheelbot/stop_all.sh
sleep 2
echo "$(date +%T) lidar check"; python3 ~/wheelbot/lidar_check.py | tail -2
[ -f nav2_params.yaml ] || bash make_nav2_params.sh || exit 1
echo "$(date +%T) starting robot (keep STILL)"
setsid nohup ros2 launch ~/wheelbot/bringup.launch.py >/tmp/bring.log 2>&1 </dev/null &
sleep 10
echo "$(date +%T) starting Nav2 on $MAP"
setsid nohup ros2 launch nav2_bringup bringup_launch.py map:=$HOME/wheelbot/$MAP.yaml \
  params_file:=$HOME/wheelbot/nav2_params.yaml >/tmp/nav.log 2>&1 </dev/null &
sleep 40
python3 - <<'EOF' || exit 1
import json, math, os, time, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
p = json.load(open(os.path.expanduser('~/wheelbot/lastpose.json')))
print('last pose', round(p['x'], 3), round(p['y'], 3), round(math.degrees(p['yaw']), 1), 'deg, saved', p.get('time'))
rclpy.init(); n = rclpy.create_node('set_pose')
pub = n.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = p['x'], p['y']
m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(p['yaw']/2), math.cos(p['yaw']/2)
c = [0.0]*36; c[0] = c[7] = 0.04; c[35] = 0.03; m.pose.covariance = c
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.2)
time.sleep(1)
for _ in range(2):
    m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m); rclpy.spin_once(n, timeout_sec=0.5)
print('initialpose sent to', pub.get_subscription_count(), 'subscriber(s)')
EOF
sleep 25
echo "$(date +%T) starting pose saver + patrol"
setsid nohup python3 -u ~/wheelbot/pose_saver.py >/tmp/pose_saver.log 2>&1 </dev/null &
setsid nohup python3 -u ~/wheelbot/patrol.py >/tmp/patrol.log 2>&1 </dev/null &
echo "$(date +%T) done - follow with: tail -f /tmp/patrol.log"
