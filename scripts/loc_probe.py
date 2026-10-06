#!/usr/bin/env python3
"""Watch localization for N s: AMCL pose updates (jumps, spread), map->odom correction drift, odom vs EKF."""
import math, sys, time
import rclpy, tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile

T = float(sys.argv[1]) if len(sys.argv) > 1 else 40
def yaw(q): return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

rclpy.init(); n = rclpy.create_node('loc_probe')
amcl, odom, wodom = [], [], []
n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: amcl.append((time.time(), m)),
                      QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Odometry, 'odom', lambda m: odom.append(m), 10)
n.create_subscription(Odometry, 'wheel/odom', lambda m: wodom.append(m), 10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
corr = []
t0 = time.time(); last = 0
while time.time() - t0 < T:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last > 1.0:
        last = time.time()
        try:
            t = buf.lookup_transform('map', 'odom', rclpy.time.Time())
            corr.append((t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)))
        except Exception:
            pass

print(f'amcl updates: {len(amcl)} in {T:.0f}s')
prev = None
for ts, m in amcl[-15:]:
    p = m.pose.pose; c = m.pose.covariance
    x, y, th = p.position.x, p.position.y, yaw(p.orientation)
    jump = '' if prev is None else f'  step {math.hypot(x-prev[0], y-prev[1]):.3f} m {math.degrees(math.remainder(th-prev[2], 2*math.pi)):+.1f} deg'
    print(f'  {ts-t0:5.1f}s  x={x:7.3f} y={y:7.3f} yaw={math.degrees(th):6.1f}  std x/y/yaw={math.sqrt(c[0]):.3f}/{math.sqrt(c[7]):.3f}/{math.degrees(math.sqrt(c[35])):.1f}deg{jump}')
    prev = (x, y, th)
if len(corr) > 1:
    dx = max(c[0] for c in corr) - min(c[0] for c in corr); dy = max(c[1] for c in corr) - min(c[1] for c in corr)
    dth = max(c[2] for c in corr) - min(c[2] for c in corr)
    print(f'map->odom correction range over {T:.0f}s: x {dx:.3f} m, y {dy:.3f} m, yaw {math.degrees(dth):.1f} deg  (= how much odometry drifted and got corrected)')
if odom and wodom:
    print(f'odom (EKF) yaw now {math.degrees(yaw(odom[-1].pose.pose.orientation)):.1f} deg, wheel-only yaw {math.degrees(yaw(wodom[-1].pose.pose.orientation)):.1f} deg')
