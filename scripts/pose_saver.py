#!/usr/bin/env python3
"""Keep ~/wheelbot/lastpose.json up to date with the AMCL pose, so autopatrol.sh can resume after a reboot.
Writes when the robot moved > 5 cm / 5 deg (or every 30 s), atomically so a power cut can't corrupt it."""
import json, math, os, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile

PATH = os.path.expanduser('~/wheelbot/lastpose.json')
last = {'x': 1e9, 'y': 1e9, 'yaw': 1e9, 't': 0.0}


def on_pose(m):
    p = m.pose.pose; q = p.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    moved = math.hypot(p.position.x - last['x'], p.position.y - last['y']) > 0.05
    turned = abs(math.remainder(yaw - last['yaw'], 2*math.pi)) > math.radians(5)
    if not (moved or turned or time.time() - last['t'] > 30):
        return
    tmp = PATH + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(dict(x=p.position.x, y=p.position.y, yaw=yaw, time=time.strftime('%F %T')), f)
        f.flush(); os.fsync(f.fileno())
    os.replace(tmp, PATH)
    last.update(x=p.position.x, y=p.position.y, yaw=yaw, t=time.time())
    print(f'saved {p.position.x:.2f} {p.position.y:.2f} {math.degrees(yaw):.0f} deg', flush=True)


rclpy.init()
n = rclpy.create_node('pose_saver')
n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_pose,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
