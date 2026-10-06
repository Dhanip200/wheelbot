#!/usr/bin/env python3
"""Drive the robot a short distance straight ahead (+) or back (-) via Nav2. Usage: nudge.py 0.5"""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros

d = max(-1.0, min(1.0, float(sys.argv[1])))  # never more than 1 m
rclpy.init()
nav = BasicNavigator('nudge')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, nav)
t0 = time.time()
while not buf.can_transform('map', 'base_link', rclpy.time.Time()) and time.time() - t0 < 10:
    rclpy.spin_once(nav, timeout_sec=0.2)
tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
q = tf.rotation; yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
g = PoseStamped(); g.header.frame_id = 'map'; g.header.stamp = nav.get_clock().now().to_msg()
g.pose.position.x = tf.translation.x + d*math.cos(yaw); g.pose.position.y = tf.translation.y + d*math.sin(yaw)
g.pose.orientation = q
print(f'from {tf.translation.x:.2f},{tf.translation.y:.2f} to {g.pose.position.x:.2f},{g.pose.position.y:.2f}', flush=True)
nav.goToPose(g)
t0 = time.time()
while not nav.isTaskComplete():
    rclpy.spin_once(nav, timeout_sec=0.2)
    if time.time() - t0 > 30: nav.cancelTask(); break
print('result:', nav.getResult())
