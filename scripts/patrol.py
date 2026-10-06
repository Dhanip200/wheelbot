#!/usr/bin/env python3
"""Patrol points on a saved map. Nav2 must be running (start_nav.sh) and 2D Pose Estimate done.

Pick points once: python3 ~/wheelbot/patrol.py record
                  -> RViz top bar "Publish Point", click the 4 spots in patrol order
Patrol:           python3 ~/wheelbot/patrol.py              (loops forever, 5 s stop at each point)
                  python3 ~/wheelbot/patrol.py --once --wait 10 --timeout 90
Points live in ~/wheelbot/points.yaml (map x, y). On arrival the robot faces the next point.
Obstacles: Nav2 already steers around / replans. No progress for --stall s (default 10) -> the goal is
re-sent; a point that still fails after --retries extra tries (default 2) is skipped.
Stop with Ctrl+C or `pkill -INT -f patrol.py` (cancels the current goal; plain pkill does not).
"""
import argparse
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile

POINTS = os.path.expanduser('~/wheelbot/points.yaml')


def headings(pts):
    """Yaw at each point = direction to the next one (wraps to the first)."""
    return [math.atan2(pts[(i + 1) % len(pts)][1] - y, pts[(i + 1) % len(pts)][0] - x)
            for i, (x, y) in enumerate(pts)]


def record(nav, n):
    pts = []

    def on_click(m):
        pts.append((round(m.point.x, 3), round(m.point.y, 3)))
        print(f'point {len(pts)}: x={pts[-1][0]} y={pts[-1][1]}')

    nav.create_subscription(PointStamped, 'clicked_point', on_click, 10)
    print(f'RViz: "Publish Point" tool, click {n} points in order...')
    while len(pts) < n:
        rclpy.spin_once(nav, timeout_sec=0.2)
    with open(POINTS, 'w') as f:
        yaml.safe_dump({'points': [list(p) for p in pts]}, f)
    print(f'saved {POINTS}')


def load_goals():
    pts = [tuple(p) for p in yaml.safe_load(open(POINTS))['points']]
    goals = []
    for (x, y), yaw in zip(pts, headings(pts)):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goals.append(g)
    return pts, goals


def patrol(nav, wait, timeout, once, stall, retries):
    pts, goals = load_goals()
    # AMCL publishes amcl_pose (latched) only once a 2D Pose Estimate is given.
    got = []
    nav.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', got.append,
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    print('waiting for 2D Pose Estimate in RViz...')
    while not got:
        rclpy.spin_once(nav, timeout_sec=0.2)
    nav._waitForNodeToActivate('bt_navigator')  # Nav2 finishes starting ~10 s after the pose

    while True:
        # Re-read points every lap so `patrol.py record` (or editing points.yaml) applies without a restart.
        try:
            new_pts, new_goals = load_goals()
            if new_pts != pts:
                print(f'points.yaml changed -> {len(new_pts)} points')
            pts, goals = new_pts, new_goals
        except Exception as e:  # half-written / bad file: keep the old route
            print(f'points.yaml unreadable ({e}), keeping old points')
        for i, g in enumerate(goals, 1):
            print(f'-> point {i}')
            if go(nav, g, i, timeout, stall, retries):
                print(f'point {i} reached, waiting {wait}s')
                time.sleep(wait)
            else:
                print(f'point {i} skipped after {retries + 1} tries')
        if once:
            break


def go(nav, g, i, timeout, stall, retries):
    """Drive to one goal. No progress for `stall` s -> cancel and re-send; failed goals are retried."""
    for attempt in range(retries + 1):
        if attempt:
            print(f'point {i}: retry {attempt}/{retries}')
            time.sleep(1.0)
        g.header.stamp = nav.get_clock().now().to_msg()
        if not nav.goToPose(g):
            print(f'point {i} rejected by Nav2')
            continue
        t0 = t_prog = time.time()
        best = None
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None and fb.distance_remaining > 0.0:
                if best is None or fb.distance_remaining < best - 0.05:
                    best, t_prog = fb.distance_remaining, time.time()
            if time.time() - t_prog > stall:
                print(f'point {i}: no progress for {stall:.0f}s')
                nav.cancelTask()
                break
            if time.time() - t0 > timeout:
                print(f'point {i}: took over {timeout:.0f}s')
                nav.cancelTask()
                break
        if nav.getResult() == TaskResult.SUCCEEDED:
            return True
        print(f'point {i} attempt failed ({nav.getResult()})')
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', nargs='?', default='patrol', choices=['patrol', 'record'])
    ap.add_argument('--n', type=int, default=4, help='points to record')
    ap.add_argument('--wait', type=float, default=5.0, help='seconds to stop at each point')
    ap.add_argument('--timeout', type=float, default=90.0, help='give up on one try at a point after this')
    ap.add_argument('--stall', type=float, default=10.0, help='re-send the goal after this many s without progress')
    ap.add_argument('--retries', type=int, default=2, help='extra tries per point before skipping it')
    ap.add_argument('--once', action='store_true', help='stop after the last point')
    a = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator()  # no waitUntilNav2Active: with AMCL it would overwrite your pose estimate
    try:
        if a.mode == 'record':
            record(nav, a.n)
        else:
            patrol(nav, a.wait, a.timeout, a.once, a.stall, a.retries)
    except KeyboardInterrupt:
        nav.cancelTask()
        print('stopped')


if __name__ == '__main__':
    h = headings([(0, 0), (1, 0), (1, 1)])
    assert abs(h[0]) < 1e-9 and abs(h[1] - math.pi / 2) < 1e-9 and abs(h[2] + 3 * math.pi / 4) < 1e-9
    main()
