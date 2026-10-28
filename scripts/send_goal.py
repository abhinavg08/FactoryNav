#!/usr/bin/env python3
"""Send a real Nav2 action goal and save observed sensor/navigation evidence."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class NavigationCheck(Node):
    def __init__(self):
        super().__init__('factorynav_navigation_check', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.scan_count = 0
        self.odom_count = 0
        self.plan_count = 0
        self.amcl_count = 0
        self.minimum_range = math.inf
        self.path_length = 0.0
        self.last_xy = None
        self.latest_plan = None
        self.max_abs_plan_y = 0.0
        self.max_abs_odom_y = 0.0
        self.obstacle_spawned = False
        self.obstacle_costmap_seen = False
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_scan(self, message):
        self.scan_count += 1
        valid = [value for value in message.ranges
                 if math.isfinite(value) and message.range_min <= value <= message.range_max]
        if valid:
            self.minimum_range = min(self.minimum_range, min(valid))

    def on_odom(self, message):
        self.odom_count += 1
        point = message.pose.pose.position
        xy = (point.x, point.y)
        self.max_abs_odom_y = max(self.max_abs_odom_y, abs(point.y))
        if self.last_xy is not None:
            delta = math.dist(xy, self.last_xy)
            if delta < 1.0:
                self.path_length += delta
        self.last_xy = xy

    def on_plan(self, message):
        self.plan_count += 1
        self.latest_plan = message
        if message.poses:
            self.max_abs_plan_y = max(self.max_abs_plan_y,
                                     max(abs(pose.pose.position.y) for pose in message.poses))

    def on_costmap(self, message):
        # This test region is free in the generated static factory map and
        # intersects only the unexpected blockage. The global costmap uses map
        # coordinates with zero map-origin yaw, as configured by factory.yaml.
        if not self.obstacle_spawned or message.header.frame_id != 'map':
            return
        info = message.info
        x0 = max(0, int((-0.5 - info.origin.position.x) / info.resolution))
        x1 = min(info.width, int((0.5 - info.origin.position.x) / info.resolution) + 1)
        y0 = max(0, int((-0.8 - info.origin.position.y) / info.resolution))
        y1 = min(info.height, int((0.8 - info.origin.position.y) / info.resolution) + 1)
        if x0 >= x1 or y0 >= y1:
            return
        for row in range(y0, y1):
            values = message.data[row * info.width + x0:row * info.width + x1]
            if values and max(values) >= 99:
                self.obstacle_costmap_seen = True
                return

    def on_amcl(self, _message):
        self.amcl_count += 1

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def lifecycle(self, name):
        client = self.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                return 'unavailable'
            future = client.call_async(GetState.Request())
            if not self.spin_until(future.done, 4.0) or future.result() is None:
                return 'timeout'
            return future.result().current_state.label
        finally:
            self.destroy_client(client)

    def evidence(self):
        return {
            'scan_messages': self.scan_count,
            'odom_messages': self.odom_count,
            'global_plan_messages': self.plan_count,
            'amcl_pose_messages': self.amcl_count,
            'observed_odom_path_length_m': round(self.path_length, 3),
            'minimum_observed_lidar_range_m': (
                round(self.minimum_range, 3) if math.isfinite(self.minimum_range) else None),
            'map_to_base_link': self.tf_buffer.can_transform('map', 'base_link', Time()),
            'odom_to_base_link': self.tf_buffer.can_transform('odom', 'base_link', Time()),
            'simulation_time_s': self.get_clock().now().nanoseconds / 1e9,
            'maximum_abs_global_plan_y_m': round(self.max_abs_plan_y, 3),
            'maximum_abs_odom_y_m': round(self.max_abs_odom_y, 3),
            'unexpected_obstacle_spawned': self.obstacle_spawned,
            'unexpected_obstacle_marked_in_global_costmap': self.obstacle_costmap_seen,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--initial-x', type=float, default=-2.0)
    parser.add_argument('--initial-y', type=float, default=-0.5)
    parser.add_argument('--initial-yaw', type=float, default=0.0)
    parser.add_argument('--skip-initial-pose', action='store_true')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--output', type=Path, default=Path('log/navigation_result.json'))
    parser.add_argument('--obstacle-after-m', type=float,
                        help='Factory test only: spawn blockage after this much observed odometry travel')
    args = parser.parse_args()
    rclpy.init()
    node = NavigationCheck()
    result = {'goal': {'x': args.x, 'y': args.y, 'yaw': args.yaw},
              'success': False, 'phase': 'startup'}
    exit_code = 1
    handle = None
    try:
        if not node.spin_until(lambda: node.scan_count >= 3 and node.odom_count >= 3
                               and node.get_clock().now().nanoseconds > 0, 90.0):
            raise RuntimeError('Timed out waiting for real /scan, /odom, and simulation clock')
        if not args.skip_initial_pose:
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'map'
            initial.pose.pose.position.x = args.initial_x
            initial.pose.pose.position.y = args.initial_y
            initial.pose.pose.orientation.z = math.sin(args.initial_yaw / 2)
            initial.pose.pose.orientation.w = math.cos(args.initial_yaw / 2)
            initial.pose.covariance[0] = 0.05
            initial.pose.covariance[7] = 0.05
            initial.pose.covariance[35] = 0.05
            for _ in range(10):
                initial.header.stamp = node.get_clock().now().to_msg()
                node.initial_pose.publish(initial)
                node.spin_until(lambda: False, 0.2)
        result['phase'] = 'localization'
        if not node.spin_until(lambda: node.tf_buffer.can_transform('map', 'base_link', Time()), 45.0):
            raise RuntimeError('map -> base_link TF is unavailable after initial pose')
        lifecycle_deadline = time.monotonic() + 60.0
        while True:
            lifecycle = {name: node.lifecycle(name) for name in (
                'amcl', 'map_server', 'planner_server', 'controller_server', 'bt_navigator')}
            if all(state == 'active' for state in lifecycle.values()):
                break
            if time.monotonic() >= lifecycle_deadline:
                break
            node.spin_until(lambda: False, 1.0)
        result['lifecycle'] = lifecycle
        if any(state != 'active' for state in lifecycle.values()):
            raise RuntimeError(f'Nav2 lifecycle nodes not all active: {lifecycle}')
        if not node.action.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('/navigate_to_pose action is unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2)
        result['phase'] = 'navigation'
        node.path_length = 0.0
        node.max_abs_odom_y = 0.0
        node.max_abs_plan_y = 0.0
        started = time.monotonic()
        goal_future = node.action.send_goal_async(goal)
        if not node.spin_until(goal_future.done, 15.0):
            raise RuntimeError('Goal acceptance timed out')
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('Nav2 rejected the navigation goal')
        action_result = handle.get_result_async()
        def finished():
            if (args.obstacle_after_m is not None and not node.obstacle_spawned
                    and node.path_length >= args.obstacle_after_m):
                spawn = subprocess.run(
                    [sys.executable, str(Path(__file__).with_name('spawn_obstacle.py'))],
                    capture_output=True, text=True, timeout=15, check=False)
                if spawn.returncode != 0:
                    raise RuntimeError(f'Obstacle spawn failed: {spawn.stdout} {spawn.stderr}')
                result['obstacle_spawn'] = json.loads(spawn.stdout)
                result['obstacle_spawn_wall_time_s'] = round(time.monotonic() - started, 3)
                result['obstacle_spawn_odom_travel_m'] = round(node.path_length, 3)
                node.obstacle_spawned = True
            return action_result.done()
        if not node.spin_until(finished, args.timeout):
            cancellation = handle.cancel_goal_async()
            node.spin_until(cancellation.done, 5.0)
            raise RuntimeError('Navigation timed out; cancellation requested')
        outcome = action_result.result()
        result['completion_wall_time_s'] = round(time.monotonic() - started, 3)
        result['action_status'] = outcome.status
        result['nav2_error_code'] = outcome.result.error_code
        result['success'] = outcome.status == GoalStatus.STATUS_SUCCEEDED
        if args.obstacle_after_m is not None:
            result['unexpected_obstacle_scenario_passed'] = (
                result['success'] and node.obstacle_spawned and node.obstacle_costmap_seen
                and node.max_abs_plan_y > 3.2 and node.max_abs_odom_y > 3.2)
            result['success'] = result['unexpected_obstacle_scenario_passed']
        result['phase'] = 'complete'
        exit_code = 0 if result['success'] else 1
    except Exception as error:
        result['error'] = str(error)
        node.get_logger().error(str(error))
        if handle is not None and handle.accepted:
            cancel = handle.cancel_goal_async()
            node.spin_until(cancel.done, 5.0)
    finally:
        result.update(node.evidence())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
