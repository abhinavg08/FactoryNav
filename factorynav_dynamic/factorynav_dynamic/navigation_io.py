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
