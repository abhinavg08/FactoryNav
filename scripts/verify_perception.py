#!/usr/bin/env python3
"""Validate live camera calibration, optical TF, annotations, and person detections."""
import argparse
import json
from pathlib import Path
import statistics
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray


class PerceptionCheck(Node):
    def __init__(self):
        super().__init__('factorynav_perception_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.raw_stamps = set()
        self.annotated_stamps = set()
        self.detection_stamps = set()
        self.person_stamps = set()
        self.raw_count = 0
        self.detection_count = 0
        self.camera_info = None
        self.latencies = []
        self.last_metrics = None
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Image, '/camera/image_raw', self.raw, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info, qos_profile_sensor_data)
        self.create_subscription(Image, '/perception/annotated', self.annotated, 10)
        self.create_subscription(Detection2DArray, '/perception/detections', self.detections, 10)
        self.create_subscription(String, '/perception/metrics', self.metrics, 10)

    @staticmethod
    def stamp(message):
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def raw(self, message):
        self.raw_count += 1
        self.raw_stamps.add(self.stamp(message))
        if self.raw_count == 1:
            Path('log').mkdir(exist_ok=True)
            cv2.imwrite('log/camera_probe.png', self.bridge.imgmsg_to_cv2(message, 'bgr8'))

    def info(self, message):
        self.camera_info = message

    def annotated(self, message):
        self.annotated_stamps.add(self.stamp(message))

    def detections(self, message):
        self.detection_count += 1
        self.detection_stamps.add(self.stamp(message))
        if any(result.hypothesis.class_id == 'person' and result.hypothesis.score >= 0.35
               for detection in message.detections for result in detection.results):
            self.person_stamps.add(self.stamp(message))

    def metrics(self, message):
        self.last_metrics = json.loads(message.data)
        self.latencies.append(self.last_metrics['latency_ms'])

    def result(self):
        info = self.camera_info
        calibration = bool(info and info.width == 640 and info.height == 480 and info.k[0] > 0
                           and info.k[4] > 0 and info.header.frame_id == 'camera_rgb_optical_frame')
        optical_tf = bool(self.buffer.can_transform('base_link', 'camera_rgb_optical_frame', Time()))
        matched = len(self.raw_stamps & self.annotated_stamps & self.detection_stamps)
        return {'success': calibration and optical_tf and len(self.person_stamps) >= 3 and matched >= 3,
                'camera_messages': self.raw_count, 'detection_messages': self.detection_count,
                'distinct_person_frames': len(self.person_stamps), 'matched_image_detection_annotation_stamps': matched,
                'valid_camera_info': calibration, 'base_to_optical_tf': optical_tf,
                'median_inference_ms': round(statistics.median(self.latencies), 2) if self.latencies else None,
                'latest_metrics': self.last_metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90)
    args = parser.parse_args()
    rclpy.init()
    node = PerceptionCheck()
    deadline = time.monotonic() + args.timeout
    started = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - started >= 15 and node.result()['success']:
                break
        result = node.result()
        result['observation_wall_time_s'] = round(time.monotonic() - started, 2)
        Path('log/perception').mkdir(parents=True, exist_ok=True)
        Path('log/perception/verification.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        return 0 if result['success'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
