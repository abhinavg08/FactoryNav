"""Run genuine YOLO26 inference on the latest simulated ROS camera image."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetector(Node):
    def __init__(self):
        super().__init__('factorynav_yolo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('weights', '/workspace/.cache/models/yolo26n.pt')
        self.declare_parameter('max_hz', 2.0)
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('image_size', 416)
        self.declare_parameter('device', 'auto')
        weights = Path(self.get_parameter('weights').value)
        weights.parent.mkdir(parents=True, exist_ok=True)
        torch.set_num_threads(2)
        selected = self.get_parameter('device').value
        self.device = ('cuda:0' if torch.cuda.is_available() else 'cpu') if selected == 'auto' else selected
        self.model = YOLO(str(weights))
        self.bridge = CvBridge()
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.latest = None
        self.received = 0
        self.processed = 0
        self.last_submitted = -1
        self.started = time.monotonic()
        self.best_person_score = 0.0
        self.output = Path('/workspace/log/perception')
        self.output.mkdir(parents=True, exist_ok=True)
        self.detections = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.annotated = self.create_publisher(Image, '/perception/annotated', 2)
        self.metrics = self.create_publisher(String, '/perception/metrics', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, '/camera/image_raw', self.receive_image, qos)
        self.create_timer(1.0 / float(self.get_parameter('max_hz').value), self.tick)
        self.get_logger().info(f'YOLO26 ready on {self.device}; latest-frame worker, no velocity publisher')

    def receive_image(self, message):
        self.received += 1
        self.latest = (self.received, message, time.monotonic())

    def image_age(self, message):
        stamp = message.header.stamp
        return self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)

    def infer(self, sequence, message):
        start = time.perf_counter()
        bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        result = self.model.predict(bgr, device=self.device,
                                    imgsz=int(self.get_parameter('image_size').value),
                                    conf=float(self.get_parameter('confidence').value),
                                    verbose=False)[0]
        boxes = [{'class': result.names[int(box.cls.item())], 'score': float(box.conf.item()),
                  'xyxy': box.xyxy[0].cpu().tolist()} for box in result.boxes]
        return sequence, message, bgr, result.plot(), boxes, (time.perf_counter() - start) * 1000

    def tick(self):
        if self.pending is not None and self.pending.done():
            try:
                self.publish_result(self.pending.result())
            except Exception as error:
                self.get_logger().error(f'Inference failed: {error}')
            self.pending = None
        if self.pending is not None or self.latest is None:
            return
        sequence, message, received_at = self.latest
        age = self.image_age(message)
        if (sequence == self.last_submitted or time.monotonic() - received_at > 1.0
                or age > 1.0 or age < -0.1):
            return
        self.last_submitted = sequence
        self.pending = self.worker.submit(self.infer, sequence, message)

    def publish_result(self, result):
        sequence, source, raw, annotated, boxes, latency = result
        if self.image_age(source) > 1.5:
            self.get_logger().warn('Discarding stale completed inference')
            return
        array = Detection2DArray()
        array.header = source.header
        for box in boxes:
            detection = Detection2D()
            detection.header = source.header
            x0, y0, x1, y1 = box['xyxy']
            detection.bbox.center.position.x = (x0 + x1) / 2
            detection.bbox.center.position.y = (y0 + y1) / 2
            detection.bbox.size_x = x1 - x0
            detection.bbox.size_y = y1 - y0
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = box['class']
            hypothesis.hypothesis.score = box['score']
            # No metric pose is estimated by this 2D detector.
            for diagonal in (0, 7, 14, 21, 28, 35):
                hypothesis.pose.covariance[diagonal] = 1e6
            detection.results.append(hypothesis)
            array.detections.append(detection)
        self.detections.publish(array)
        image = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        image.header = source.header
        self.annotated.publish(image)
        self.processed += 1
        metrics = {'device': self.device, 'received': self.received, 'processed': self.processed,
                   'source_sequence': sequence, 'latency_ms': round(latency, 2),
                   'observed_output_hz': round(self.processed / (time.monotonic() - self.started), 3),
                   'frame_id': source.header.frame_id, 'detections': boxes}
        message = String()
        message.data = json.dumps(metrics)
        self.metrics.publish(message)
        self.output.joinpath('latest_metrics.json').write_text(message.data + '\n')
        person_score = max((box['score'] for box in boxes if box['class'] == 'person'), default=0.0)
        if person_score > self.best_person_score:
            self.best_person_score = person_score
            cv2.imwrite(str(self.output / 'raw_person.png'), raw)
            cv2.imwrite(str(self.output / 'annotated_person.png'), annotated)
            self.output.joinpath('best_detection.json').write_text(json.dumps(metrics, indent=2) + '\n')
        if self.processed % 10 == 0:
            self.get_logger().info(f'{self.processed} frames; {latency:.0f} ms; {len(boxes)} detections')


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
