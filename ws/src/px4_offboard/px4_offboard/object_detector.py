#!/usr/bin/env python3
"""Run a YOLO model and tracker on the camera and publish the tracked objects."""

from collections import Counter
import os
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ObjectDetector(Node):
    """
    Detect and track objects in /camera/image_raw with an Ultralytics YOLO model.

    Every frame goes through the detector and a multi-object tracker (BoT-SORT by default,
    which compensates for camera motion). A track is published once it has been seen in
    min_hits frames, which drops brief false detections, and is labelled with the class it
    was detected as most often (the per-frame class flickers, e.g. car/van).

    Publishes vision_msgs/Detection2DArray on /detections (one per processed frame, with the
    frame's header; per object: id = track id, box in pixels, class_id = class name) and,
    while something subscribes, the frame with those boxes drawn on /camera/detections_image.
    """

    def __init__(self) -> None:
        super().__init__('object_detector')

        # A bare model name (e.g. yolo26n.pt) is looked up in model_dir; Ultralytics
        # downloads its pretrained models there on first use. Anything with a '/' is a path.
        self.declare_parameter('model', 'visdrone_yolo26s.pt')
        self.declare_parameter('model_dir', '~/.cache/yolo')
        # '' = first GPU if available, else CPU; or 'cpu', 'cuda:0', ...
        self.declare_parameter('device', '')
        # Lowest detection score passed to the tracker. Low, because the tracker uses weak
        # detections to keep existing tracks alive; it only starts a track at its own
        # new_track_thresh (0.25 in botsort.yaml / bytetrack.yaml).
        self.declare_parameter('confidence', 0.1)
        # Ultralytics tracker config: 'botsort.yaml', 'bytetrack.yaml' (no camera-motion
        # compensation, cheaper) or a path to a modified copy
        self.declare_parameter('tracker', 'botsort.yaml')
        # Frames a track must be seen in before it is published (20 = ~0.7 s at 30 Hz). In
        # Baylands, fragments at the frame edges had 6-18 hits, real targets 40+.
        self.declare_parameter('min_hits', 20)
        # Inference size (longest side, px). The camera is 1920x1200 and looks down, so
        # targets are small; 640 (Ultralytics' default) shrinks a person to a few pixels.
        self.declare_parameter('imgsz', 1280)
        # Class names to keep, e.g. ['car', 'truck', 'person']; unset = all
        self.declare_parameter('classes', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('report_period', 5.0)

        model = self.get_parameter('model').value
        if '/' not in model:
            model_dir = os.path.expanduser(self.get_parameter('model_dir').value)
            os.makedirs(model_dir, exist_ok=True)
            model = os.path.join(model_dir, model)
        self.device = self.get_parameter('device').value or (
            'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.confidence = self.get_parameter('confidence').value
        self.imgsz = self.get_parameter('imgsz').value
        self.tracker = self.get_parameter('tracker').value
        self.min_hits = self.get_parameter('min_hits').value

        self.model = YOLO(model)
        self.class_ids = None
        classes = self.get_parameter_or('classes').value  # None while unset
        if classes:
            ids = {name: i for i, name in self.model.names.items()}
            unknown = [c for c in classes if c not in ids]
            if unknown:
                raise ValueError(f'Classes not in {model}: {unknown}')
            self.class_ids = [ids[c] for c in classes]

        # The first prediction initialises CUDA and takes seconds; do it before any frames.
        self.model.predict(np.zeros((1200, 1920, 3), np.uint8), device=self.device,
                           imgsz=self.imgsz, verbose=False)

        # Same QoS as camera_monitor.py: RELIABLE, because BEST_EFFORT loses large frames.
        # Depth 1: if inference falls behind, process the newest frame and drop the rest.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()
        self.create_subscription(Image, topic, self.image_callback, image_qos)
        self.detections_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.image_pub = self.create_publisher(Image, '/camera/detections_image', image_qos)

        self.frame_count = 0
        self.inference_ms = 0.0
        self.callback_ms = 0.0
        self.last_counts = {}
        self.hits = {}  # track id -> frames seen
        self.class_votes = {}  # track id -> Counter of class ids it was detected as
        self.last_seen = {}  # track id -> frame number
        self.frame_number = 0
        self.last_report = time.monotonic()
        self.create_timer(self.get_parameter('report_period').value, self.report)
        self.get_logger().info(
            f'Detecting with {model} on {self.device} (imgsz {self.imgsz}, '
            f'confidence {self.confidence}, classes {classes or "all"}), tracking with '
            f'{self.tracker} (min_hits {self.min_hits}) from {topic}')

    def image_callback(self, msg: Image) -> None:
        """Detect and track objects in one frame and publish the confirmed tracks."""
        start = time.monotonic()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # persist=True keeps the tracker's state between calls. The result holds only
        # objects with a track (boxes.id); unmatched detections are dropped.
        result = self.model.track(
            frame, persist=True, tracker=self.tracker, device=self.device,
            imgsz=self.imgsz, conf=self.confidence, classes=self.class_ids,
            verbose=False)[0]

        self.frame_number += 1
        track_ids = [] if result.boxes.id is None else [int(i) for i in result.boxes.id]
        for track_id, cls in zip(track_ids, result.boxes.cls.tolist()):
            self.hits[track_id] = self.hits.get(track_id, 0) + 1
            self.last_seen[track_id] = self.frame_number
            self.class_votes.setdefault(track_id, Counter())[int(cls)] += 1
        # Forget tracks the tracker has long since dropped (it keeps lost ones track_buffer
        # = 30 frames by default), so the dicts don't grow forever.
        for track_id in [t for t, n in self.last_seen.items() if self.frame_number - n > 300]:
            del self.hits[track_id], self.last_seen[track_id], self.class_votes[track_id]
        confirmed = [i for i, t in enumerate(track_ids) if self.hits[t] >= self.min_hits]
        result = result[confirmed]
        track_ids = [track_ids[i] for i in confirmed]
        if track_ids:  # label each track with its majority class (last column of boxes.data)
            result.boxes.data[:, -1] = torch.tensor(
                [self.class_votes[t].most_common(1)[0][0] for t in track_ids],
                dtype=result.boxes.data.dtype, device=result.boxes.data.device)

        detections = Detection2DArray(header=msg.header)
        counts = {}
        for track_id, (cx, cy, w, h), cls, score in zip(
                track_ids, result.boxes.xywh.tolist(), result.boxes.cls.tolist(),
                result.boxes.conf.tolist()):
            name = self.model.names[int(cls)]
            counts[name] = counts.get(name, 0) + 1
            detection = Detection2D(header=msg.header, id=str(track_id))
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.detections_pub.publish(detections)

        # Drawing and sending a ~6.9 MB frame costs more than inference; skip it when unused.
        if self.image_pub.get_subscription_count():
            annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            annotated.header = msg.header
            self.image_pub.publish(annotated)

        self.frame_count += 1
        self.inference_ms += result.speed['inference']
        self.callback_ms += 1000 * (time.monotonic() - start)
        self.last_counts = counts

    def report(self) -> None:
        """Log the processing rate, mean inference and callback times and the latest detections."""
        now = time.monotonic()
        if self.frame_count:
            rate = self.frame_count / (now - self.last_report)
            found = ', '.join(f'{n} {c}' for n, c in sorted(self.last_counts.items()))
            self.get_logger().info(
                f'{rate:.1f} Hz, inference {self.inference_ms / self.frame_count:.1f} ms '
                f'(whole callback {self.callback_ms / self.frame_count:.1f} ms); '
                f'last frame: {found or "nothing"}')
        else:
            self.get_logger().warn('No camera frames received')
        self.frame_count = 0
        self.inference_ms = 0.0
        self.callback_ms = 0.0
        self.last_report = now


def main(args=None) -> None:
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        pass
    finally:
        object_detector.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
