from __future__ import annotations

import time
from typing import List, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .utils import encode_image, flatten_points, map_points_to_original, rotate_image

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    PYZBAR_AVAILABLE = True
except ImportError:  # pragma: no cover - optional dependency
    pyzbar_decode = None
    PYZBAR_AVAILABLE = False


class QRDetector(Node):
    def __init__(self) -> None:
        super().__init__('qr_detector')

        self.declare_parameter('input_image_topic', '/image_in/compressed')
        self.declare_parameter('output_image_topic', '/image_out/compressed')
        self.declare_parameter('text_topic', '/qr/text')
        self.declare_parameter('encode_format', 'jpeg')
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('rotate_attempts', [0, 90, 180, 270])
        self.declare_parameter('use_pyzbar_fallback', True)
        self.declare_parameter('draw_box', True)
        self.declare_parameter('draw_text', True)
        self.declare_parameter('max_fps', 30)
        self.declare_parameter('skip_rate', 1)
        self.declare_parameter('qos_reliability', 'best_effort')

        input_topic = self.get_parameter('input_image_topic').value
        output_topic = self.get_parameter('output_image_topic').value
        text_topic = self.get_parameter('text_topic').value
        encode_format = self.get_parameter('encode_format').value
        jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        rotate_attempts = self._sanitize_angles(self.get_parameter('rotate_attempts').value)
        self.use_pyzbar_fallback = bool(self.get_parameter('use_pyzbar_fallback').value)
        self.draw_box = bool(self.get_parameter('draw_box').value)
        self.draw_text = bool(self.get_parameter('draw_text').value)
        max_fps = float(self.get_parameter('max_fps').value)
        skip_rate = max(1, int(self.get_parameter('skip_rate').value))
        reliability_param = str(self.get_parameter('qos_reliability').value).lower()

        self.encode_format = encode_format
        self.jpeg_quality = jpeg_quality
        self.rotate_attempts = rotate_attempts
        self.skip_rate = skip_rate
        self.min_process_interval = 0.0 if max_fps <= 0 else 1.0 / max_fps
        self.last_processed_time = 0.0
        self.frame_counter = 0
        self._warned_pyzbar = False

        qos_profile = QoSProfile(depth=10, reliability=self._parse_reliability(reliability_param))

        self.sub_image = self.create_subscription(
            CompressedImage, input_topic, self.image_callback, qos_profile
        )
        self.pub_image = self.create_publisher(CompressedImage, output_topic, qos_profile)
        self.pub_text = self.create_publisher(String, text_topic, qos_profile)

        self.detector = cv2.QRCodeDetector()

    def image_callback(self, msg: CompressedImage) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.skip_rate != 0:
            return

        now = time.monotonic()
        if self.min_process_interval > 0.0 and (now - self.last_processed_time) < self.min_process_interval:
            return
        self.last_processed_time = now

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('Failed to decode incoming compressed image')
            return

        decoded_texts: List[str] = []
        annotated = image.copy()

        for angle in self.rotate_attempts:
            rotated = rotate_image(image, angle)
            texts, polygons = self._detect_with_opencv(rotated)

            if self.use_pyzbar_fallback:
                texts, polygons = self._augment_with_pyzbar(rotated, texts, polygons)

            if not texts:
                continue

            for text in texts:
                if text not in decoded_texts:
                    decoded_texts.append(text)

            if self.draw_box or self.draw_text:
                annotated = self._draw_annotations(annotated, polygons, texts, angle, image.shape[:2])

        if decoded_texts:
            text_msg = String()
            text_msg.data = '\n'.join(decoded_texts)
            self.pub_text.publish(text_msg)

        success, buffer, fmt = encode_image(annotated, self.encode_format, self.jpeg_quality)
        if not success:
            self.get_logger().warn('Failed to encode annotated image')
            return

        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = fmt
        out_msg.data = buffer.tobytes()
        self.pub_image.publish(out_msg)

    def _detect_with_opencv(self, image: np.ndarray) -> Tuple[List[str], List[np.ndarray]]:
        texts: List[str] = []
        polygons: List[np.ndarray] = []
        try:
            retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(image)
        except cv2.error:
            retval = False
            decoded_info = []
            points = None

        if retval and decoded_info:
            iter_points = points if points is not None else [None] * len(decoded_info)
            for text, pts in zip(decoded_info, iter_points):
                if not text:
                    continue
                texts.append(text)
                if pts is not None:
                    polygons.append(flatten_points(np.array(pts, dtype=np.float32)))

        if texts:
            return texts, polygons

        try:
            text, pts, _ = self.detector.detectAndDecode(image)
        except cv2.error:
            text, pts = '', None
        if text:
            texts.append(text)
            if pts is not None and len(pts) > 0:
                polygons.append(flatten_points(np.array(pts, dtype=np.float32)))
        return texts, polygons

    def _augment_with_pyzbar(
        self,
        image: np.ndarray,
        texts: Sequence[str],
        polygons: Sequence[np.ndarray],
    ) -> Tuple[List[str], List[np.ndarray]]:
        results = list(texts)
        polys = list(polygons)

        if not PYZBAR_AVAILABLE:
            if not self._warned_pyzbar:
                self.get_logger().warn('pyzbar is not available; fallback disabled')
                self._warned_pyzbar = True
            return results, polys

        barcodes = pyzbar_decode(image)
        for barcode in barcodes:
            text = barcode.data.decode('utf-8', errors='ignore')
            if not text:
                continue
            if text not in results:
                results.append(text)
            if barcode.polygon:
                pts = np.array([[p.x, p.y] for p in barcode.polygon], dtype=np.float32)
                polys.append(pts)
        return results, polys

    def _draw_annotations(
        self,
        image: np.ndarray,
        polygons: Sequence[np.ndarray],
        texts: Sequence[str],
        angle: int,
        shape: Tuple[int, int],
    ) -> np.ndarray:
        annotated = image.copy()
        for idx, pts in enumerate(polygons):
            pts = flatten_points(np.array(pts, dtype=np.float32))
            if pts.size == 0:
                continue
            mapped = map_points_to_original(pts, angle, shape)
            mapped = mapped.reshape(-1, 1, 2).astype(int)
            if self.draw_box:
                cv2.polylines(annotated, [mapped], True, (0, 255, 0), 2)
            if self.draw_text and idx < len(texts):
                text = texts[idx]
                text_pos = mapped[0, 0]
                cv2.putText(
                    annotated,
                    text,
                    (int(text_pos[0]), int(text_pos[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
        return annotated

    @staticmethod
    def _sanitize_angles(raw_angles: Sequence[int]) -> List[int]:
        angles: List[int] = []
        for angle in raw_angles:
            try:
                normalized = int(angle) % 360
            except (TypeError, ValueError):
                continue
            if normalized not in (0, 90, 180, 270):
                continue
            if normalized not in angles:
                angles.append(normalized)
        if not angles:
            angles.append(0)
        return angles

    @staticmethod
    def _parse_reliability(value: str) -> QoSReliabilityPolicy:
        mapping = {
            'best_effort': QoSReliabilityPolicy.BEST_EFFORT,
            'reliable': QoSReliabilityPolicy.RELIABLE,
            'system_default': QoSReliabilityPolicy.SYSTEM_DEFAULT,
        }
        return mapping.get(value, QoSReliabilityPolicy.BEST_EFFORT)


def main(args=None) -> None:  # pragma: no cover - ROS 2 entry point
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
