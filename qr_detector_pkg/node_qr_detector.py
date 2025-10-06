from __future__ import annotations

import csv
import datetime
import os
import time
from typing import List, Sequence, Set, Tuple

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
        self.declare_parameter('log_enable', False)
        self.declare_parameter('log_root_dir', '~/qr_logs')
        self.declare_parameter('log_image_format', 'jpeg')
        self.declare_parameter('log_jpeg_quality', 95)
        self.declare_parameter('log_limit_per_frame', 1)
        self.declare_parameter('log_filename_sanitize_replace', '_')

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
        self.log_enable = bool(self.get_parameter('log_enable').value)
        log_root_dir = str(self.get_parameter('log_root_dir').value)
        log_format_param = str(self.get_parameter('log_image_format').value).lower()
        self.log_image_format = 'png' if log_format_param == 'png' else 'jpeg'
        jpeg_quality_param = int(self.get_parameter('log_jpeg_quality').value)
        self.log_jpeg_quality = max(0, min(100, jpeg_quality_param))
        limit_param = int(self.get_parameter('log_limit_per_frame').value)
        self.log_limit_per_frame = max(1, limit_param)
        replacement = str(self.get_parameter('log_filename_sanitize_replace').value)
        self.log_filename_sanitize_replace = replacement if replacement else '_'
        self.session_dir: str | None = None
        self.csv_path: str | None = None
        self.seen_texts: Set[str] = set()
        if self.log_enable:
            if not self._setup_logging(log_root_dir):
                self.log_enable = False

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

        if self.log_enable and decoded_texts:
            self._handle_logging(image, decoded_texts)

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

    def _setup_logging(self, root_dir_param: str) -> bool:
        root_dir = os.path.expanduser(root_dir_param)
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        session_dir = os.path.join(root_dir, timestamp)
        try:
            os.makedirs(session_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f'Failed to create log directory {session_dir}: {exc}')
            return False

        csv_path = os.path.join(session_dir, 'qr-list.csv')
        try:
            with open(csv_path, 'w', newline='', encoding='utf-8') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(['timestamp', 'text', 'filename'])
        except OSError as exc:
            self.get_logger().error(f'Failed to initialize log CSV {csv_path}: {exc}')
            return False

        self.session_dir = session_dir
        self.csv_path = csv_path
        return True

    def _handle_logging(self, raw_image: np.ndarray, decoded_texts: Sequence[str]) -> None:
        if not self.session_dir or not self.csv_path:
            return

        new_texts: List[str] = []
        for text in decoded_texts:
            if not text:
                continue
            if text in self.seen_texts:
                continue
            new_texts.append(text)

        if not new_texts:
            return

        success, encoded, fmt = encode_image(raw_image, self.log_image_format, self.log_jpeg_quality)
        if not success:
            self.get_logger().warn('Failed to encode raw image for logging')
            return

        extension = '.png' if fmt == 'png' else '.jpeg'
        limit = min(len(new_texts), self.log_limit_per_frame)
        for text in new_texts[:limit]:
            sanitized = self._sanitize_filename(text)
            filename = sanitized + extension
            file_path = os.path.join(self.session_dir, filename)
            try:
                with open(file_path, 'wb') as image_file:
                    image_file.write(encoded.tobytes())
            except OSError as exc:
                self.get_logger().warn(f'Failed to write log image {file_path}: {exc}')
                continue

            timestamp = datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat()
            try:
                with open(self.csv_path, 'a', newline='', encoding='utf-8') as csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow([timestamp, text, filename])
            except OSError as exc:
                self.get_logger().warn(f'Failed to append CSV log {self.csv_path}: {exc}')
            else:
                self.get_logger().info(f'Saved QR log: {text} -> {filename}')

            self.seen_texts.add(text)

    def _sanitize_filename(self, text: str) -> str:
        replace = self.log_filename_sanitize_replace
        invalid_chars = '\\/:*?"<>|\n\r\t'
        sanitized = ''.join(
            c if c not in invalid_chars and 31 < ord(c) < 127 else replace for c in text
        )
        sanitized = sanitized.strip().replace(' ', replace)
        if not sanitized:
            sanitized = f'QR_{int(time.time() * 1000)}'
        return sanitized


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
