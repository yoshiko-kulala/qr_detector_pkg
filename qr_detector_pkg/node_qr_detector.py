from __future__ import annotations

import csv
import datetime
import os
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
        self._log_root_dir = str(self.get_parameter('log_root_dir').value)
        self._log_image_format = str(self.get_parameter('log_image_format').value).lower()
        self._log_jpeg_quality = int(self.get_parameter('log_jpeg_quality').value)
        self.log_limit_per_frame = max(0, int(self.get_parameter('log_limit_per_frame').value))
        self._log_filename_replace = str(self.get_parameter('log_filename_sanitize_replace').value)

        self.encode_format = encode_format
        self.jpeg_quality = jpeg_quality
        self.rotate_attempts = rotate_attempts
        self.skip_rate = skip_rate
        self.min_process_interval = 0.0 if max_fps <= 0 else 1.0 / max_fps
        self.last_processed_time = 0.0
        self.frame_counter = 0
        self._warned_pyzbar = False
        self._log_image_format = self._normalize_log_format(self._log_image_format)
        self._log_jpeg_quality = self._clamp_quality(self._log_jpeg_quality)
        self._log_filename_replace = self._sanitize_replace_str(self._log_filename_replace)
        self._log_extension = '.png' if self._log_image_format == 'png' else '.jpg'
        self._log_imwrite_params = self._build_log_imwrite_params()
        self.log_session_dir = None
        self.log_csv_path = None
        self.seen_texts = set()
        if self.log_enable:
            self._setup_logging()

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
        raw_image = image.copy()
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
            self._save_qr_logs(raw_image, decoded_texts)
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

    def _setup_logging(self) -> None:
        root_dir = os.path.expanduser(self._log_root_dir)
        timestamp = datetime.datetime.now(datetime.timezone.utc).astimezone().strftime("%Y-%m-%d-%H-%M-%S")
        session_dir = os.path.join(root_dir, timestamp)
        try:
            os.makedirs(session_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f"Failed to create log directory '{session_dir}': {exc}")
            self.log_enable = False
            return

        csv_path = os.path.join(session_dir, "qr-list.csv")
        try:
            with open(csv_path, "w", newline='', encoding='utf-8') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["timestamp", "text", "filename"])
        except OSError as exc:
            self.get_logger().error(f"Failed to initialize log CSV '{csv_path}': {exc}")
            self.log_enable = False
            return

        self.log_session_dir = session_dir
        self.log_csv_path = csv_path
        self.get_logger().info(f"Logging detections to '{session_dir}'")

    def _save_qr_logs(self, image: np.ndarray, texts: Sequence[str]) -> None:
        if not self.log_enable or not self.log_session_dir or not texts:
            return

        if self.log_limit_per_frame == 0:
            return

        new_texts = [text for text in texts if text and text not in self.seen_texts]
        if not new_texts:
            return

        limit = self.log_limit_per_frame
        if limit > 0:
            new_texts = new_texts[:limit]

        for text in new_texts:
            filename = self._build_log_filename(text)
            file_path = os.path.join(self.log_session_dir, filename)
            if not self._write_log_image(file_path, image):
                self.get_logger().warn(f"Failed to save log image for '{text}'")
                self.seen_texts.add(text)
                continue

            if not self._append_log_csv(self._current_timestamp(), text, filename):
                self.get_logger().warn(f"Failed to append log CSV for '{text}'")

            self.seen_texts.add(text)

    def _build_log_filename(self, text: str) -> str:
        if not self.log_session_dir:
            return f"qr{self._log_extension}"

        base = self._sanitize_filename(text)
        candidate = base
        counter = 1
        path = os.path.join(self.log_session_dir, f"{candidate}{self._log_extension}")
        while os.path.exists(path):
            candidate = f"{base}{self._log_filename_replace}{counter}"
            path = os.path.join(self.log_session_dir, f"{candidate}{self._log_extension}")
            counter += 1
        return f"{candidate}{self._log_extension}"

    def _write_log_image(self, path: str, image: np.ndarray) -> bool:
        try:
            ok = cv2.imwrite(path, image, self._log_imwrite_params)
        except cv2.error as exc:
            self.get_logger().warn(f"cv2 error while saving '{path}': {exc}")
            return False
        if not ok:
            self.get_logger().warn(f"cv2.imwrite returned False for '{path}'")
        return ok

    def _append_log_csv(self, timestamp: str, text: str, filename: str) -> bool:
        if not self.log_csv_path:
            return False
        try:
            with open(self.log_csv_path, "a", newline='', encoding='utf-8') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([timestamp, text, filename])
            return True
        except OSError as exc:
            self.get_logger().warn(f"Failed to write to CSV '{self.log_csv_path}': {exc}")
            return False

    def _sanitize_filename(self, value: str) -> str:
        replace = self._log_filename_replace or '_'
        safe_chars = []
        for char in value:
            if char.isalnum() or char in ('-', '_'):
                safe_chars.append(char)
            else:
                safe_chars.append(replace)
        name = ''.join(safe_chars).strip().lstrip('.')
        if not name:
            name = 'qr'
        return name[:128]

    def _normalize_log_format(self, value: str) -> str:
        if value in {'png'}:
            return 'png'
        return 'jpeg'

    @staticmethod
    def _clamp_quality(value: int) -> int:
        return max(0, min(100, int(value)))

    def _build_log_imwrite_params(self) -> List[int]:
        if self._log_image_format == 'png':
            compression = max(0, min(9, int((100 - self._log_jpeg_quality) / 10)))
            return [int(cv2.IMWRITE_PNG_COMPRESSION), compression]
        return [int(cv2.IMWRITE_JPEG_QUALITY), self._log_jpeg_quality]

    def _sanitize_replace_str(self, replace: str) -> str:
        if not replace:
            return '_'
        if any(sep in replace for sep in ('/', '\\')):
            return '_'
        return replace

    @staticmethod
    def _current_timestamp() -> str:
        return datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat()


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
