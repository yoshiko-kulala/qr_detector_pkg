from __future__ import annotations

from typing import List, Sequence, Tuple

import cv2
import numpy as np


_ROTATE_OPS = {
    0: lambda img: img,
    90: lambda img: cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE),
    180: lambda img: cv2.rotate(img, cv2.ROTATE_180),
    270: lambda img: cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE),
}

def rotate_image(image: np.ndarray, angle: int) -> np.ndarray:
    angle = angle % 360
    if angle not in _ROTATE_OPS:
        raise ValueError(f"Unsupported rotation angle: {angle}")
    return _ROTATE_OPS[angle](image)


def map_points_to_original(points: Sequence[Sequence[float]], angle: int, shape: Tuple[int, int]) -> np.ndarray:
    angle = angle % 360
    h, w = shape
    mapped: List[Tuple[float, float]] = []
    for x_rot, y_rot in points:
        if angle == 0:
            mapped.append((x_rot, y_rot))
        elif angle == 90:
            mapped.append(((w - 1) - y_rot, x_rot))
        elif angle == 180:
            mapped.append(((w - 1) - x_rot, (h - 1) - y_rot))
        elif angle == 270:
            mapped.append((y_rot, (h - 1) - x_rot))
        else:
            raise ValueError(f"Unsupported rotation angle: {angle}")
    return np.array(mapped, dtype=np.float32)


def flatten_points(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=np.float32)
    if arr.size == 0:
        return arr.reshape(0, 2)
    if arr.ndim >= 3:
        arr = arr.reshape(-1, 2)
    elif arr.ndim == 1:
        if arr.size % 2 != 0:
            raise ValueError('Point array length must be even to reshape into pairs')
        arr = arr.reshape(-1, 2)
    return arr


def encode_image(image: np.ndarray, fmt: str, quality: int) -> Tuple[bool, np.ndarray, str]:
    fmt = fmt.lower()
    ext = {
        "jpeg": ".jpg",
        "jpg": ".jpg",
        "png": ".png",
        "webp": ".webp",
    }.get(fmt, ".jpg")
    params: List[int] = []
    if ext in (".jpg", ".jpeg"):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    elif ext == ".png":
        compression = max(0, min(9, int((100 - min(max(quality, 0), 100)) / 10)))
        params = [int(cv2.IMWRITE_PNG_COMPRESSION), compression]
    success, encoded = cv2.imencode(ext, image, params)
    return success, encoded, fmt if fmt in {"jpeg", "jpg", "png", "webp"} else "jpeg"
