from __future__ import annotations

from typing import List, Tuple, Optional
import cv2 as cv
import numpy as np


def pointInRect(point: Tuple[int, int] | None, rect: Tuple[int, int, int, int] | None) -> bool:
    """Return True if point (x,y) lies strictly inside rectangle (x,y,w,h)."""
    if point is None or rect is None:
        return False
    x1, y1, w, h = rect
    x2, y2 = x1 + w, y1 + h
    x, y = point
    return (x1 < x < x2) and (y1 < y < y2)


def GetPreviewImage(large_image: np.ndarray) -> Tuple[np.ndarray, int, int]:
    """Resize to 640x480 then crop to show gate frame area; returns (image, height, width)."""
    preview_image = cv.resize(large_image.copy(), (640, 480))
    image_height, image_width = preview_image.shape[:2]

    # Crop region: keep entire resized image but retain hook points for future tuning
    y1 = 0
    y2 = image_height
    x1 = 0
    x2 = image_width
    preview_image = preview_image[y1:y2, x1:x2].copy()
    image_height, image_width = preview_image.shape[:2]
    return preview_image, image_height, image_width


def ProcessImage(
    large_image: np.ndarray,
    centre_box: List[int],
    draw_rects: bool = True,
    exposure_level: float = -8.0,
    lower_threshold: int = 150,
) -> Tuple[np.ndarray, Optional[Tuple[float, float]], Optional[np.ndarray]]:
    """Find sprocket hole contour and return preview image, centre point, and box.

    - large_image: BGR frame (OpenCV format)
    - centre_box: [x, y, w, h] target region
    - draw_rects: draw overlays when True
    - exposure_level, lower_threshold: preserved for compatibility/visuals
    """
    MIN_AREA_OF_SPROKET = 3000
    MAX_AREA_OF_SPROKET = int(MIN_AREA_OF_SPROKET * 1.30)

    preview_image, image_height, image_width = GetPreviewImage(large_image)

    # Crop to target ROI along x-axis to isolate sprockets
    x1 = int(centre_box[0])
    x2 = int(centre_box[0] + centre_box[2])
    frame = preview_image[0:image_height, x1:x2]

    # Blur and threshold to emphasize sprocket hole
    matrix = (5, 9)
    frame_blur = cv.GaussianBlur(frame, matrix, 0)
    imgGry = cv.cvtColor(frame_blur, cv.COLOR_BGR2GRAY)
    _, threshold = cv.threshold(imgGry, lower_threshold, 255, cv.THRESH_BINARY)

    # Paste threshold view for visualization (into green channel stripe on left)
    preview_image[0:image_height, 0:centre_box[2], 1] = threshold

    contours, _ = cv.findContours(threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if draw_rects:
        cv.rectangle(
            preview_image,
            (centre_box[0], centre_box[1]),
            (centre_box[0] + centre_box[2], centre_box[1] + centre_box[3]),
            (128, 0, 128),
            2,
        )

    contours = sorted(contours, key=lambda x: cv.contourArea(x), reverse=True)

    if len(contours) > 0:
        contour = contours[0]
        area = cv.contourArea(contour)
        if MIN_AREA_OF_SPROKET < area < MAX_AREA_OF_SPROKET:
            rect = cv.minAreaRect(contour)
            rotation = rect[2]
            centre = rect[0]
            centre = (centre[0] + centre_box[0], centre[1])
            box = cv.boxPoints(rect).astype(int)
            if draw_rects:
                cv.circle(preview_image, (int(centre[0]), int(centre[1])), 12, (0, 150, 150), -1)
            return preview_image, centre, box
    else:
        cv.putText(preview_image, "No contour", (0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)

    return preview_image, None, None
