from __future__ import annotations

from typing import Optional, Tuple
import time
import logging

import cv2 as cv
from picamera2 import Picamera2


logger = logging.getLogger(__name__)


class CameraController:
    """Encapsulates Picamera2 configuration and capture helpers.

    Responsibilities:
    - Configure preview/high-res pipelines
    - Start/stop camera lifecycle
    - Stable exposure and AWB helpers (auto once, then lock)
    - Provide frames in BGR for OpenCV
    """

    def __init__(self) -> None:
        self.camera: Optional[Picamera2] = None
        self.shutter_speed: int = 1000
        self.iso: int = 100
        self.locked_awb_gains: Optional[Tuple[float, float]] = None
        self.swap_rb: bool = False

    # ---------- Lifecycle ----------
    def configure_high_res_camera(self) -> Tuple[int, int]:
        """Configure the camera for high-resolution video capture and return PixelArraySize (w, h).
        Reuses an existing Picamera2 instance when possible to avoid device-busy errors.
        """
        if self.camera is None:
            logger.info("Configuring high res camera settings (new instance)")
            self.camera = Picamera2()
        else:
            logger.info("Reconfiguring camera to high res settings")
            if self.camera.started:
                self.camera.stop()
        config = self.camera.create_video_configuration(
            main={"size": (2028, 1520), "format": "RGB888"}
        )
        self.camera.configure(config)
        # Disable automatic algorithms by default; we'll run them manually once
        self.camera.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
        })
        return self._pixel_array_size()

    def configure_low_res_camera(self) -> Tuple[int, int]:
        """Configure the camera for low-resolution preview and return PixelArraySize (w, h).
        Reuses an existing Picamera2 instance when possible to avoid device-busy errors.
        """
        if self.camera is None:
            self.camera = Picamera2()
        else:
            if self.camera.started:
                self.camera.stop()
        config = self.camera.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
        self.camera.configure(config)
        # Keep AE/AWB disabled by default; caller can perform auto once then lock
        self.camera.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
        })
        return self._pixel_array_size()

    def start(self) -> None:
        assert self.camera is not None, "Camera not configured"
        if not self.camera.started:
            self.camera.start()

    def stop_and_close(self) -> None:
        if self.camera is not None:
            if self.camera.started:
                self.camera.stop()
            self.camera.close()
            self.camera = None

    def _pixel_array_size(self) -> Tuple[int, int]:
        assert self.camera is not None
        pas = self.camera.camera_properties["PixelArraySize"]
        # Picamera2 returns (width, height)
        return int(pas[0]), int(pas[1])

    # ---------- Capture ----------
    def capture_bgr(self, stream: str = "main"):
        """Capture a frame and return an image suitable for OpenCV (BGR).
        Assumes RGB888 stream; converts RGB->BGR. Handles RGBA->BGR if needed.
        """
        assert self.camera is not None, "Camera not configured"
        img = self.camera.capture_array(stream)
        if len(img.shape) == 3 and img.shape[2] == 4:
            # Convert RGBA to BGR
            try:
                bgr = cv.cvtColor(img, cv.COLOR_RGBA2BGR)
            except Exception:
                # Fallback channel reorder if cvtColor not available
                bgr = img[:, :, 2::-1]
            return bgr[:, :, ::-1] if self.swap_rb else bgr
        if len(img.shape) == 3 and img.shape[2] == 3:
            # Convert RGB888 to BGR for OpenCV
            bgr = cv.cvtColor(img, cv.COLOR_RGB2BGR)
            return bgr[:, :, ::-1] if self.swap_rb else bgr
        return img

    # ---------- Controls ----------
    def auto_shutter_speed(self) -> Tuple[int, float]:
        """Temporarily enable AE to get sane ExposureTime and AnalogueGain, then disable AE again."""
        assert self.camera is not None, "Camera not configured"
        self.camera.set_controls({"AeEnable": True})
        time.sleep(2)
        metadata = self.camera.capture_metadata()
        shutter_speed = int(metadata["ExposureTime"])  # us
        analogue_gain = float(metadata["AnalogueGain"])  # ~ISO/100
        self.camera.set_controls({"AeEnable": False})
        logger.info("Auto exposure settled: shutter=%s us, analogue_gain=%.2f", shutter_speed, analogue_gain)
        return shutter_speed, analogue_gain

    def set_exposure(self, shutter_speed: int, iso: int) -> None:
        """Disable AE and set ExposureTime + AnalogueGain derived from ISO.
        Note: ISO is approximated as analogue_gain = ISO/100.
        """
        assert self.camera is not None, "Camera not configured"
        self.shutter_speed = int(shutter_speed)
        self.iso = int(iso)
        analogue_gain = self.iso / 100.0
        self.camera.set_controls({
            "AeEnable": False,
            "ExposureTime": self.shutter_speed,
            "AnalogueGain": analogue_gain,
        })
        time.sleep(0.5)
        if self.camera.started:
            md = self.camera.capture_metadata()
            logger.info("Exposure set: iso=%s, shutter=%s us (md ExposureTime=%s)", self.iso, self.shutter_speed, md.get("ExposureTime"))

    def auto_white_balance(self, newgain: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        """Run AWB briefly, lock gains, and return them. If newgain provided, apply and lock it."""
        assert self.camera is not None, "Camera not configured"
        if newgain is None:
            self.camera.set_controls({"AwbEnable": True})
            time.sleep(2)
            md = self.camera.capture_metadata()
            gains = tuple(md["ColourGains"])  # type: ignore
            self.camera.set_controls({"AwbEnable": False, "ColourGains": gains})
        else:
            gains = (float(newgain[0]), float(newgain[1]))
            self.camera.set_controls({"AwbEnable": False, "ColourGains": gains})
        self.locked_awb_gains = (float(gains[0]), float(gains[1]))
        logger.info("AWB locked with gains: %s", self.locked_awb_gains)
        return self.locked_awb_gains

    # ---------- Misc ----------
    def toggle_swap_rb(self) -> None:
        self.swap_rb = not self.swap_rb
