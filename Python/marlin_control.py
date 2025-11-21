from __future__ import annotations

import logging
from datetime import datetime
import time
from typing import Iterable

import serial


logger = logging.getLogger(__name__)


class Marlin:
    """Thin wrapper around a Marlin-compatible serial connection and common commands."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, timeout: float = 5.0) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: serial.Serial | None = None

    # ---------- Lifecycle ----------
    def connect(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            timeout=self.timeout,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
        )
        # Drain initial chatter
        self._wait_for_reply(echo=False)
        # Initial setup
        self.send_multiple([
            "M502",
            "G21",
            "M211 S0",
            "G90",
            "G92 X0 Y0 Z0",
            "M201 Y0",
            "M18 S15",
            "M203 X1000.00 Y1000.00 Z5000.00",
        ])
        self.set_light(255)
        self.send("M92 Y10 Z8.888888")  # steps per unit
        self.send("M400")  # wait complete

    def disconnect(self) -> None:
        if self.ser is None:
            return
        self.set_light(0)
        self.send_multiple(["M84"])  # steppers off
        self.ser.close()
        self.ser = None

    # ---------- Commands ----------
    def send(self, cmd: str) -> None:
        assert self.ser is not None, "Serial not connected"
        logger.info("Sending GCODE %s", cmd)
        if not self.ser.isOpen():  # type: ignore[attr-defined]
            raise RuntimeError("Port closed")
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.read_all()
        self.ser.write(cmd.encode("utf-8"))
        self.ser.write(b"\n")
        if not self._wait_for_reply():
            raise RuntimeError("Bad GCODE command or no valid reply from Marlin")

    def send_multiple(self, cmds: Iterable[str]) -> None:
        for c in cmds:
            self.send(c)

    def set_light(self, level: int = 255) -> None:
        if level > 0:
            self.send(f"M106 S{level}")
        else:
            self.send("M107")

    def move_film(self, y: float, feed_rate: int) -> None:
        self.send(f"G0 Y{y:.4f} F{feed_rate}")
        self.send("M400")

    def move_reel(self, z: float, feed_rate: int, wait_for_completion: bool = True) -> None:
        self.send(f"G0 Z{z:.4f} F{feed_rate}")
        if wait_for_completion:
            self.send("M400")

    # ---------- Internal ----------
    def _wait_for_reply(self, echo: bool = True) -> bool:
        assert self.ser is not None, "Serial not connected"
        tstart = datetime.now()
        while True:
            if self.ser.in_waiting > 0:  # type: ignore[attr-defined]
                serial_bytes = self.ser.readline()
                if echo and serial_bytes.startswith(b"echo:"):
                    logger.info("Marlin R: %s", serial_bytes.decode("ascii", errors="ignore").strip())
                if serial_bytes == b"ok\n":
                    return True
                # reset timeout window when receiving anything
                tstart = datetime.now()
            else:
                time.sleep(0.01)
                duration = (datetime.now() - tstart).total_seconds()
                if duration > 3:
                    return False
