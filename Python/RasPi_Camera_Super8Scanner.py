# Super8Scanner.py
#
# (c)2021 Stuart Pittaway
# (c)2025 Rewritten for picamera2 by Tim Hayes
#
# The purpose of this program is to digitize Super8 film reel using a Raspberry Pi camera.
# It uses OpenCV to detect the alignment of the images using the film reel sprockets as alignment targets.
# It outputs a PNG image per frame, which are vertically aligned, but frame borders and horizontal alignment
# are not cropped, removed or fixed. This is the job of a second script to complete this work.
#
# Camera images are captured and saved as PNG to avoid any compression artifacts during
# the capture and alignment processes.
#
# Expects to control a MARLIN style stepper driver board
# Y axis is used to drive film feed rollers
# Z axis is used to drive film reel take up spool
# FAN output is used to drive LED light for back light of frames

from libcamera import controls
import queue
from threading import Thread
import numpy as np
import cv2 as cv
import glob
from pathlib import Path
import math
from datetime import datetime
import time
import logging
from camera_control import CameraController
from marlin_control import Marlin
from image_processing import GetPreviewImage, ProcessImage, pointInRect

# Configure logging early; keep INFO level to mirror prior print usage
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

class ScannerConfig:
    FRAME_WIDTH_MM = 5.79
    FRAME_HEIGHT_MM = 4.01
    FILM_THICKNESS_MM = 0.150
    INNER_DIAMETER_OF_TAKE_UP_SPOOL_MM = 32.0 # 46.0 for 400ft reels
    FRAMES_TO_WAIT_UNTIL_SPOOLING = 8
    CAMERA_EXPOSURE = [-8.0]
    NUDGE_FEED_RATE = 1000
    STANDARD_FEED_RATE = 12000
    FRAME_SPACING = 16.0 #16.42

class ScannerState:
    def __init__(self):
        self.camera_ctl: CameraController | None = None
        self.shutter_speed = 1000
        self.iso = 100
        self.locked_awb_gains = None
        self.swap_rb = True
        self.lower_threshold = 150
        self.new_lower_threshold_value = 0
        self.new_shutter_speed_value = 0

    def on_threshold_change(self, val):
        self.new_lower_threshold_value = val

    def on_shutter_speed_change(self, val):
        self.new_shutter_speed_value = val

def capture_bgr(state: ScannerState):
    assert state.camera_ctl is not None
    return state.camera_ctl.capture_bgr("main")

NUM_THREADS = 3

q = queue.Queue(maxsize=10)

"""pointInRect moved to image_processing.py"""


"""Marlin serial helpers migrated to marlin_control.Marlin"""


"""GetPreviewImage moved to image_processing.py"""


"""ProcessImage moved to image_processing.py"""


def MoveFilm(marlin: Marlin, y: float, feed_rate: int):
    marlin.move_film(y, feed_rate)
    time.sleep(0.25) # wait for camera to catch up


def MoveReel(marlin: Marlin, z: float, feed_rate: int, wait_for_completion=True):
    marlin.move_reel(z, feed_rate, wait_for_completion)


def SetMarlinLight(marlin: Marlin, level: int = 255):
    marlin.set_light(level)


def ConnectToMarlin() -> Marlin:
    m = Marlin()
    m.connect()
    return m


def DisconnectFromMarlin(marlin: Marlin):
    marlin.disconnect()


def OutputFolder(exposures: list) -> str:
    """Ensure output directories exist and return base 'Capture' folder as string.

    This preserves existing behavior (paths like 'Capture-8.0').
    """
    cwd = Path.cwd()
    # cwd = Path("/mnt/super8/Super8MovieFrameStorage")  # Override for my external drive
    # Create folders for the different EV exposure levels
    for e in exposures:
        (cwd / f"Capture{e}").mkdir(parents=True, exist_ok=True)

    # Image Output path - create if needed
    base = cwd / "Capture"
    base.mkdir(parents=True, exist_ok=True)
    return str(base)


def StartupAlignment(marlin: Marlin, centre_box, state: ScannerState):
    WINDOW_NAME = 'Startup Alignment'

    marlin_y = 0
    reel_z = 0

    return_value = False

    # Configure low-res preview using the shared controller
    if state.camera_ctl is None:
        state.camera_ctl = CameraController()
    # Always sync swap flag from global for consistent default
    state.camera_ctl.swap_rb = state.swap_rb
    state.camera_ctl.configure_low_res_camera()
    
    # Convert ISO to analogue gain for initial setup
    analogue_gain = state.iso / 100.0
    
    # Disable AE initially; set analogue gain from iso approx
    assert state.camera_ctl.camera is not None
    state.camera_ctl.camera.set_controls({"AeEnable": False, "AnalogueGain": analogue_gain})

    # Start with the existing preview configuration from configureLowResCamera()
    state.camera_ctl.start()
    time.sleep(1.5)

    # Let AE pick sane values once, then lock exposure for alignment stability
    state.shutter_speed, analogue_gain = state.camera_ctl.auto_shutter_speed()
    state.iso = int(analogue_gain * 100)
    state.camera_ctl.set_exposure(state.shutter_speed, state.iso)
    # awb_gain = AutoWB(camera)

    threshold_enable = False

    state.new_shutter_speed_value = state.shutter_speed
    state.new_lower_threshold_value = state.lower_threshold

    cv.namedWindow(WINDOW_NAME)
    trackbar_name = 'Threshold value'
    cv.createTrackbar(trackbar_name, WINDOW_NAME, state.lower_threshold, 254, state.on_threshold_change)
    cv.setTrackbarMin(trackbar_name, WINDOW_NAME, 50)

    trackbar_name2 = 'Camera shutter speed'
    cv.createTrackbar(trackbar_name2, WINDOW_NAME, state.shutter_speed, 100000, state.on_shutter_speed_change)
    cv.setTrackbarMin(trackbar_name2, WINDOW_NAME, 50)

    while True:
        image = capture_bgr(state)

        # Mirror vertical - sprocket is now on left of image
        image = cv.flip(image, 0)

        preview_image, centre, _ = ProcessImage(image, centre_box, True, lower_threshold=state.lower_threshold)

        if threshold_enable:
            _, preview_image = cv.threshold(cv.cvtColor(preview_image, cv.COLOR_BGR2GRAY), state.lower_threshold, 255,
                                            cv.THRESH_BINARY)

        if centre is None:
            cv.putText(preview_image, "Sproket hole not detected",
                       (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, cv.LINE_AA)
        else:
            cv.putText(preview_image, "Sproket hole detected, press SPACE to start scanning",
                       (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 1, cv.LINE_AA)

        cv.putText(preview_image, "press UP/DOWN to nudge reel, SPACE to cont.", (8, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7,
                   (255, 0, 255), 1, cv.LINE_AA)
        cv.putText(preview_image, "j to jump forward, t toggle threshold style.", (8, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7,
                   (255, 0, 255), 1, cv.LINE_AA)
        cv.putText(preview_image, "Threshold, value={0}".format(state.lower_threshold), (8, 115), cv.FONT_HERSHEY_SIMPLEX,
                   0.7, (255, 0, 255), 1, cv.LINE_AA)
        cv.putText(preview_image, "shutter_speed, value={0}, iso={1}".format(state.shutter_speed, state.iso), (8, 300),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 1, cv.LINE_AA)
        cv.putText(preview_image, "r to rewind spool (1 revolution), ESC to quit", (8, 330), cv.FONT_HERSHEY_SIMPLEX,
                   0.7, (255, 0, 255), 1, cv.LINE_AA)
        cv.putText(preview_image, f"Swap R<->B: {'ON' if state.swap_rb else 'OFF'} (press 'c')", (8, 360), cv.FONT_HERSHEY_SIMPLEX, 0.6,
                   (255, 255, 255), 1, cv.LINE_AA)

        cv.imshow(WINDOW_NAME, preview_image)

        if state.new_lower_threshold_value != state.lower_threshold:
            state.lower_threshold = state.new_lower_threshold_value

        if state.shutter_speed != state.new_shutter_speed_value:
            state.shutter_speed = state.new_shutter_speed_value
            state.camera_ctl.set_exposure(state.shutter_speed, state.iso)

        k = cv.waitKeyEx(30)

        if k == ord(' '):  # SPACE key to continue
            return_value = True
            break

        if k == ord('t'):
            threshold_enable = not threshold_enable
        if k == ord('c'):
            state.swap_rb = not state.swap_rb
            if state.camera_ctl is not None:
                state.camera_ctl.toggle_swap_rb()
            print('Channel swap (R<->B):', state.swap_rb)

        if k == ord('s'):
            state.shutter_speed, analogue_gain = state.camera_ctl.auto_shutter_speed()
            state.iso = int(analogue_gain * 100)  # update iso based on new gain

        if k == ord('a'):
            # Re-evaluate AWB and update locked gains
            state.locked_awb_gains = state.camera_ctl.auto_white_balance()

        if k == 27:
            return_value = False
            break

        if k == 65362:
            marlin_y += 1
            MoveFilm(marlin, marlin_y, 1000)

        if k == ord('j'):
            marlin_y += 100
            MoveFilm(marlin, marlin_y, 8000)

        if k == 65364:
            marlin_y -= 1
            MoveFilm(marlin, marlin_y, 1000)

        if k == ord('r'):
            reel_z -= 360
            MoveReel(marlin, reel_z, 20000, False)

    state.camera_ctl.stop_and_close()
    state.camera_ctl = None
    cv.destroyWindow(WINDOW_NAME)
    return return_value


def determineStartingFrameNumber(path: str, ext: str) -> int:
    existing_files = sorted(glob.glob(str(Path(path) / f"frame_????????.{ext}")), reverse=True)

    if len(existing_files) > 0:
        latest = Path(existing_files[0])
        return 1 + int(latest.stem.split('_')[1])

    return 0


def calculateAngleForSpoolTakeUp(inner_diameter_spool: float, frame_height: float, film_thickness: float,
                                 frames_on_spool: int, new_frames_to_spool: int) -> float:
    '''Calculate the angle to wind the take up spool forward based on
    known number of frames already on the spool and the amount of frames we want to add.
     May return more than 1 full revolution of the wheel (for example 650 degrees)'''
    r = inner_diameter_spool / 2
    existing_tape_length = frame_height * frames_on_spool
    spool_radius = math.sqrt(existing_tape_length *
                             film_thickness / math.pi + r ** 2)
    circumference = 2 * math.pi * spool_radius
    arc_length = new_frames_to_spool * frame_height
    angle = arc_length / circumference * 360
    # logger.debug("spool_radius %s circumference %s degrees %s arc_length %s", spool_radius, circumference, angle, arc_length)
    return angle


def configureHighResCamera(state: ScannerState) -> tuple[int, int]:
    if state.camera_ctl is None:
        state.camera_ctl = CameraController()
    # Always sync swap flag from global before configuring
    state.camera_ctl.swap_rb = state.swap_rb
    return state.camera_ctl.configure_high_res_camera()


def configureLowResCamera(state: ScannerState) -> tuple[int, int]:
    if state.camera_ctl is None:
        state.camera_ctl = CameraController()
    # Always sync swap flag from global before configuring
    state.camera_ctl.swap_rb = state.swap_rb
    return state.camera_ctl.configure_low_res_camera()


def ServiceImageWriteQueue(q):
    path = OutputFolder([])

    while True:
        data = q.get(block=True, timeout=None)

        target_dir = Path(path + f"{data['exposure']}")
        target_dir.mkdir(parents=True, exist_ok=True)
        filename = target_dir / f"frame_{data['number']:08d}.png"
        
        if not cv.imwrite(str(filename), data["image"], [cv.IMWRITE_PNG_COMPRESSION, 2]):
            raise IOError("Failed to save image")
        
        q.task_done()


def main():
    logger.info("OpenCV Version %s", cv.__version__)

    state = ScannerState()
    config = ScannerConfig()

    path = OutputFolder(config.CAMERA_EXPOSURE)
    logger.info("Output folder is %s", path)
    starting_frame_number = determineStartingFrameNumber(path + "-8.0", "png")
    logger.info("Starting at frame number %s", starting_frame_number)

    if state.camera_ctl is None:
        state.camera_ctl = CameraController()
    # Always sync swap flag from global for consistent color across phases
    state.camera_ctl.swap_rb = state.swap_rb
    highres_width, highres_height = configureHighResCamera(state)

    preview_image, image_height, image_width = GetPreviewImage(
        np.zeros((highres_height, highres_width, 3), np.uint8))

    logger.info("Camera configured for resolution %s x %s. Preview image %s x %s",
                highres_width, highres_height, image_width, image_height)

    centre_box = [50, 0, 40, 64]
    centre_box[1] = int(image_height / 2 - centre_box[3] / 2)

    marlin = ConnectToMarlin()

    try:
        if StartupAlignment(marlin, centre_box, state):
            time_start = datetime.now()
            frame_number = starting_frame_number
            frames_already_on_spool = frame_number
            frames_to_add_to_spool = 0
            marlin_y = 0.0
            last_y_list = []
            reel_z = 0

            marlin.send("G92 X0 Y0 Z0")
            marlin.send("M18 X Z")

            manual_control = False
            micro_adjustment_steps = 0

        for i in range(NUM_THREADS):
            worker = Thread(target=ServiceImageWriteQueue, args=(q,))
            worker.daemon = True
            worker.start()
            
        highres_width, highres_height = configureHighResCamera(state)
        assert state.camera_ctl is not None
        state.camera_ctl.start()
        time.sleep(1.5)
        # Let AE pick sane exposure on the final config, then lock it
        state.shutter_speed, analogue_gain = state.camera_ctl.auto_shutter_speed()
        state.iso = int(analogue_gain * 100)
        state.camera_ctl.set_exposure(state.shutter_speed, state.iso)
        # Always run AWB once on the final capture configuration, then lock and reuse gains
        state.locked_awb_gains = state.camera_ctl.auto_white_balance()
        while True:
                time.sleep(0.1)
                freeze_frame = capture_bgr(state)
                freeze_frame = cv.flip(freeze_frame, 0)
                logger.debug('frame captured: %s', manual_control)
                manual_grab = False

                if frames_to_add_to_spool > config.FRAMES_TO_WAIT_UNTIL_SPOOLING + 3:
                    angle = calculateAngleForSpoolTakeUp(
                        config.INNER_DIAMETER_OF_TAKE_UP_SPOOL_MM, config.FRAME_HEIGHT_MM,
                        config.FILM_THICKNESS_MM, frames_already_on_spool, config.FRAMES_TO_WAIT_UNTIL_SPOOLING)
                    reel_z -= angle
                    MoveReel(marlin, reel_z, 8000, False)
                    # MoveReel(marlin, reel_z, 8000)
                    frames_already_on_spool += config.FRAMES_TO_WAIT_UNTIL_SPOOLING
                    frames_to_add_to_spool -= config.FRAMES_TO_WAIT_UNTIL_SPOOLING

                if micro_adjustment_steps > 25:
                    logger.warning("Emergency manual mode as too many small adjustments made")
                    manual_control = True

                if manual_control:
                    print("Waiting for command key press")
                    k = cv.waitKey(10000) & 0xFF
                else:
                    k = cv.waitKey(10) & 0xFF

                if k == 27:
                    break

                if k == ord('c'):
                    state.swap_rb = not state.swap_rb
                    if state.camera_ctl is not None:
                        state.camera_ctl.toggle_swap_rb()
                    print('Channel swap (R<->B):', state.swap_rb)

                if k == ord('m') and not manual_control:
                    manual_control = True

                if manual_control:
                    if k == 32:
                        logger.info("Manual control ended")
                        manual_control = False
                        starting_frame_number = frame_number
                        time_start = datetime.now()
                        micro_adjustment_steps = 0
                        continue
                    if k == ord(','):
                        state.shutter_speed -= 50
                        if state.shutter_speed < 0:
                            state.shutter_speed = 0
                        state.camera_ctl.set_exposure(state.shutter_speed, state.iso)
                    if k == ord('.'):
                        state.shutter_speed += 50
                        if state.shutter_speed > 180000:
                            state.shutter_speed = 180000
                        state.camera_ctl.set_exposure(state.shutter_speed, state.iso)
                    if k == ord('a'):
                        state.locked_awb_gains = state.camera_ctl.auto_white_balance()
                        assert state.camera_ctl.camera is not None
                        state.camera_ctl.camera.set_controls({"AwbEnable": False, "ColourGains": state.locked_awb_gains})
                    if k == ord('f'):
                        marlin_y += 1
                        MoveFilm(marlin, marlin_y, 500)
                    if k == ord('b'):
                        marlin_y -= 1
                        MoveFilm(marlin, marlin_y, 500)
                    if k == ord('['):
                        state.lower_threshold -= 1
                    if k == ord(']'):
                        state.lower_threshold += 1
                    if k == ord('{'):
                        state.lower_threshold -= 10
                    if k == ord('}'):
                        state.lower_threshold += 10
                    if k == ord('g'):
                        manual_grab = True

                last_exposure = config.CAMERA_EXPOSURE[0]
                preview_image, centre, _ = ProcessImage(
                    freeze_frame, centre_box, True, last_exposure, lower_threshold=state.lower_threshold)

                if frame_number > 0:
                    fps = (frame_number - starting_frame_number) / \
                          (datetime.now() - time_start).total_seconds()
                    cv.putText(preview_image, "Frames {0}, Capture FPS {1:.2f}, fp/h {2:.1f}".format(
                        frame_number - starting_frame_number, fps, fps * 3600), (8, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                               (255, 0, 255), 1, cv.LINE_AA)
                    cv.putText(preview_image, "Threshold {0}".format(
                        state.lower_threshold), (8, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv.LINE_AA)
                cv.putText(preview_image, f"Swap R<->B: {'ON' if state.swap_rb else 'OFF'} (press 'c')", (8, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (255, 255, 255), 1, cv.LINE_AA)

                if manual_control:
                    cv.putText(preview_image, "Manual Control Active, keys f/b to align",
                               (0, 300), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
                    cv.putText(preview_image, "g to grab, [ and ] alter threshold. SPACE to continue",
                               (0, 350), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

                if centre is None and not manual_grab:
                    cv.putText(preview_image, "SPROKET HOLE LOST", (16, 100),
                               cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv.LINE_AA)

                cv.imshow('RawVideo', preview_image)
                cv.waitKey(5)

                if centre is None and not manual_grab:
                    manual_control = True
                    continue

                if manual_control and not manual_grab:
                    continue

                if not pointInRect(centre, centre_box) and not manual_grab:
                    micro_adjustment_steps += 1
                    centre_y = int(centre_box[1] + centre_box[3] / 2)
                    diff_pixels = abs(int(centre_y - centre[1]))

                    if centre[1] > centre_y:
                        logger.debug("FORWARD! %s diff pixels= %s", marlin_y, diff_pixels)
                        marlin_y += 1.5
                    else:
                        logger.debug("REVERSE! %s diff pixels= %s", marlin_y, diff_pixels)
                        marlin_y -= 0.5

                    MoveFilm(marlin, marlin_y, config.NUDGE_FEED_RATE)
                    continue

                try:
                    if manual_grab:
                        logger.info("Manual Grab!")

                    for my_exposure in config.CAMERA_EXPOSURE:
                        highres_image_height, highres_image_width = freeze_frame.shape[:2]
                        thumbnail = cv.resize(freeze_frame, (0, 0), fx=0.50, fy=0.50)
                        cv.imshow("Exposure", thumbnail)

                        q.put({"number": frame_number, "exposure": my_exposure, "image": freeze_frame})
                        logger.debug("Image put onto queue, q length= %s", q.qsize())

                    frame_number += 1
                    frames_to_add_to_spool += 1

                    marlin_y += config.FRAME_SPACING
                    MoveFilm(marlin, marlin_y, config.STANDARD_FEED_RATE)
                    micro_adjustment_steps = 0

                except BaseException as err:
                    logger.exception("High Res Capture Loop Error %r, %r", err, type(err))

    finally:
        logger.info("Waiting for image write queue to empty... length= %s", q.qsize())
        q.join()
        logger.info("Destroy windows")
        cv.destroyAllWindows()
        logger.info("Disconnect Marlin")
        DisconnectFromMarlin(marlin)

        if state.camera_ctl is not None:
            state.camera_ctl.stop_and_close()


if __name__ == "__main__":
    main()
