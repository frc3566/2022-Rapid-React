import multiprocessing as mp
import cv2
import numpy as np
import time
import logging
import math
import json
import sys

from networktables import NetworkTables, NetworkTablesInstance
from Constants import Constants
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

configFile = "/boot/frc.json"


class CameraConfig:
    pass

server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []


def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)


def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True


def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True


def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True


def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera


def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
                server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        NetworkTablesInstance.NotifyFlags.IMMEDIATE |
        NetworkTablesInstance.NotifyFlags.NEW |
        NetworkTablesInstance.NotifyFlags.UPDATE)

    return server


def start_camera_server():

    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    readConfig()

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))


class ShooterCameraProcess(mp.Process):

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.nt = NetworkTables.getTable('LiveWindow/ShooterCamera')
        self.goal_detect = False
        start_camera_server()

    def process_method(self):

        width = 640
        height = 480

        # CameraServer.getInstance().startAutomaticCapture()

        input_stream = CameraServer.getInstance().getVideo()

        output_stream = CameraServer.getInstance().putVideo('Processed', Constants.WIDTH, Constants.HEIGHT)
        binary_stream = CameraServer.getInstance().putVideo('Binary', Constants.WIDTH, Constants.HEIGHT)

        logging.basicConfig(level=logging.DEBUG)

        # Allocating new images is very expensive, always try to preallocate
        img = np.zeros(shape=(640, 480, 3), dtype=np.uint8)

        # Wait for NetworkTables to start
        time.sleep(0.5)

        # preallocate, get shape
        frame_time, input_img = input_stream.grabFrame(img)
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

        width = 640
        height = 480

        x_mid = width // 2
        y_mid = height // 2

        goal_detected = False

        # loop forever
        while True:
            if not NetworkTables.isConnected:
                print("restarting network table in shooter process")
                NetworkTables.startClientTeam(3566)
                NetworkTables.startDSClient()

            start_time = time.time()

            hsv_min = (57, 100, 24)
            hsv_max = (84, 255, 255)

            frame_time, input_img = input_stream.grabFrame(img)

            # Notify output of error and skip iteration
            if input_img.size == 0:
                self.logger.error("no frame")
                continue

            # print(input_img)

            # input_img = np.array(input_img)
            #
            # input_img = cv2.undistort(src=input_img, cameraMatrix=Constants.CAMERA_MATRIX,
            #                           distCoeffs=Constants.DIST_COEF)

            # input_stream = cv2.flip(input_stream, 0)

            output_img = np.copy(input_img)

            # Convert to HSV and threshold image
            hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
            binary_img = cv2.inRange(hsv_img, hsv_min, hsv_max)

            _, contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

            x_list = []
            y_list = []

            for contour in contour_list:
                # Ignore small contours that could be because of noise/bad thresholding
                area = cv2.contourArea(contour)
                if area < 15:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                if area / w / h < Constants.MIN_TARGET2RECT_RATIO:
                   continue

                cv2.drawContours(output_img, contour, -1, color=(255, 255, 255), thickness=-1)

                rect = cv2.minAreaRect(contour)
                center, size, angle = rect
                center = [int(dim) for dim in center]  # Convert to int so we can draw

                x_list.append(center[0])
                y_list.append(center[1])

            x = x_mid
            y = y_mid

            if(len(x_list)==0 or len(y_list)==0):
                goal_detected = False
                continue
            else:
                goal_detected = True

            x_angle = math.atan((center[0] - x_mid) / Constants.FOCAL_LENGTH_X)
            y_angle = math.atan((center[1] - y_mid) / Constants.FOCAL_LENGTH_Y) + Constants.CAMERA_MOUNT_ANGLE

            distance = Constants.CAMERA_GOAL_DELTA_H / math.tan(y_angle)

            processing_time = time.time() - start_time
            fps = 1 / processing_time
            cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))

            # update nt
            self.nt.putNumber("last_update_time", time.time())

            self.nt.putNumber("processing_time", processing_time)
            self.nt.putNumber("fps", fps)

            self.nt.putBoolean("goal_detected", goal_detected)

            self.nt.putNumber("x_angle", x_angle)
            self.nt.putNumber("y_angle", y_angle)
            self.nt.putNumber("distance", distance)

            output_stream.putFrame(output_img)
            binary_stream.putFrame(binary_img)

            NetworkTables.flush()

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")

