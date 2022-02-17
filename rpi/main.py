import atexit
import logging
import logging.config
import time
import numbers
import sys
import json
import multiprocessing as mp
import numpy as np
from networktables import NetworkTables, NetworkTablesInstance
from IntakeCameraProcess import IntakeCameraProcess
from ShooterCameraProcess import ShooterCameraProcess
from Constants import Constants
from ProcessManager import ProcessManager
from queue import Full, Empty
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

print(sys.version)

ntinst = NetworkTablesInstance
print("Setting up NetworkTables")
NetworkTables.startClientTeam(3566)
NetworkTables.startDSClient()


logging.getLogger().setLevel(logging.DEBUG)


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

# sharing data with mp queue
shooter_frame_in_queue = mp.Queue(10)
intake_frame_in_queue = mp.Queue(10)

shooter_frame_out_queue = mp.Queue(1)
intake_frame_out_queue = mp.Queue(1)

shooter_nt_queue = mp.Queue(100)
intake_nt_queue = mp.Queue(100)

# create process managers

intakeCameraTable = NetworkTables.getTable("LiveWindow/IntakeCamera");
intakeCamera_last_update_time = intakeCameraTable.getNumber("last_update_time", 0.0)

def intakeCamera_is_updated():
    global intakeCamera_last_update_time
    global intakeCameraTable

    current_update_time = intakeCameraTable.getNumber("last_update_time", 0.0)

    if intakeCamera_last_update_time == current_update_time:
        return False
    elif intakeCamera_last_update_time < current_update_time:
        intakeCamera_last_update_time = current_update_time
        return True
    else:
        logging.error("Intake Camera Time Error")
        return False


shooterCameraTable = NetworkTables.getTable("LiveWindow/ShooterCamera");
shooterCamera_last_update_time = shooterCameraTable.getNumber("last_update_time", 0.0)


def shooterCamera_is_updated():
    global shooterCamera_last_update_time
    global shooterCameraTable

    current_update_time = shooterCameraTable.getNumber("last_update_time", 0.0)

    if shooterCamera_last_update_time == current_update_time:
        return False
    elif shooterCamera_last_update_time < current_update_time:
        shooterCamera_last_update_time = current_update_time
        return True
    else:
        logging.error("Intake Camera Time Error")
        return False

# processes

if __name__ == '__main__':

    start_camera_server()

    intake_out_stream = CameraServer.getInstance().putVideo('shooter', Constants.WIDTH, Constants.HEIGHT)

    shooter_in_stream = CameraServer.getInstance().getVideo()
    shooter_out_stream = CameraServer.getInstance().putVideo('intake', Constants.WIDTH, Constants.HEIGHT)

    intakeCameraProcesManager = ProcessManager(lambda: IntakeCameraProcess(intake_nt_queue,
                                               intake_frame_in_queue, intake_frame_out_queue),
                                               name="intake_camera_proces")

    shooterCameraProcessManager = ProcessManager(lambda: ShooterCameraProcess(shooter_nt_queue,
                                                 shooter_frame_in_queue, shooter_frame_out_queue),
                                                 name="shooter_camera_process")

    def cleanup():
        intakeCameraProcesManager.end_process()

        shooterCameraProcessManager.end_process()

        logging.info("cleaned up processes")

    atexit.register(cleanup)

    # main loop
    while True:

        if not NetworkTables.isConnected:
            print("connection lost, restarting network table")
            NetworkTables.startClientTeam(3566)
            NetworkTables.startDSClient()

        if not intake_frame_in_queue.empty():
            intake_frame_in_queue.get_nowait()

        img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

        frame_time, img = shooter_in_stream.grabFrame(img)
        intake_frame_in_queue.put_nowait((frame_time, img))

        # update nt
        try:
            while True:
                key, value = intake_nt_queue.get_nowait()
                if isinstance(value, numbers.Number):
                    intakeCameraTable.putNumber(key, value)
                elif type(value) == bool:
                    intakeCameraTable.putBoolean(key, value)

        except Empty:
            pass

        try:
            while True:
                key, value = shooter_nt_queue.get_nowait()
                if isinstance(value, numbers.Number):
                    shooterCameraTable.putNumber(key, value)
                elif type(value) == bool:
                    shooterCameraTable.putBoolean(key, value)

        except Empty:
            pass

        NetworkTables.flush()

        # update camera server

        try:
            while True:
                frame = intake_frame_out_queue.get_nowait()
                intake_out_stream.putFrame(frame)
        except Empty:
            pass

        try:
            while True:
                frame = shooter_frame_out_queue.get_nowait()
                shooter_out_stream.putFrame(frame)
        except Empty:
            shooter_out_stream.putFrame(img)

        # get CV
        intakeCameraProcesManager.checkin(intakeCamera_is_updated())

        shooterCameraProcessManager.checkin(shooterCamera_is_updated())

