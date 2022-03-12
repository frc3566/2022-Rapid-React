import atexit
import logging
import logging.config
import time
import numbers
import sys
import json
import multiprocessing as mp
import numpy as np
from signal import *
from networktables import NetworkTables, NetworkTablesInstance
from IntakeCameraProcess import IntakeCameraProcess
from ShooterCameraProcess import ShooterCameraProcess
from Constants import Constants
from ProcessManager import ProcessManager
from queue import Full, Empty
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

print(sys.version)

# ntinst = NetworkTablesInstance
print("Setting up NetworkTables")
if not NetworkTables.isConnected():
    print("connecting network table")
    NetworkTables.startClientTeam(3566)
    # NetworkTables.initialize(server='10.35.66.2')
    time.sleep(1)

if NetworkTables.isConnected():
    print("Network table connected")

logging.getLogger().setLevel(logging.DEBUG)

# sharing data with mp queue
shooter_frame_in_queue = mp.Queue(10)
intake_frame_in_queue = mp.Queue(10)

shooter_frame_out_queue = mp.Queue(1)
intake_frame_out_queue = mp.Queue(1)

shooter_nt_queue = mp.Queue(100)
intake_nt_queue = mp.Queue(100)

# create process managers

intakeCameraTable = NetworkTables.getTable("LiveWindow/IntakeCamera")
intakeCamera_last_update_time = intakeCameraTable.getNumber("last_update_time", 0.0)

time.sleep(0.5)

def intakeCamera_is_updated():
    global intakeCamera_last_update_time
    global intakeCameraTable

    current_update_time = intakeCameraTable.getNumber("last_update_time", 0.0)

    if intakeCamera_last_update_time == current_update_time:
        return False
    else:
        intakeCamera_last_update_time = current_update_time
        return True


shooterCameraTable = NetworkTables.getTable("LiveWindow/ShooterCamera");
shooterCamera_last_update_time = shooterCameraTable.getNumber("last_update_time", 0.0)


def shooterCamera_is_updated():
    global shooterCamera_last_update_time
    global shooterCameraTable

    current_update_time = shooterCameraTable.getNumber("last_update_time", 0.0)

    if shooterCamera_last_update_time == current_update_time:
        return False
    else:
        shooterCamera_last_update_time = current_update_time
        return True


if __name__ == '__main__':
    CameraServer.getInstance().startAutomaticCapture(name="shooter",
        path="/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0"
        )

    intake_out_stream = CameraServer.getInstance().putVideo('intake', Constants.WIDTH, Constants.HEIGHT)

    shooter_in_stream = CameraServer.getInstance().getVideo()
    shooter_out_stream = CameraServer.getInstance().putVideo('shooter_processed', Constants.WIDTH, Constants.HEIGHT)

    intakeCameraProcesManager = ProcessManager(lambda end_queue: IntakeCameraProcess(intake_nt_queue,
                                               intake_frame_in_queue, intake_frame_out_queue, end_queue),
                                               name="intake_camera_proces")

    shooterCameraProcessManager = ProcessManager(lambda end_queue: ShooterCameraProcess(shooter_nt_queue,
                                                 shooter_frame_in_queue, shooter_frame_out_queue, end_queue),
                                                 name="shooter_camera_process")

    # CameraServer.getInstance().startAutomaticCapture(name="climber",
    #     path="/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index1"
    #                                                         )

    def cleanup(*args):
        intakeCameraProcesManager.end_process()

        shooterCameraProcessManager.end_process()

        NetworkTables.stopClient()

        NetworkTables.shutdown()

        logging.info("cleaned up processes and network table")

        sys.exit()


    for sig in (SIGABRT, SIGILL, SIGINT, SIGSEGV, SIGTERM):
        signal(sig, cleanup)

    atexit.register(cleanup)

    FMSTable = NetworkTables.getTable("FMSInfo")

    def update_alliance_color(table, key, value, isNew):
        Constants.isRed = FMSTable.getBoolean("IsRedAlliance", True)
        if(Constants.isRed):
            print("Switched to alliance: ", "RED", sep=" ")
        else:
            print("Switched to alliance: ", "BLUE", sep=" ")

    FMSTable.addEntryListener(update_alliance_color)

    Constants.isRed = FMSTable.getBoolean("IsRedAlliance", True)
    if (Constants.isRed):
        print("Switched to alliance: ", "RED", sep=" ")
    else:
        print("Switched to alliance: ", "BLUE", sep=" ")

    # main loop
    while True:

        # print(NetworkTables.isConnected())

        if not NetworkTables.isConnected():
            print("connection lost, restarting network table")
            NetworkTables.startClientTeam(3566)
            # NetworkTables.initialize(server='10.35.66.2')
            time.sleep(1)

        img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

        frame_time, img = shooter_in_stream.grabFrame(img)
        if not img.size == 0:
            try:
                shooter_frame_in_queue.put_nowait((frame_time, img))
            except Full:
                # print("shooter frame in full")
                pass

        # print(img)
        # print(shooter_frame_in_queue.qsize())

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

        # NetworkTables.flush()

        # update camera server

        frame = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

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
            pass

        # get CV
        intakeCameraProcesManager.checkin(intakeCamera_is_updated())

        shooterCameraProcessManager.checkin(shooterCamera_is_updated())

