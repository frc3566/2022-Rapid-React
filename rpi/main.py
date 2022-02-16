import atexit
import logging
import logging.config
import time
from networktables import NetworkTables, NetworkTablesInstance
import sys

from LocalizationCameraProcess import LocalizationCameraProcess
from IntakeCameraProcess import IntakeCameraProcess
from ShooterCameraProcess import ShooterCameraProcess
from Constants import Constants
from ProcessManager import ProcessManager

print(sys.version)

ntinst = NetworkTablesInstance
print("Setting up NetworkTables")
NetworkTables.startClientTeam(3566)
NetworkTables.startDSClient()


logging.getLogger().setLevel(logging.DEBUG)

if Constants.noRoboRIO:
    NetworkTables.initialize()

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

    intakeCameraProcesManager = ProcessManager(lambda: IntakeCameraProcess(), name="intake_camera_proces")

    shooterCameraProcessManager = ProcessManager(lambda: ShooterCameraProcess(), name="shooter_camera_process")

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

        # get CV
        intakeCameraProcesManager.checkin(intakeCamera_is_updated())

        # shooterCameraProcessManager.checkin(shooterCamera_is_updated())

