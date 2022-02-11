import logging
import logging.config
import multiprocessing as mp
import time
from datetime import datetime
from networktables import NetworkTables
import yaml
import numpy as np
import numbers
import math
import sys

from LocalizationCameraProcess import LocalizationCameraProcess
from IntakeCameraProcess import IntakeCameraProcess
from ShooterCameraProcess import ShooterCameraProcess
from Constants import Constants
from ProcessManager import ProcessManager

print(sys.version)

# logging setup
# with open('config/logging_config.yaml', 'r') as yaml_file:
#     logging_config = yaml.safe_load(yaml_file)
# for _, handler in logging_config['handlers'].items():
#     if 'filename' in handler:
#         timestamp = datetime.now().strftime(r'%m%d_%H%M%S')
#         handler['filename'] = str(f'log/{timestamp}.log')
# logging.config.dictConfig(logging_config)

logging.getLogger().setLevel(logging.DEBUG)

NetworkTables.startClientTeam(3566)

if Constants.noRoboRIO:
    NetworkTables.initialize()

# create process managers
localizationCameraTable = NetworkTables.getTable("localizationCamera");
localizationCamera_last_update_time = localizationCameraTable.getNumber("last_update_time", 0.0)


def localizationCamera_is_updated():
    global localizationCamera_last_update_time
    global localizationCameraTable

    current_update_time = localizationCameraTable.getNumber("last_update_time", 0.0)

    if localizationCamera_last_update_time == current_update_time:
        return False
    elif localizationCamera_last_update_time < current_update_time:
        localizationCamera_last_update_time = current_update_time
        return True
    else:
        logging.error("Localization Camera Time Error")
        return False


intakeCameraTable = NetworkTables.getTable("IntakeCamera");
intakeCamera_last_update_time = intakeCameraTable.getNumber("last_update_time", 0.0)


def intakeCamera_is_updated():
    global intakeCamera_last_update_time
    global intakeCameraTable

    current_update_time = localizationCameraTable.getNumber("last_update_time", 0.0)

    if intakeCamera_last_update_time == current_update_time:
        return False
    elif intakeCamera_last_update_time < current_update_time:
        intakeCamera_last_update_time = current_update_time
        return True
    else:
        logging.error("Intake Camera Time Error")
        return False


shooterCameraTable = NetworkTables.getTable("ShooterCamera");
shooterCamera_last_update_time = shooterCameraTable.getNumber("last_update_time", 0.0)


def shooterCamera_is_updated():
    global shooterCamera_last_update_time
    global shooterCameraTable

    current_update_time = localizationCameraTable.getNumber("last_update_time", 0.0)

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

    localizationCamera_process_manager = ProcessManager(lambda: LocalizationCameraProcess())

    time.sleep(4) # must have t265 launch first to work

    intakeCameraProcesManager = ProcessManager(lambda: IntakeCameraProcess())

    shooterCameraProcessManager = ProcessManager(lambda: ShooterCameraProcess())

    # main loop
    while True:

        if (NetworkTables.isConnected() == False):
            NetworkTables.initialize(server='10.35.66.2')
            if Constants.noRoboRIO:
                NetworkTables.initialize()

        # get pose
        # localizationCamera_process_manager.checkin(localizationCamera_is_updated())

        # get CV
        # intakeCameraProcesManager.checkin(intakeCamera_is_updated())

        shooterCameraProcessManager.checkin(shooterCamera_is_updated())
