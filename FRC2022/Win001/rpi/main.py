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

from T265Process import T265Process
from IntakeCameraProcess import IntakeCameraProcess
from TargetCameraProcess import TargetCameraProcess
from Constants import Constants
from ProcessManager import ProcessManager


# logging setup
with open('config/logging_config.yaml', 'r') as yaml_file:
    logging_config = yaml.safe_load(yaml_file)
for _, handler in logging_config['handlers'].items():
    if 'filename' in handler:
        timestamp = datetime.now().strftime(r'%m%d_%H%M%S')
        handler['filename'] = str(f'log/{timestamp}.log')
logging.config.dictConfig(logging_config)


def t265_is_updating():
    return True

def IntakeCamera_is_updating():
    return True

def TargetCamera_is_updating():
    return True

# processes

if __name__ == '__main__':

    t265_process_manager = ProcessManager(lambda: T265Process(), t265_is_updating)

    time.sleep(4) # must have t265 launch first to work

    IntakeCameraProcesManager = ProcessManager(lambda: IntakeCameraProcess(), IntakeCamera_is_updating)

    TargetCameraProcessManager = ProcessManager(lambda: TargetCameraProcess(), TargetCamera_is_updating)

    # main loop
    while True:
        # get pose
        t265_process_manager.update()

        # get CV
        IntakeCameraProcesManager.update()

        # TargetCameraProcessManager.update()
