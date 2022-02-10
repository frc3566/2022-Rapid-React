import multiprocessing as mp
import cv2
import json
import numpy as np
import time
import logging
import math
import os

from cscore import CameraServer
from networktables import NetworkTables
from Constants import Constants


class ShooterCameraProcess(mp.Process):

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.nt = NetworkTables.getTable('ShooterCamera')
        self.goal_detect = False

    def process_method(self):
        with open('/boot/frc.json') as f:
            config = json.load(f)
        camera = config['cameras'][0]

        width = camera['width']
        height = camera['height']

        # CameraServer.getInstance().startAutomaticCapture()

        # input_stream = CameraServer.getInstance().getVideo()
        input_stream = cv2.VideoCapture(0)

        output_stream = CameraServer.getInstance().putVideo('Processed', width, height)
        binary_stream = CameraServer.getInstance().putVideo('Binary', width, height)

        NetworkTables.startClientTeam(3566)
        logging.basicConfig(level=logging.DEBUG)

        # NetworkTables.initialize(server='10.35.66.2')

        # Allocating new images is very expensive, always try to preallocate
        img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

        # Wait for NetworkTables to start
        time.sleep(0.5)

        # preallocate, get shape
        # frame_time, input_img = input_stream.grabFrame(img)
        ret, input_img = input_stream.read()
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

        hsv_height, hsv_width, channels = hsv_img.shape

        x_mid = hsv_width // 2
        y_mid = hsv_height // 2

        FOV = 60

        while True:
            # if (NetworkTables.isConnected() == False):
            #     NetworkTables.initialize(server='10.35.66.2')

            start_time = time.time()

            hsv_min = (57, 100, 24)
            hsv_max = (84, 255, 255)

            # frame_time, input_img = input_stream.grabFrame(img)
            ret, input_img = input_stream.read()

            input_img = cv2.undistort(input_img, Constants.camera_matrix, Constants.dist_coefs)

            # input_stream = cv2.flip(input_stream, 0)

            output_img = np.copy(input_img)

            # Notify output of error and skip iteration
            if not ret:
                output_stream.notifyError(input_stream.getError())
                self.logger.exception("no frame")
                continue

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
            y_angle = math.atan((center[1] - y_mid) / Constants.FOCAL_LENGTH_y) + Constants.CAMERA_MOUNT_ANGLE

            distance = Constants.CAMERA_GOAL_DELTA_H / math.tan(y_angle)

            processing_time = time.time() - start_time
            fps = 1 / processing_time
            cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
            # output_stream.putFrame(output_img)

            output_stream.putFrame(output_img)

            binary_stream.putFrame(binary_img)



            # update nt
            self.nt.putNumber("last_update_time", time.time());

            self.nt.putNumber("processing_time", processing_time);
            self.nt.putNumber("fps", fps)

            self.nt.putBoolean("goal_detected", goal_detected)

            self.nt.putNumber("x_angle", x_angle)
            self.nt.putNumber("y_angle", y_angle)
            self.nt.putNUmber("distance", distance)

            self.nt.flush()



    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")
