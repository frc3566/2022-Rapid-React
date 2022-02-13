import multiprocessing as mp
import cv2
import numpy as np
import time
import logging
import math

from networktables import NetworkTables
from Constants import Constants
from cscore import CameraServer


class ShooterCameraProcess(mp.Process):

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.nt = NetworkTables.getTable('LiveWindow/ShooterCamera')
        self.goal_detect = False

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

        # loop forever
        while True:

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
            # output_stream.putFrame(output_img)

            output_stream.putFrame(output_img)

            binary_stream.putFrame(binary_img)

            # update nt
            self.nt.putNumber("last_update_time", time.time())

            self.nt.putNumber("processing_time", processing_time)
            self.nt.putNumber("fps", fps)

            self.nt.putBoolean("goal_detected", goal_detected)

            self.nt.putNumber("x_angle", x_angle)
            self.nt.putNumber("y_angle", y_angle)
            self.nt.putNumber("distance", distance)

            NetworkTables.flush()

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")


#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }


