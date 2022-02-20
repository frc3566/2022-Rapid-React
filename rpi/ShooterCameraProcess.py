import multiprocessing as mp
import cv2
import numpy as np
import time
import logging
import math
from Constants import Constants
from multiprocessing import Queue
from queue import Full, Empty


class ShooterCameraProcess(mp.Process):

    def __init__(self, nt_queue, frame_in_queue, frame_out_queue):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.goal_detected = False
        self.nt_queue = nt_queue
        self.frame_in_queue = frame_in_queue
        self.frame_out_queue = frame_out_queue

    def process_method(self):

        width = 640
        height = 480

        # Allocating new images is very expensive, always try to preallocate
        input_img = np.zeros(shape=(640, 480, 3), dtype=np.uint8)

        # Wait for NetworkTables to start
        time.sleep(0.5)

        # preallocate, get shape
        # try:
        #     frame_time, input_img = self.frame_in_queue.get_nowait()
        # except Empty:
        #     # self.logger.error("no frame")
        #     pass
        # self.logger.debug("yes frame")

        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

        width = 640
        height = 480

        x_mid = width // 2
        y_mid = height // 2

        goal_detected = False

        # loop forever
        while True:

            start_time = time.time()

            hsv_min = (57, 100, 24)
            hsv_max = (84, 255, 255)

            # Notify output of error and skip iteration
            try:
                frame_time, input_img = self.frame_in_queue.get_nowait()
            except Empty:
                # self.logger.error("no frame")
                continue

            # print("yes frame")

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

            if len(x_list) == 0 or len(y_list) == 0:
                goal_detected = False
                continue
            else:
                goal_detected = True

            x_angle = math.atan((center[0] - x_mid) / Constants.FOCAL_LENGTH_X)
            y_angle = math.atan((center[1] - y_mid) / Constants.FOCAL_LENGTH_Y) + Constants.CAMERA_MOUNT_ANGLE

            distance = Constants.CAMERA_GOAL_DELTA_H / math.tan(y_angle)

            processing_time = time.time() - start_time
            fps = 1 / processing_time
            # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))

            # update nt
            # self.nt.putNumber("last_update_time", time.time())
            #
            # self.nt.putNumber("processing_time", processing_time)
            # self.nt.putNumber("fps", fps)
            #
            # self.nt.putBoolean("goal_detected", goal_detected)
            #
            # self.nt.putNumber("x_angle", x_angle)
            # self.nt.putNumber("y_angle", y_angle)
            # self.nt.putNumber("distance", distance)
            #
            # output_stream.putFrame(output_img)
            # binary_stream.putFrame(binary_img)

            try:
                self.nt_queue.put_nowait(("last_update_time", time.time()))

                self.nt_queue.put_nowait(("processing_time", processing_time))
                self.nt_queue.put_nowait(("fps", fps))

                self.nt_queue.put_nowait(("goal_detected", goal_detected))

                self.nt_queue.put_nowait(("x_angle", x_angle))
                self.nt_queue.put_nowait(("y_angle", y_angle))
                self.nt_queue.put_nowait(("distance", distance))

                # print(goal_detected, x_angle, y_angle, distance)
            except Full:
                logging.error("shooter nt full")
                pass

            try:
                self.frame_out_queue.put_nowait(output_img)
            except Full:
                pass


    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")


