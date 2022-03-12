import multiprocessing as mp
import cv2
import numpy as np
import time
import logging
import math
import sys
from Constants import Constants
from multiprocessing import Queue
from queue import Full, Empty


class ShooterCameraProcess(mp.Process):

    def __init__(self, nt_queue, frame_in_queue, frame_out_queue, end_queue):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.goal_detected = False
        self.nt_queue = nt_queue
        self.frame_in_queue = frame_in_queue
        self.frame_out_queue = frame_out_queue
        self.end_queue = end_queue

    def process_method(self):
        # Wait for NetworkTables to start
        time.sleep(0.5)

        while True:
            try:
                message = self.end_queue.get_nowait()
                if message == "END":
                    sys.exit()
            except Empty:
                pass

            # Notify output of error and skip iteration
            try:
                frame_time, input_img = self.frame_in_queue.get_nowait()
                rv = self.process_image(input_img)
                for key, value in rv['network_table'].items():
                    self.nt_queue.put_nowait((key, value))
                self.frame_out_queue.put_nowait(rv['binary_img'])
            except Exception as e:
                self.logger.error(str(e))

    def process_image(self, input_img):
        output_img = input_img.copy()
        start_time = time.time()
        goal_detected = False
        hsv_min = (60, 80, 15)
        hsv_max = (90, 255, 255)

        # Convert to HSV and threshold image
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        binary_img = cv2.inRange(hsv_img, hsv_min, hsv_max)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

        x_list = []
        y_list = []

        y_min = 1000
        x_tar = 0
        y_tar = 0

        for contour in contour_list:
            # Ignore small contours that could be because of noise/bad thresholding
            area = cv2.contourArea(contour)
            if area < 10:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if area / w / h < Constants.MIN_TARGET2RECT_RATIO:
                continue


            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            center = [int(dim) for dim in center]  # Convert to int so we can draw

            x_list.append(center[0])
            y_list.append(center[1])

        if len(x_list) == 0 or len(y_list) == 0:
            goal_detected = False
        else:
            goal_detected = True

        # chenyx code starts
        centers = np.array([x_list, y_list]).T # shape (n, 2)
        def find_target_index(centers):
            """
            This function finds the main cluster (a cluster of n / 2 points)
            :param centers: np array of shape (n, 2), giving the centers of
            all contours, n MUST NOT be 0
            :return: the center of the contour with the lowest y value
            after filter
            """
            # return index
            num_contour = len(centers)
            assert(num_contour != 0)
            if num_contour == 1:
                return 0
            if num_contour == 2:
                return centers[:, 1].argmin()
            center_errors = np.zeros((num_contour,))
            for center_index, center in enumerate(centers):
                other_points = np.array([
                    centers[i] for i in range(len(centers))
                    if i != center_index])
                errors = np.sum((other_points - center) ** 2, axis=1)
                sorted_error = np.sort(errors)
                center_errors[center_index] = sorted_error[num_contour // 2]
            center_index = np.argmin(center_errors)
            center_error = center_errors[center_index]
            center = centers[center_index]
            errors = np.sum((centers - center) ** 2, axis=1)
            valid_points = centers[errors <= center_error]
            return np.argmin(valid_points[:, 1])

        if len(centers) > 0:
            index = find_target_index(centers)
            x_tar, y_tar = centers[index]
            cv2.drawContours(output_img, contour_list[index], -1,
                             color=(255, 128, 128), thickness=-1)
        # chenyx code ends

        # this is the dis used
        dis = 1.162 * y_tar + 78.171
        # this is the angle used
        y_mid = 59
        x_mid = 79  # pixel
        u = x_tar - x_mid
        x_angle = math.atan(u / Constants.FOCAL_LENGTH_X)
        x_angle = math.degrees(x_angle)

        # legacy
        v = y_tar - y_mid
        y_angle = math.radians(Constants.CAMERA_MOUNT_ANGLE) - math.atan(v / Constants.FOCAL_LENGTH_Y)
        distance = Constants.CAMERA_GOAL_DELTA_H / math.tan(y_angle)
        y_angle = math.degrees(y_angle)
        processing_time = time.time() - start_time
        fps = 1 / processing_time
        # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))

        return {
            "network_table": {
                "last_update_time": time.time() % 2048,
                "processing_time": processing_time,
                "fps": fps,
                "goal_detected": goal_detected,
                "x_angle": x_angle,
                "y_angle": y_angle,
                "distance": distance,
                "y_min": y_tar,
                "line_best_fit": dis
            },
            "binary_img": binary_img,
            "output_img": output_img,
        }


    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")


if __name__ == '__main__':
    process = ShooterCameraProcess(None, None, None, None)
    camera = cv2.VideoCapture(0)  # change index if necessary
    while True:
        ret, frame = camera.read()
        # import pdb; pdb.set_trace()
        cv2.imshow("output_image", frame)
        cv2.waitKey(0)
        rv = process.process_image(frame)
