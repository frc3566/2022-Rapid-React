import cv2
import numpy as np
import math
import cv2
import json
import time

from cscore import CameraServer
from networktables import NetworkTables

from util.showImage import show

from util.transformation import rotationMatrixToEulerAngles
from Constants import Constants

with open('/boot/frc.json') as f:
    config = json.load(f)
camera = config['cameras'][0]

width = camera['width']
height = camera['height']

CameraServer.getInstance().startAutomaticCapture()

input_stream = CameraServer.getInstance().getVideo()
# input_stream = cv2.VideoCapture(0)

output_stream = CameraServer.getInstance().putVideo('Processed', width, height)
binary_stream = CameraServer.getInstance().putVideo('Binary', width, height)

NetworkTables.startClientTeam(3566)

# NetworkTables.initialize(server='10.35.66.2')

# Table for vision output information
vision_nt = NetworkTables.getTable('GoalCamera')

# Allocating new images is very expensive, always try to preallocate
img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

# Wait for NetworkTables to start
time.sleep(0.5)

# preallocate, get shape
frame_time, frame = input_stream.grabFrame(img)
frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

while True:
    if (NetworkTables.isConnected() == False):
        NetworkTables.initialize(server='10.35.66.2')
    frame_yaw = 0

    frame_time, frame = input_stream.grabFrame(img)

    # HSV filtering
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    thresh = cv2.inRange(frame_HSV, Constants.HSV_LOW, Constants.HSV_HIGH)

    # thresh = cv2.blur(thresh, (5, 5)) # may not be necessary

    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

    # print(contours)

    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    thresh = cv2.addWeighted(thresh, 0.6, frame_HSV, 0.4, 0)

    # find target contour
    target = None
    for contour in contours:
        # start prelim checks
        area = cv2.contourArea(contour)

        if area < Constants.MIN_TARGET_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if area / w / h > Constants.MAX_TARGET2RECT_RATIO:
            continue
        epsilon = 0.004 * cv2.arcLength(contour, closed=True)
        approx = cv2.approxPolyDP(contour, epsilon, closed=True)
        if len(approx) < 8:
            continue

        cv2.drawContours(thresh, [contour], -1, (255, 0, 0))

        # start solvepnp check
        # four corners have largest dot product with direction vector
        arg_extreme_points = (Constants.EXTREME_VECTOR @ contour[:, 0, :].T).argmax(axis=-1)
        extreme_points = np.take(contour, arg_extreme_points, axis=0).squeeze()
        ret, rvec, tvec = cv2.solvePnP(Constants.TARGET_3D,
                                       extreme_points.astype(np.float32),
                                       Constants.CAMERA_MATRIX,
                                       Constants.DISTORTION_COEF)
        if not ret:
            continue

        # calculate field coordinate
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        field_xyz = np.matmul(rotation_matrix.T, -tvec).flatten()
        field_x, field_y, field_z = field_xyz

        # calculate only 2d distance
        target_distance = math.hypot(field_x, field_y)
        if target_distance > Constants.MAX_TARGET_DISTANCE \
                or target_distance < Constants.MIN_TARGET_DISTANCE:
            continue

        # camera orientation in world coord
        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix.T)
        pitch += 90
        yaw += 90
        if abs((pitch - Constants.PITCH + 180) % 360 - 180) > \
                Constants.MAX_ALLOWABLE_YPR_DIFF\
                or abs((roll - Constants.ROLL + 180) % 360 - 180) > \
                Constants.MAX_ALLOWABLE_YPR_DIFF:
            print(f"orientation error with yaw {yaw:.2f}"
                                f" pitch {pitch:.2f} roll {roll:.2f}")
            continue

        for c, v in zip("xyztpr", [field_x, field_y, field_z, yaw, pitch, roll]):
            print(f'CV/camera_{c}', v)

        # transform to WPI robotics convention
        y, z, x = tvec[:, 0] * (-1, -1, 1)
        target_relative_dir_left = math.atan2(y, x) / math.pi * 180
        target_t265_azm = frame_yaw + target_relative_dir_left
        field_theta = math.atan2(field_y, field_x) / math.pi * 180 \
                      - 180 - target_relative_dir_left

        # found the correct target
        print("field_coord:" + ''.join(f"{v:7.2f}" for v in field_xyz))
        print(f'distance {target_distance:5.2f} '
                   f'toleft {target_relative_dir_left:5.1f} '
                   f'field_theta {field_theta:5.1f} ')
        # if found more than one target reject all
        for point in extreme_points:
            cv2.circle(thresh, tuple(point), 4, (0, 0, 255), -1)
        if target is not None:
            target = None
            print("double target")
            break
        target = (True,
                  target_distance,
                  target_relative_dir_left,
                  target_t265_azm,
                  [field_x, field_y, field_theta])

    # show("color_threashed", thresh)
    # show("color", frame_HSV)

    output_stream.putFrame(frame_HSV)
    binary_stream.putFrame(thresh)