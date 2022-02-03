import cv2
import numpy as np
import math

from D435Process import show
from util.transformation import rotationMatrixToEulerAngles
from Constants import Constants

cap = cv2.VideoCapture(0) # change when debugging on local
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, Constants.EXPOSURE_AUTO)
cap.set(cv2.CAP_PROP_EXPOSURE, Constants.EXPOSURE_ABS)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Constants.HEIGHT)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, Constants.WIDTH)

while True:
    ret, frame = cap.read()
    if not ret:
        raise Exception('no frame')

    # HSV filtering
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(frame_HSV, Constants.HSV_LOW, Constants.HSV_HIGH)
    # thresh = cv2.blur(thresh, (5, 5)) # may not be necessary

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    thresh = cv2.addWeighted(thresh, 0.6, frame, 0.4, 0)

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
            self.logger.warning(f"orientation error with yaw {yaw:.2f}"
                                f" pitch {pitch:.2f} roll {roll:.2f}")
            continue

        for c, v in zip("xyztpr", [field_x, field_y, field_z, yaw, pitch, roll]):
            self.putNtable(f'CV/camera_{c}', v)

        # transform to WPI robotics convention
        y, z, x = tvec[:, 0] * (-1, -1, 1)
        target_relative_dir_left = math.atan2(y, x) / math.pi * 180
        target_t265_azm = frame_yaw + target_relative_dir_left
        field_theta = math.atan2(field_y, field_x) / math.pi * 180 \
                      - 180 - target_relative_dir_left

        # found the correct target
        self.debug("field_coord:" + ''.join(f"{v:7.2f}" for v in field_xyz))
        self.debug(f'distance {target_distance:5.2f} '
                   f'toleft {target_relative_dir_left:5.1f} '
                   f'field_theta {field_theta:5.1f} ')
        # if found more than one target reject all
        for point in extreme_points:
            cv2.circle(thresh, tuple(point), 4, (0, 0, 255), -1)
        if target is not None:
            target = None
            self.logger.warning("double target")
            break
        target = (True,
                  target_distance,
                  target_relative_dir_left,
                  target_t265_azm,
                  [field_x, field_y, field_theta])


    # show(thresh)
    self.putFrame('shoot', thresh)