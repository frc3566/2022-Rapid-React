import numpy as np


class Constants:
    DEBUG = True

    isRed = False # True = Red, False = Blue

    # microsoft cam
    EXPOSURE_AUTO = 1  # 1 off 3 on
    EXPOSURE_ABS = 5  # 5-20000, use v4l2 to check
    WIDTH = 640
    HEIGHT = 480
    CAMERA_MATRIX = np.array(
        [[726.80788453, 0., 349.5506398],
         [0., 725.12121263, 216.65755001],
         [0., 0., 1.]]
    )

    # DISTORTION_COEF = np.array([0.434228643, -12.2270248, -0.0078620537,
    #                             0.0164108833, 99.6424132])

    FOCAL_LENGTH_X = CAMERA_MATRIX[0][0]
    FOCAL_LENGTH_Y = CAMERA_MATRIX[1][1]

    CAMERA_MOUNT_ANGLE = 50 # 41
    CAMERA_HEIGHT = 0.91
    CAMERA_GOAL_DELTA_H = 2.64 - CAMERA_HEIGHT

    # camera
    # matrix:
    # [[726.80788453   0.         349.5506398]
    #  [0.         725.12121263 216.65755001]
    # [0.
    # 0.
    # 1.]]
    # distortion
    # coefficients: [4.34228643e-01 - 1.22270248e+01 - 7.86205370e-03  1.64108833e-02
    #                9.96424132e+01]

    H_MIN = 57
    H_MAX = 84
    S_MIN = 100
    S_MAX = 255
    V_MIN = 24
    V_MAX = 255
    HSV_LOW = (H_MIN, S_MIN, V_MIN)
    HSV_HIGH = (H_MAX, S_MAX, V_MAX)

    # CV - target in WPI world-coord
    # 2022 rapic react vision target
    # origin at the center of the target circle
    TARGET_3D = np.array([[0.0635, 0.178, 0.6755],  # top left
                          [0.0635, 0, 0.6755],  # bottom left
                          [-0.0635, 0, 0.6755],  # bottom right
                          [-0.0635, 0.178, 0.6755]])  # top right

    MIN_TARGET_AREA = 180
    MIN_TARGET2RECT_RATIO = 0.25
    MAX_TARGET_DISTANCE = 10
    MIN_TARGET_DISTANCE = 2
    EXTREME_VECTOR = np.array([[-2.5, 1], [-2.5, -1], [2.5, -1], [2.5, 1]])
    # TopLeft, ButLeft, ButRight, TopRight

    PITCH = 18

    # Connection, these constants may be changed for different process
    DISCONNECT_DURATION = 1.0  # sec
    RESTART_DURATION = 10.0  # sec

    MA_MOMENTUM = 0.9
    UPDATE_PERIOD = 3  # sec
