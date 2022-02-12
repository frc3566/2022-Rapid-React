import numpy as np

class Constants:
        # microsoft cam
        EXPOSURE_AUTO = 1  # 1 off 3 on
        EXPOSURE_ABS = 5  # 5-20000, use v4l2 to check
        WIDTH = 640
        HEIGHT = 480
        # CAMERA_MATRIX = np.array(
        #     [[705.09765532, 0., 316.48307281],
        #      [0., 705.62034134, 223.3216818],
        #      [0., 0., 1.]]
        # )
        #
        # DISTORTION_COEF = np.array([0.1226279, -0.43199166, 0.00196861,
        #                             -0.00344178, -1.21531281])

        CAMERA_MATRIX = np.array(
                [[1963.32664, 0., 871.03963],
                 [0., 1, 976.75769, 580.232988],
                 [0., 0., 1.]]
        )

        DIST_COEF = np.array([0.247965266, -12.7875847, -0.00291252742,
                              0.0030271429, 192.291259])

        FOCAL_LENGTH_X = CAMERA_MATRIX[0][0]
        FOCAL_LENGTH_Y = CAMERA_MATRIX[1][1]

        #TODO measure these
        CAMERA_MOUNT_ANGLE = 30
        CAMERA_GOAL_DELTA_H = 100

        # camera
        # matrix:
        # [[1.96332664e+03 0.00000000e+00 8.71039630e+02]
        #  [0.00000000e+00 1.97675769e+03 5.80232988e+02]
        # [0.00000000e+00
        # 0.00000000e+00
        # 1.00000000e+00]]
        # distortion
        # coefficients: [2.47965266e-01 - 1.27875847e+01 - 2.91252742e-03  3.02714290e-03
        #                1.92291259e+02]

        H_MIN = 57
        H_MAX = 84
        S_MIN = 100
        S_MAX = 255
        V_MIN = 24
        V_MAX = 255
        HSV_LOW = (H_MIN, S_MIN, V_MIN)
        HSV_HIGH = (H_MAX, S_MAX, V_MAX)

        # CV - target in WPI world-coord
        #2022 rapic react vision target
        #origin at the center of the target circle
        TARGET_3D = np.array([[0.0635, 0.178, 0.6755],  #top left
                              [0.0635, 0, 0.6755],  #bottom left
                              [-0.0635, 0, 0.6755],  # bottom right
                              [-0.0635, 0.178, 0.6755]]) #top right

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

        DEBUG = False
        MA_MOMENTUM = 0.9
        UPDATE_PERIOD = 3  # sec

        # sets the nt to server when testing without roboRIO
        noRoboRIO = True