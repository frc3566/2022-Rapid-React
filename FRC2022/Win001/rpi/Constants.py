import numpy as np

class Constants:
        # microsoft cam
        EXPOSURE_AUTO = 1  # 1 off 3 on
        EXPOSURE_ABS = 5  # 5-20000, use v4l2 to check
        WIDTH = 640
        HEIGHT = 480
        CAMERA_MATRIX = np.array(
            [[705.09765532, 0., 316.48307281],
             [0., 705.62034134, 223.3216818],
             [0., 0., 1.]]
        )
        DISTORTION_COEF = np.array([0.1226279, -0.43199166, 0.00196861,
                                    -0.00344178, -1.21531281])
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
                              [-0.0635, 0.178, 0.6755], #top right
                              [-0.0635, 0, 0.6755]])#bottom right
                              
        MIN_TARGET_AREA = 180
        MAX_TARGET2RECT_RATIO = 0.25
        MAX_TARGET_DISTANCE = 10
        MIN_TARGET_DISTANCE = 2
        EXTREME_VECTOR = np.array([[-1, -0.3], [-1, 2], [1, 2], [1, -0.3]])
        # TopLeft, ButLeft, ButRight, TopRight