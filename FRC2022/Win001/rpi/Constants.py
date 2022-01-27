import numpy as np

class Constants:
    H_MIN = 50 
    H_MAX = 70
    S_MIN = 240
    S_MAX = 255
    V_MIN = 190
    V_MAX = 210
    HSV_LOW = (H_MIN, S_MIN, V_MIN)
    HSV_HIGH = (H_MAX, S_MAX, V_MAX)

    FOV = 60

    MIN_TARGET_AREA = 15
    MIN_TARGET2RECT_RATIO = 0.5
