import math as m
import pyrealsense2 as rs
import numpy as np
import numpy.linalg as la
import cv2
from statistics import mean
from util.showImage import show

import random
from util.SphereFitting import *


align_to = rs.stream.color
align = rs.align(align_to)

hole_filler = rs.hole_filling_filter()

MIN_DIS = 0.25
MAX_DIS = 9

DEPTH_H = 480
DEPTH_W = 640
FPS = 30

DIS_RADIUS_PRODUCT_MIN = 40
DIS_RADIUS_PRODUCT_MAX = 100

MIN_CONTOUR_SIZE = 50

# hMin = 0
# hMax = 255
# sMin = 0
# sMax = 255
# vMin = 0
# vMax = 255

#red
hMin = 0
hMax = 5
sMin = 150
sMax = 255
vMin = 0
vMax = 255

#blue
# hMin = 90
# hMax = 100
# sMin = 70
# sMax = 255
# vMin = 0
# vMax = 255

HEIGHT = 17 * 0.0254

def get_intrinsics(profile):
    intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([
        [intrin.fx, 0, intrin.ppx],
        [0, intrin.fy, intrin.ppy],
        [0, 0, 1]
    ])
    K_inv = la.inv(K)
    return K, K_inv

def calculate_plane_distance(points, plane):
    return (np.dot(points, plane[:3]) + plane[3]) / np.sqrt(
        plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)

def circle_sample(image_3d, x, y, r):
    adjusted_r = int(1 / m.sqrt(2) * r * 0.8)
    dis_list = []
    for i in range(100):
        new_x = int(x + random.randint(0, adjusted_r * 2) - adjusted_r)
        new_y = int(y + random.randint(0, adjusted_r * 2) - adjusted_r)
        if not (0 <= new_x < DEPTH_W and 0 <= new_y < DEPTH_H):
            continue
        x_3d, y_3d, z_3d = image_3d[new_y, new_x]
        dis = m.sqrt(x_3d * x_3d + y_3d * y_3d + z_3d * z_3d)
        if not (MIN_DIS <= dis <= MAX_DIS):
            continue
        dis_list.append(dis)
    if len(dis_list) == 0:
        return 0
    return mean(dis_list)


pipeline_d435 = rs.pipeline()
config_d435 = rs.config()

# config_d435.enable_device('f1230148')

config_d435.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
config_d435.enable_stream(rs.stream.color, DEPTH_W, DEPTH_H, rs.format.bgr8, FPS)

profile_d435 = pipeline_d435.start(config_d435)
depth_sensor = profile_d435.get_device().first_depth_sensor()
# depth_sensor.set_option(rs.option.depth_units, 0.0001)
DEPTH_UNIT = depth_sensor.get_option(rs.option.depth_units)
print(DEPTH_UNIT)

#profile_d435 = config_d435.resolve(rs.pipeline_wrapper(pipeline_d435))

#get camera intrinsics matrix
K, K_inv = get_intrinsics(profile_d435)

#define plane detecting computers
normal_computer = cv2.rgbd.RgbdNormals_create(DEPTH_H, DEPTH_W, cv2.CV_32F, K)
plane_computer = cv2.rgbd.RgbdPlane_create(
    cv2.rgbd.RgbdPlane_RGBD_PLANE_METHOD_DEFAULT,
    int(DEPTH_W * DEPTH_H / 5000), int(DEPTH_W * DEPTH_H / 15), 0.013,
    0.01, 0, 0 # quadratic error
)

# def callback(value):
#     pass
#
# def setup_trackbars(range_filter):
#     cv2.namedWindow("Trackbars", 0)
#
#     for i in ["MIN", "MAX"]:
#         v = 0 if i == "MIN" else 255
#
#         for j in range_filter:
#             cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)
#
# def get_trackbar_values(range_filter):
#     values = []
#
#     for i in ["MIN", "MAX"]:
#         for j in range_filter:
#             v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
#             values.append(v)
#
#     return values
#
# range_filter = 'HSV'
# setup_trackbars(range_filter)

#setup preset
preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
for i in range(int(preset_range.max)):
    visulpreset = depth_sensor.get_option_value_description(
        rs.option.visual_preset, i)
    # if visulpreset == 'Default':
    #     print('set default')
    #     depth_sensor.set_option(rs.option.visual_preset, i)

try:
    while True:
        frames = pipeline_d435.wait_for_frames()

        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            raise Exception(
                "depth_frame and/or color_frame unavailable")
        color_img = np.asanyarray(color_frame.get_data())
        # color_img = color_img[:, :, ::-1]
        # print(color_img.dtype)
        # Convert images to numpy arrays
        depth_frame = hole_filler.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        # depth_image = depth_image/DEPTH_UNIT * 0.001
        depth_image = depth_image * DEPTH_UNIT

        # hMin, sMin, vMin, hMax, sMax, vMax = get_trackbar_values(range_filter)

        hsv_color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        color_thresh_img = cv2.inRange(hsv_color_img, (hMin, sMin, vMin),
                                       (hMax, sMax, vMax))

        color_thresh_img = cv2.dilate(color_thresh_img, None, iterations=4)
        color_thresh_img = cv2.erode(color_thresh_img, None, iterations=2)

        show('color_thresh_img', color_thresh_img)

        image_3d = cv2.rgbd.depthTo3d(depth_image, K)

        # show('image_3d', image_3d)

        unknown_mask = np.isnan(image_3d[..., -1])
        normal = normal_computer.apply(image_3d)
        plane_labels, plane_coeffs = plane_computer.apply(image_3d, normal)
        dis_to_cam = la.norm(image_3d, axis=-1)

        # print(dis_to_cam)

        valid_mask = (dis_to_cam < MAX_DIS) * (dis_to_cam > MIN_DIS) * \
                        (plane_labels == 255)

        # print(valid_mask)

        # print(valid_mask.shape)
        show("valid_mask", valid_mask.astype(np.uint8) * 255)

        
        color_masked = np.logical_or(valid_mask, unknown_mask).astype(np.uint8) * color_thresh_img
        color_masked = cv2.morphologyEx(color_masked, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        color_masked = cv2.morphologyEx(color_masked, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(color_masked, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        ball_dis = 1e9
        ball_angle = 0
        best_score = 1e9
        for index, contour in enumerate(contours):
            contour_area = cv2.contourArea(contour)
            if contour_area < MIN_CONTOUR_SIZE:
                continue
            (x_2d, y_2d), r = cv2.minEnclosingCircle(contour)
            # print(contour_area / (m.pi * r * r))
            if contour_area / (m.pi * r * r) < 0.55:
                continue
            dis = circle_sample(image_3d, x_2d, y_2d, r)
            if dis < MIN_DIS:
                continue

            if not DIS_RADIUS_PRODUCT_MIN < dis * r < DIS_RADIUS_PRODUCT_MAX:
                print(f"raidus distance ratio skip {dis * r:.3f}")
                continue

            x_2d, y_2d, r = int(x_2d), int(y_2d), int(r)

            contour_mask = np.zeros((DEPTH_H, DEPTH_W), dtype=np.uint8)
            contour_mask = cv2.circle(contour_mask, (x_2d, y_2d), int(r * 0.9), 255, -1)

            cv2.drawContours(color_img, contours, index, (200, 0, 200), 2)
            # cv2.circle(color_img, (x_2d, y_2d), int(r * 0.9), 255, 4)



            final_mask = (color_masked == 255) * valid_mask * (contour_mask == 255)
            show("final mask", final_mask.astype(np.uint8) * 255)
            points = image_3d[final_mask]

            # print(f"min dis to ball {depth_image[contour_mask].min()}")

            # contour_mask.tofile("contour_mask")
            # image_3d.tofile("image_3d")

            confidence, sphere = fit_sphere_LSE_RANSAC(points)

            sphere_r, center_x, center_y, center_z = sphere
            center_dis = (center_x ** 2 + center_y ** 2 + center_z ** 2) ** 0.5

            print(f"{confidence:.5f} {sphere_r:.5f} {dis:.5f} {center_dis:.5f}")
            if confidence < 0.4 or sphere_r > 2 or sphere_r < 0.05:
                # print("skip")
                continue

            # print(f"{confidence:.5f} {sphere_r:.5f} {dis:.5f} {center_dis:.5f}")

            pt_3d = K_inv @ [x_2d, y_2d, 1] * dis
            angle = m.degrees(m.atan(pt_3d[0] / dis))
            # convert dis to planer dis for output
            dis_2d = m.sqrt(max(0.1, dis * dis - HEIGHT * HEIGHT))
            score = dis_2d + abs(angle) / 40
            if score < best_score:
                ball_dis = dis_2d
                ball_angle = angle
                ball_circle = ((x_2d, y_2d), r)
                best_score = score
                print(ball_dis, -ball_angle, sep=" ")
            cv2.drawContours(color_img, contours, index, (200, 0, 200), 2)

            cv2.circle(color_img, (x_2d, y_2d), r, (0, 0, 200), 2)

            print(f"dis * r: {dis * r}")
            print(f"pi * r^2: {np.pi * (r ** 2)}")

        color_thresh_img = np.stack((color_thresh_img, color_thresh_img, color_thresh_img), axis=-1)
        color_masked = np.stack((color_masked, color_masked, color_masked), axis=-1)
        output = cv2.vconcat([color_thresh_img, color_masked, color_img])

        show('color_masked', color_masked)
        
        # show('output', output)

        # try:
            # self.ball_queue.put_nowait((ball_dis, -ball_angle,
                                        # frame_yaw - ball_angle))
            
        show('color', color_img)
        # self.putFrame("intake", color_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
        
        if cv2.waitKey(1) & 0xFF == ord('s'):
            numpy.save("")


finally:
    print('stop')
    pipeline_d435.stop()
