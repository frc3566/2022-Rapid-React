import math as m
import time
import pyrealsense2 as rs
import numpy as np
import numpy.linalg as la
import cv2
import random
import multiprocessing as mp
import logging
from cscore import CameraServer
from util.SphereFitting import *
from statistics import mean
from util.showImage import show
from networktables import NetworkTables

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

# red
red_hMin = 0
red_hMax = 5
red_sMin = 150
red_sMax = 255
red_vMin = 0
red_vMax = 255

# blue
blue_hMin = 90
blue_hMax = 100
blue_sMin = 70
blue_sMax = 255
blue_vMin = 0
blue_vMax = 255

HEIGHT = 17 * 0.0254

output_stream = CameraServer.getInstance().putVideo('IntakeCamera', DEPTH_W, DEPTH_H)

class IntakeCameraProcess(mp.Process):

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.nt = NetworkTables.getTable('IntakeCamera')


    def get_intrinsics(self, profile):
        intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        K = np.array([
            [intrin.fx, 0, intrin.ppx],
            [0, intrin.fy, intrin.ppy],
            [0, 0, 1]
        ])
        K_inv = la.inv(K)
        return K, K_inv


    def calculate_plane_distance(self, points, plane):
        return (np.dot(points, plane[:3]) + plane[3]) / np.sqrt(
            plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)


    def circle_sample(self, image_3d, x, y, r):
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

    def process_method(self):

        pipeline = rs.pipeline()
        config = rs.config()

        # config.enable_device('f1230148')

        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
        config.enable_stream(rs.stream.color, DEPTH_W, DEPTH_H, rs.format.bgr8, FPS)

        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        DEPTH_UNIT = depth_sensor.get_option(rs.option.depth_units)
        print(DEPTH_UNIT)

        # profile = config.resolve(rs.pipeline_wrapper(pipeline_d435))

        # get camera intrinsics matrix
        K, K_inv = self.get_intrinsics(profile)

        # define plane detecting computers
        normal_computer = cv2.rgbd.RgbdNormals_create(DEPTH_H, DEPTH_W, cv2.CV_32F, K)
        plane_computer = cv2.rgbd.RgbdPlane_create(
            cv2.rgbd.RgbdPlane_RGBD_PLANE_METHOD_DEFAULT,
            int(DEPTH_W * DEPTH_H / 5000), int(DEPTH_W * DEPTH_H / 15), 0.013,
            0.01, 0, 0  # quadratic error
        )

        # setup preset
        preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
        for i in range(int(preset_range.max)):
            visulpreset = depth_sensor.get_option_value_description(
                rs.option.visual_preset, i)
            # if visulpreset == 'Default':
            #     print('set default')
            #     depth_sensor.set_option(rs.option.visual_preset, i)

        try:
            while True:
                frames = pipeline.wait_for_frames()

                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    raise Exception(
                        "depth_frame and/or color_frame unavailable")
                color_img = np.asanyarray(color_frame.get_data())
                # color_img = color_img[:, :, ::-1]

                # Convert images to numpy arrays
                depth_frame = hole_filler.process(depth_frame)
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_image = depth_image * DEPTH_UNIT

                hsv_color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

                red_thresh_img = cv2.inRange(hsv_color_img, (red_hMin, red_sMin, red_vMin),
                                               (red_hMax, red_sMax, red_vMax))

                red_thresh_img = cv2.dilate(red_thresh_img, None, iterations=4)
                red_thresh_img = cv2.erode(red_thresh_img, None, iterations=2)

                blue_thresh_img = cv2.inRange(hsv_color_img, (blue_hMin, blue_sMin, blue_vMin),
                                               (blue_hMax, blue_sMax, blue_vMax))

                blue_thresh_img = cv2.dilate(blue_thresh_img, None, iterations=4)
                blue_thresh_img = cv2.erode(blue_thresh_img, None, iterations=2)

                color_thresh_img = (red_thresh_img, blue_thresh_img)

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

                # 0 is red, 1 is blue

                ball_x_list = []*2
                ball_y_list = []*2

                ball_dis = (1e9, 1e9)
                ball_angle = (0, 0)

                colors = ((200, 0, 0), (0, 0, 200))

                for i in range(2):
                    best_score = 1e9

                    color_masked = np.logical_or(valid_mask, unknown_mask).astype(np.uint8) * color_thresh_img[i]
                    color_masked = cv2.morphologyEx(color_masked, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                    color_masked = cv2.morphologyEx(color_masked, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

                    contours, _ = cv2.findContours(color_masked, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)

                    for index, contour in enumerate(contours):
                        contour_area = cv2.contourArea(contour)
                        if contour_area < MIN_CONTOUR_SIZE:
                            continue
                        (x_2d, y_2d), r = cv2.minEnclosingCircle(contour)
                        # print(contour_area / (m.pi * r * r))
                        if contour_area / (m.pi * r * r) < 0.55:
                            continue
                        dis = self.circle_sample(image_3d, x_2d, y_2d, r)
                        if dis < MIN_DIS:
                            continue

                        if not DIS_RADIUS_PRODUCT_MIN < dis * r < DIS_RADIUS_PRODUCT_MAX:
                            print(f"raidus distance ratio skip {dis * r:.3f}")
                            continue

                        x_2d, y_2d, r = int(x_2d), int(y_2d), int(r)

                        contour_mask = np.zeros((DEPTH_H, DEPTH_W), dtype=np.uint8)
                        contour_mask = cv2.circle(contour_mask, (x_2d, y_2d), int(r * 0.9), 255, -1)

                        final_mask = (color_masked == 255) * valid_mask * (contour_mask == 255)
                        # show("final mask", final_mask.astype(np.uint8) * 255)
                        points = image_3d[final_mask]

                        # print(f"min dis to ball {depth_image[contour_mask].min()}")

                        confidence, sphere = fit_sphere_LSE_RANSAC(points)

                        sphere_r, center_x, center_y, center_z = sphere
                        center_dis = (center_x ** 2 + center_y ** 2 + center_z ** 2) ** 0.5

                        print(f"{confidence:.5f} {sphere_r:.5f} {dis:.5f} {center_dis:.5f}")
                        if confidence < 0.4 or sphere_r > 2 or sphere_r < 0.05:
                            # print("skip")
                            continue

                        # print(f"{confidence:.5f} {sphere_r:.5f} {dis:.5f} {center_dis:.5f}")

                        pt_3d = center_x, center_y, center_z

                        ball_x_list[i].append(center_x)
                        ball_y_list[i].append(center_y)

                        # convert for output
                        dis_2d = (center_x ** 2 + center_z ** 2) ** 0.5
                        angle = m.degrees(m.atan(pt_3d[0] / dis_2d))

                        score = dis_2d + abs(angle) / 40
                        if score < best_score:
                            ball_dis[i] = dis_2d
                            ball_angle[i] = angle
                            best_score = score
                            # print(ball_dis, -ball_angle, sep=" ")

                        cv2.drawContours(color_img, contours, index, (200, 0, 200), 2)
                        cv2.circle(color_img, (x_2d, y_2d), r, colors[i], 2)

                    # print(f"dis * r: {dis * r}")
                    # print(f"pi * r^2: {np.pi * (r ** 2)}")

                # color_thresh_img = np.stack((color_thresh_img, color_thresh_img, color_thresh_img), axis=-1)
                # color_masked = np.stack((color_masked, color_masked, color_masked), axis=-1)

                # show('color_masked', color_masked)

                # show('color', color_img)


                #update nt
                self.nt.putNumber("last_update_time", time.time())

                self.nt.putNumber("red_ball_distance", ball_dis[0])
                self.nt.putNumber("red_ball_angle", ball_angle[0])

                self.net.putNumberArray("red_ball_x_list", ball_x_list[0])
                self.net.putNumberArray("red_ball_y_list", ball_y_list[0])

                self.nt.putNumber("blue_ball_distance", ball_dis[1])
                self.nt.putNumber("blue_ball_angle", ball_angle[1])

                self.net.putNumberArray("blue_ball_x_list", ball_x_list[1])
                self.net.putNumberArray("blue_ball_y_list", ball_y_list[1])

                output_stream.putFrame(color_img)

                self.nt.flush()

        finally:
            print('stop')
            pipeline.stop()

    def run(self):
        try:
            self.process_method()
        except Exception:
            self.logger.exception("exception uncaught in process_method, "
                                  "wait for root process to restart this")