from _pynetworktables.table import NetworkTable
from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time
import logging
import os

def main():

   with open('/boot/frc.json') as f:
      config = json.load(f)
   camera = config['cameras'][0]

   width = camera['width']
   height = camera['height']

   CameraServer.getInstance().startAutomaticCapture()

   input_stream = CameraServer.getInstance().getVideo()
   #input_stream = cv2.VideoCapture(0)

   output_stream = CameraServer.getInstance().putVideo('Processed', width, height)
   binary_stream = CameraServer.getInstance().putVideo('Binary', width, height)

   NetworkTables.startClientTeam(3566)
   logging.basicConfig(level=logging.DEBUG)

   # NetworkTables.initialize(server='10.35.66.2')

   # Table for vision output information
   vision_nt = NetworkTables.getTable('GoalCamera')

   # Allocating new images is very expensive, always try to preallocate
   img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

   # Wait for NetworkTables to start
   time.sleep(0.5)

   #preallocate, get shape
   frame_time, input_img = input_stream.grabFrame(img)
   hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

   hsv_height, hsv_width, channels = hsv_img.shape

   x_mid = hsv_width // 2
   y_mid = hsv_height // 2

   FOV = 60

   while True:
      if(NetworkTables.isConnected() == False):
         NetworkTables.initialize(server='10.35.66.2')
      
      start_time = time.time()

      hsv_min = (57, 100, 24)
      hsv_max = (84, 255, 255)

      h_min = vision_nt.getAutoUpdateValue("h_min", defaultValue = hsv_min[0]).getNumber(defaultValue = hsv_min[0])
      s_min = vision_nt.getAutoUpdateValue("s_min", defaultValue = hsv_min[1]).getNumber(defaultValue = hsv_min[1])
      v_min = vision_nt.getAutoUpdateValue("v_min", defaultValue = hsv_min[2]).getNumber(defaultValue = hsv_min[2])

      h_max = vision_nt.getAutoUpdateValue("h_max", defaultValue = hsv_max[0]).getNumber(defaultValue = hsv_max[0])
      s_max = vision_nt.getAutoUpdateValue("s_max", defaultValue = hsv_max[1]).getNumber(defaultValue = hsv_max[1])
      v_max = vision_nt.getAutoUpdateValue("v_max", defaultValue = hsv_max[2]).getNumber(defaultValue = hsv_max[2])

      hsv_min = (h_min, s_min, v_min)
      hsv_max = (h_max, s_max, v_max)

      frame_time, input_img = input_stream.grabFrame(img)
      
      # input_stream = cv2.flip(input_stream, 0)
      
      output_img = np.copy(input_img)

      # Notify output of error and skip iteration
      if frame_time == 0:
         output_stream.notifyError(input_stream.getError())
         continue

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
         # if area / w / h < Constants.MIN_TARGET2RECT_RATIO:
         #    continue

         cv2.drawContours(output_img, contour, -1, color = (255, 255, 255), thickness = -1)
         
         rect = cv2.minAreaRect(contour)
         center, size, angle = rect
         center = [int(dim) for dim in center] # Convert to int so we can draw

         x_list.append((center[0] - width / 2) / (width / 2))
         y_list.append((center[1] - height / 2) / (height / 2))

      vision_nt.putNumberArray('target_x', x_list)
      vision_nt.putNumberArray('target_y', y_list)

      cv2.circle(output_img, center = (x_mid, y_mid) , radius = 3, color = (0, 0, 255), thickness = -1)

      processing_time = time.time() - start_time
      fps = 1 / processing_time
      cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
      # output_stream.putFrame(output_img)

      output_stream.putFrame(output_img)

      binary_stream.putFrame(binary_img)

      # print(hsv_img.shape)
      # print(hsv_width, " ", hsv_height)
      # print(x_mid, " ", y_mid)


      primaryTarX = 0.0

      #get angle
      if(len(x_list) != 0):
         primaryTarX = x_list[0]
      
      tarAngle = FOV / 2 * primaryTarX
      vision_nt.putNumber("angle", tarAngle)

      print(NetworkTables.isConnected())
      print(hsv_img[60, 80,0], hsv_img[60, 80,1], hsv_img[60, 80,2], sep=' ')

      print(x_list)

      print(os.path.dirname(os.path.abspath(__file__)))

main()





