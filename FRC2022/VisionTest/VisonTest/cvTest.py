import cv2
import json
import numpy as np
import time

def main():
   input_steam = cv2.VideoCapture(0)


   ret, input_img = input_steam.read()
   output_img = np.copy(input_img)

   height, width, channels = input_img.shape

   x_mid = width // 2
   y_mid = height // 2

   while True:

      ret, input_img = input_steam.read()
      output_img = np.copy(input_img)

      # Convert to HSV and threshold image
      hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
      binary_img = cv2.inRange(hsv_img, (0,100,150), (20,140,220))

      contour_list, _= cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

      for contour in contour_list:

         # Ignore small contours that could be because of noise/bad thresholding
         if cv2.contourArea(contour) < 100:
            continue

         cv2.drawContours(output_img, contour, -1, color = (255, 255, 255), thickness = 1)

         rect = cv2.minAreaRect(contour)
         center, size, angle = rect
         center = [int(dim) for dim in center] # Convert to int so we can dr=

         # Draw rectangle and circle
        #  cv2.drawContours(output_img, np.int0(cv2.boxPoints(rect)), -1, color = (0, 0, 255), thickness = 2)
         cv2.circle(output_img, center = center, radius = 3, color = (0, 0, 255), thickness = -1)



      cv2.circle(output_img, center = (x_mid, y_mid) , radius = 3, color = (0, 0, 255), thickness = -1)

      print(x_mid, " ", y_mid)
      print(hsv_img[y_mid, x_mid,0], hsv_img[y_mid, x_mid,1], hsv_img[y_mid, x_mid,2], sep=' ')
      
      cv2.imshow("frame", output_img)
      cv2.imshow("binary", binary_img)
      cv2.waitKey(1)

      if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
   input_steam.release()
   cv2.destroyAllWindows()

main()

# import numpy as np
# import cv2 as cv
# cap = cv.VideoCapture(0)
# if not cap.isOpened():
#     print("Cannot open camera")
#     exit()
# while True:
#     # Capture frame-by-frame
#     ret, frame = cap.read()
#     # if frame is read correctly ret is True
#     if not ret:
#         print("Can't receive frame (stream end?). Exiting ...")
#         break
#     # Our operations on the frame come here
#     gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#     # Display the resulting frame
#     cv.imshow('frame', gray)
#     cv.waitKey(1)
# # When everything done, release the capture
# cap.release()
# cv.destroyAllWindows()

