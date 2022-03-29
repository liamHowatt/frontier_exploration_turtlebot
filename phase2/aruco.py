#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from std_msgs.msg import String

X, Y = 0, 1

desired_aruco_dictionary = "DICT_5X5_50"

ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class camera_1:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)

        self.location = rospy.Publisher('location', String, queue_size=100)

    def callback(self,data):

        bridge = CvBridge()

        # try:
        #   cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #   rospy.logerr(e)
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        (rows,cols,channels) = cv_image.shape

        image = cv_image


        #resized_image = cv2.resize(image, (360, 640)) 
        resized_image = image 

        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        thresh = 40
        img_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

        #cv2.imshow("B&W Image", gray)
        #cv2.imshow("B&W Image /w threshold", img_bw)

        qr_result = decode(img_bw)

        #print (qr_result)

        #qr_data = qr_result[0].data
        #print qr_data

        #(x, y, w, h) = qr_result[0].rect
        #print qr_result[0].rect

        aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
        aruco_params = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(resized_image, aruco_dict, parameters=aruco_params)

        if len(corners) > 0:
            ids = ids.flatten()
            for (marker_corner, marker_id) in zip(corners, ids):
                corners = marker_corner.reshape((4,2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))
                
                # Draw the bounding box of the ArUco detection
                cv2.line(resized_image, top_left, top_right, (0, 255, 0), 2)
                cv2.line(resized_image, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(resized_image, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(resized_image, bottom_left, top_left, (0, 255, 0), 2)
                
                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(resized_image, (center_x, center_y), 4, (0, 0, 255), -1)
                
                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv2.putText(resized_image, str(marker_id), 
                    (top_left[0], top_left[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)



                xpos = (top_right[X]+top_left[X]+bottom_right[X]+bottom_left[X])/4.0
                ypos = (top_right[Y]+top_left[Y]+bottom_right[Y]+bottom_left[Y])/4.0
                # print points
                # print 'Position:', xpos, ypos, xpos-320, 480-ypos
            
                dist = math.sqrt(math.pow((xpos-320),2) + math.pow((480-ypos),2))
                angle = math.atan(float(xpos-320)/float(480-ypos))
                area = math.pow(((abs(top_right[Y]-bottom_right[Y])+abs(top_left[Y]-bottom_left[Y]))/2.0),2)
                area_calibration = 2000
                distance_calibration = 100.0
                real_dist = (((area_calibration/area)*distance_calibration)/math.cos(angle))
                print 'Distance:', real_dist, 'mm @', math.degrees(angle), 'degrees', area
            
                            
                location_str = str(real_dist)+", "+str(math.degrees(angle)) + ", " + str(marker_id) # <-- OUTPUT STRING FORMAT
                
                rospy.loginfo(location_str)
                self.location.publish(location_str)



        # if qr_result:

        #     #cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)
        #     (p1, p2, p3, p4) = qr_result[0].polygon
        #     #print p1.x, p2.y, p3.x, p4.y

        #     points = np.array([[p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y], [p4.x, p4.y]])
        #     cv2.polylines(resized_image, np.int32([points]), True, (0,255,0),4)
        #     xpos = (p1.x+p2.x+p3.x+p4.x)/4.0
        #     ypos = (p1.y+p2.y+p3.y+p4.y)/4.0
        #     print points
        #     print 'Position:', xpos, ypos, xpos-320, 480-ypos
        
        #     dist = math.sqrt(math.pow((xpos-320),2) + math.pow((480-ypos),2))
        #     angle = math.atan(float(xpos-320)/float(480-ypos))
        #     area = math.pow(((abs(p1.y-p2.y)+abs(p3.y-p4.y))/2.0),2)
        #     area_calibration = 2000
        #     distance_calibration = 100.0
        #     real_dist = (((area_calibration/area)*distance_calibration)/math.cos(angle))
        #     print 'Distance:', real_dist, 'mm @', math.degrees(angle), 'degrees', area
        
        #     location = rospy.Publisher('location', String, queue_size=100)
        #     rate = rospy.Rate(10)
        
        #     location_str = str(real_dist)+", "+str(math.degrees(angle)) # <-- OUTPUT STRING FORMAT
        
        #     rospy.loginfo(location_str)
        #     location.publish(location_str)

        #text = "{}".format(qr_data)
        #cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("Camera output", resized_image)

        cv2.waitKey(5)

    

def main():
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()
