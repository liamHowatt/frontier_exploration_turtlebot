#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from std_msgs.msg import String

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
    self.location = rospy.Publisher('/location', String, queue_size=100)

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
    thresh = 125
    img_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

    #cv2.imshow("B&W Image", gray)
    #cv2.imshow("B&W Image /w threshold", img_bw)

    qr_result = decode(img_bw)

    print (qr_result)
    
    #qr_data = qr_result[0].data
    #print qr_data

    #(x, y, w, h) = qr_result[0].rect
    #print qr_result[0].rect
    if qr_result:

        #cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)
        (p1, p2, p3, p4) = qr_result[0].polygon
        #print p1.x, p2.y, p3.x, p4.y

        points = np.array([[p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y], [p4.x, p4.y]])
        cv2.polylines(resized_image, np.int32([points]), True, (0,255,0),4)
        xpos = (p1.x+p2.x+p3.x+p4.x)/4.0
        ypos = (p1.y+p2.y+p3.y+p4.y)/4.0
        print 'Position:', xpos, ypos, xpos-320, 480-ypos
    
        dist = math.sqrt(math.pow((xpos-320),2) + math.pow((480-ypos),2))
        angle = math.degrees(math.atan(float(xpos-320)/float(480-ypos)))

        print 'Distance:', dist, '@', angle, 'degrees'
    
        # location = rospy.Publisher('location', String, queue_size=100)
        # rate = rospy.Rate(10)
        location_str = str(dist)+", "+str(angle)
        rospy.loginfo(location_str)
        self.location.publish(location_str)

    #text = "{}".format(qr_data)
    #cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.imshow("Camera output", resized_image)
    # cv2.imshow("Camera output", img_bw)

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
