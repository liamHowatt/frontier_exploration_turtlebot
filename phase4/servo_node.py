#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
import serial

def main():
    rospy.init_node("servo", anonymous=False)
    Servo()
    print "spinning"
    rospy.spin()


class Servo:

    def __init__(self):
        print "connecting to arduino"
        self.arduino = serial.Serial("/dev/ttyAMC0", 115200)
        time.sleep(2)
        print "connected. subscribing"
        self.cam_sub = rospy.Subscriber("/servo", String, self.callback)

    def callback(self, data):
        self.arduino.write(int(data.data).to_bytes(1, "little"))


if __name__ == "__main__":
    main()

