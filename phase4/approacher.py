#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
import os
import json

GRAB_THRESH = 200.0
SEE_THRESH = 15.0

def main():
    rospy.init_node("approacher", anonymous=False)
    Approacher()
    print "spinning"
    rospy.spin()


def make_twist(vel, turn):
    twist = Twist()

    # max vel 0.26
    twist.linear = Vector3(vel, 0.0, 0.0) # x, y, z

    # max turn 1.82
    # right turn is negative
    # left turn is positive
    twist.angular = Vector3(0.0, 0.0, turn) # x, y, z

    return twist


def constrain(low, x, hi):
    return max(low, min(x, hi))


class Approacher:

    def __init__(self):
        self.aruco_sub = rospy.Subscriber("location", String, self.callback)
        self.wheels = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.explore_alive = True
        self.job_done = False

    def callback(self, data):
        aruco_data = json.loads(data.data)
        print "pseudo_size = {}, pseudo_angle = {}".format(aruco_data["pseudo_size"], aruco_data["pseudo_angle"])

        if self.job_done:
            return
        
        if aruco_data["pseudo_size"] < SEE_THRESH:
            return
        
        if self.explore_alive:
            print "killing explore"
            self.explore_alive = False
            # os.system("rosnode kill /move_base")
            os.system("rosnode kill /explore")

        if aruco_data["pseudo_size"] > GRAB_THRESH:
            print "at marker. going home!"
            self.wheels.publish(make_twist(0.0, 0.0))
            os.system("rostopic pub /move_base_simple/goal $(rostopic type /move_base_simple/goal) '{header: {frame_id: \"map\"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'")
            self.job_done = True
        else:

            twist = make_twist(0.065, -constrain(-1.81, aruco_data["pseudo_angle"] / 320.0 * 1.82, 1.81))
            print twist.linear.x, twist.angular.z

            self.wheels.publish(twist)



if __name__ == "__main__":
    main()

