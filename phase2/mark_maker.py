#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from itertools import count


def main():
    rospy.init_node("mark_maker", anonymous=False)
    MarkMaker()
    rospy.spin()


def create_marker(id, x, y):
    marker = Marker()

    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.id = id

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    return marker


class MarkMaker:

    def __init__(self):
        self.mark_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
        self.cam_sub = rospy.Subscriber("/location", String, self.cam_callback)

        # self.id_gen = count()
        # self.unique_markers = []

    def cam_callback(self, data):
        # rostopic pub /location std_msgs/String '0.0,0.0' -r 2
        distance, angle, marker_id = data.data.split(",")
        distance = float(distance)
        angle = float(angle)
        marker_id = int(marker_id)

        marker = create_marker(marker_id, distance / 1000.0, 0.0)
        self.mark_pub.publish(marker)


if __name__ == "__main__":
    main()


"""
>>> Marker()
header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
ns: ''
id: 0
type: 0
action: 0
pose: 
  position: 
    x: 0.0
    y: 0.0
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
scale: 
  x: 0.0
  y: 0.0
  z: 0.0
color: 
  r: 0.0
  g: 0.0
  b: 0.0
  a: 0.0
lifetime: 
  secs: 0
  nsecs:         0
frame_locked: False
points: []
colors: []
text: ''
mesh_resource: ''
mesh_use_embedded_materials: False
"""