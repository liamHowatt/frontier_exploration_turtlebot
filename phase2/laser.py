#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String
import time

class Persistor:
    def __init__(self, persist_for):
        self.persist_for = persist_for
        self.last_seen = float("-inf")
        self.data = None
    def set(self, data):
        self.last_seen = time.time()
        self.data = data
    def get(self):
        if (time.time() - self.last_seen) <= self.persist_for:
            return self.data
        else:
            return None

class Scanterceptor:

    def __init__(self):
        self.scan_sub = rospy.Subscriber("/scanx", LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=100)

        self.cam_sub = rospy.Subscriber("/location", String, self.cam_callback)

        self.persistor = Persistor(1)

    def scan_callback(self, data):
        assert len(data.intensities) == 360
        assert len(data.ranges) == 360

        cam_data = self.persistor.get()

        if cam_data is not None:
            distance, angle = cam_data

            angle_at = 180 + angle
           
            field = {
                (ang + 180) % 360
                for ang
                in xrange(
                    int(angle_at - 7.5),
                    int(angle_at + (7.5 + 1)) + 1
                )
            }

            new_ranges = []
            new_intensities = []
            for i, (range, intensity) in enumerate(zip(data.ranges, data.intensities)):
                if i in field:
                    new_ranges.append(0.5)
                    new_intensities.append(7500.0)
                else:
                    new_ranges.append(range)
                    new_intensities.append(intensity)
            data.ranges = new_ranges
            data.intensities = new_intensities

        self.scan_pub.publish(data)

    def cam_callback(self, data):
        distance, angle, _ = data.data.split(",", 2)
        self.persistor.set( (float(distance), float(angle)) )

def main():
    rospy.init_node("scan_recv", anonymous=False)

    Scanterceptor()

    rospy.spin()


if __name__ == "__main__":
    main()
