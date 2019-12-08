#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


def callback(data):
    #rospy.loginfo(data.range)
    #print data.angle_min,data.angle_max
    #print data.range_min,data.range_max
    print len(data.ranges)

def sensor():
    rospy.init_node('sensor', anonymous=True)
    rospy.Subscriber('/base_scan', LaserScan, callback)
    rospy.spin()


if __name__ == "__main__":
    sensor()
