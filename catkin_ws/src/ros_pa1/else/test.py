#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def callback(laser, odom):
    print laser.ranges[:11]
    print odom.pose.pose.position.x
    # print target.pose.position.x


if __name__ == "__main__":
    rospy.init_node("listener_combined",anonymous=True)
    laser_sub = message_filters.Subscriber('/base_scan', LaserScan)
    odom_sub = message_filters.Subscriber('/base_pose_ground_truth',
                                        Odometry)
    target_sub = rospy.Subscriber('/homing_signal', PoseStamped)
    ts = message_filters.TimeSynchronizer([laser_sub,odom_sub],10)
    ts.registerCallback(callback)
    rospy.spin()
