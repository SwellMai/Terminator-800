#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def callback(data):
    #rospy.loginfo("\n"+rospy.get_caller_id())
    #rospy.loginfo(data.pose.pose)
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.x
    pos_z = data.pose.pose.position.x

    ori_x = data.pose.pose.orientation.x
    ori_y = data.pose.pose.orientation.y
    ori_z = data.pose.pose.orientation.z
    ori_w = data.pose.pose.orientation.w
    return (pos_x,pos_y,pos_z),(ori_x,ori_y,ori_z,ori_w)
    #print data.pose.pose 


def detector():
    rospy.init_node('detector',anonymous=True)
    rospy.Subscriber('/base_pose_ground_truth',Odometry,callback)
    #rospy.spin()

if __name__ == "__main__":
    detector()