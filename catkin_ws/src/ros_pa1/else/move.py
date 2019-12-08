#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def move():
    rospy.init_node('robot_move', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # rate = rospy.Rate(10)

    vel_msg = Twist()

    vel_msg.linear.x = 4
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 4

    while not rospy.is_shutdown():
        vel_pub.publish(vel_msg)
        # rate.sleep()

if __name__ == "__main__":
    try:
        move()
    except rospy.ROSInterruptException:
        pass
