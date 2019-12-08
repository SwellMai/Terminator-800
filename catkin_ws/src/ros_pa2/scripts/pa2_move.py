#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from tf.transformations import euler_from_quaternion


class Move:

    def __init__(self, planned_route, startx, starty, goalx, goaly):

        self.planned_route = planned_route
        self.target_x = goalx
        self.target_y = goaly
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.ori_x = 0
        self.ori_y = 0
        self.ori_z = 0
        self.ori_w = 0
        self.eul_x = 0
        self.eul_y = 0
        self.eul_z = 0
        self.start_x = startx
        self.start_y = starty
        self.startpoint_recorded = False
        self.ranges = []
        self.L = 0
        self.L_max = 0
        self.L_min = 0
        self.FL = 0
        self.FL_min = 0
        self.FL_max = 0
        self.F = 0
        self.F_min = 0
        self.F_max = 0
        self.FR = 0
        self.FR_min = 0
        self.FR_max = 0
        self.R = 0
        self.R_min = 0
        self.R_max = 0
        self.walked = 0
        self.run = True
        self.can_GOALSEEK = True
        self.can_WALLFOLLOW = False
        self.WALLFOLLOW_turned = False
        self.final_change = True
        # self.ismove = True
        self.point_judge_thre = 0.01
        self.collision_thre = 1

        

    def callback(self, odom, laser):
        self.pos_x = odom.pose.pose.position.x
        self.pos_y = odom.pose.pose.position.y
        self.pos_z = odom.pose.pose.position.z
        self.ori_x = odom.pose.pose.orientation.x
        self.ori_y = odom.pose.pose.orientation.y
        self.ori_z = odom.pose.pose.orientation.z
        self.ori_w = odom.pose.pose.orientation.w
        eul = euler_from_quaternion(
            (self.ori_x, self.ori_y, self.ori_z, self.ori_w))
        self.eul_x = eul[0]
        self.eul_y = eul[1]
        self.eul_z = eul[2]
        self.ranges = laser.ranges
        self.R = np.mean(self.ranges[:72])
        self.R_min = np.min(self.ranges[:72])
        self.R_max = np.max(self.ranges[:72])
        self.FR = np.mean(self.ranges[72:144])
        self.FR_min = np.min(self.ranges[72:144])
        self.FR_max = np.max(self.ranges[72:144])
        self.F = np.mean(self.ranges[144:216])
        self.F_min = np.min(self.ranges[144:216])
        self.F_max = np.max(self.ranges[144:216])
        self.FL = np.mean(self.ranges[216:288])
        self.FL_min = np.min(self.ranges[216:288])
        self.FL_max = np.max(self.ranges[216:288])
        self.L = np.mean(self.ranges[288:])
        self.L_min = np.min(self.ranges[288:])
        self.L_max = np.max(self.ranges[288:])
        if self.startpoint_recorded == False:
            self.start_x = self.pos_x
            self.start_y = self.pos_y
            self.startpoint_recorded = True
        # print self.pos_x, self.ranges[:6]
        # print self.target_x

    def collect_data(self):
        rate = rospy.Rate(100)
        laser_sub = message_filters.Subscriber('/base_scan', LaserScan)
        odom_sub = message_filters.Subscriber(
            '/base_pose_ground_truth', Odometry)
        ts = message_filters.TimeSynchronizer(
            [odom_sub, laser_sub], 10)
        ts.registerCallback(self.callback)
        rate.sleep()

    def log_print(self):
        print "------------------------------------------------------------"
        print "position:("+str(self.pos_x) + ","+str(self.pos_y) + \
            ") target:(" + str(self.target_x)+"," + str(self.target_y)+")"
        print "L:"+str(self.L)+"\nFL:"+str(self.FL)
        print "F:" + str(self.F)+"\nFR:"+str(self.FR)+"\nR:"+str(self.R)
        print "\nL_min:"+str(self.L_min) + "L_max:"+str(self.L_max)
        print "F_min:"+str(self.F_min) + "L_max:"+str(self.F_max)
        print "FR_min:"+str(self.FR_min) + "FR_max:"+str(self.FR_max)
        print "eul_z:"+str(self.eul_z)
        print "distance:"+str(self.distance_to_line(self.pos_x, self.pos_y))
        print 'walked:'+str(self.walked)
        print "can_GOALSEEK:" + \
            str(self.can_GOALSEEK) + "  can_WALLFOLLOW:" + \
            str(self.can_WALLFOLLOW)
        print "============================================================"

    def move(self, angle, speed):
        # rospy.init_node('robot_move', anonymous=True)
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # rate = rospy.Rate(10)
        vel_msg = Twist()

        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angle

        vel_pub.publish(vel_msg)

    def distance_to_line(self, x, y):
        up = np.abs((self.target_y-self.start_y)*x-(self.target_x-self.start_x)
                    * y+(self.target_x*self.start_y)-(self.target_y*self.start_x))
        low = np.sqrt(np.power(self.target_y-self.start_y, 2) +
                      np.power(self.target_x-self.start_x, 2))
        return (up/low)

    def distance_to_point(self, node_1, node_2):
        return np.sqrt((node_1[0]-node_2[0])**2+(node_1[1]-node_2[1])**2)

    def goal_seek(self, target_node):
        rospy.loginfo(
            "move from" + str([self.pos_x, self.pos_y])+" to " + str(target_node))
        target_x = target_node[0]
        target_y = target_node[1]
        while self.distance_to_point([self.pos_x, self.pos_y], target_node) > self.point_judge_thre:
            self.collect_data()
            # rospy.loginfo(str(self.distance_to_point([self.pos_x, self.pos_y], target_node)))
            xie = np.sqrt(abs(target_y-self.pos_y)**2+abs(target_x-self.pos_x)**2)
            ling = abs(target_x-self.pos_x)
            abs_ang = np.arccos(ling / xie) if target_y>self.pos_y else (np.arccos(ling / xie)+np.pi)% 2*np.pi
            adj_eul_z = self.eul_z if self.eul_z > 0 else self.eul_z+2*np.pi #
            positive_clock = False if adj_eul_z<= abs_ang<= (adj_eul_z+np.pi )%(2*np.pi) else True
            # rospy.loginfo("\nmove from" + str([self.pos_x, self.pos_y])+" to " + str(target_node))
            # rospy.loginfo(str(adj_eul_z)+", "+str(abs_ang))
            if np.abs(adj_eul_z - abs_ang) > 0.15:  # change angle to the target
                if positive_clock:
                    self.move(-0.15, 0)
                    # rospy.loginfo("t")
                else:
                    self.move(0.15, 0)
                    # rospy.loginfo("f")
            else:
                if adj_eul_z - abs_ang > 0.1:
                    self.move(-0.5, 0.35)
                elif adj_eul_z - abs_ang < 0.1:
                    self.move(0.05, 0.35)

    def route_follow(self):
        #test
        # while not rospy.is_shutdown():
        #     self.collect_data()
        #     print self.pos_x,self.eul_x,self.eul_y,self.eul_z
        print self.planned_route
        for i in self.planned_route[1:-1]:
            self.goal_seek(i)


if __name__ == "__main__":
    pass
