#!/usr/bin/env python
import rospy
import os
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from tf.transformations import euler_from_quaternion
import math

class node():
        def __init__(self, x, y, g_cost=0.0, h_cost=0.0, f_cost=0.0):
            self.x = x
            self.y = y
            self.g_cost = g_cost
            self.h_cost = h_cost
            self.f_cost = f_cost
            self.parent = None

        def __str__(self):
            return "x:"+str(self.x)+", y:"+str(self.y)+", g_cost"+str(self.g_cost)+", f_cost"+str(self.f_cost)

class A_Star():

    
    def __init__(self, startx, starty, goalx, goaly, grid_map):
        self.open = []
        self.closed = []
        self.grid_map = grid_map
        self.step = grid_map.astype(np.str)
        self.start_node = node(startx, starty)
        self.end_node = None
        startx_trans, starty_trans = self.transfer_to_array_coordinate(
            self.start_node)
        assert self.grid_map[startx_trans, starty_trans] == 0
        self.start_node = node(startx_trans, starty_trans)

        self.goal_node = node(goalx, goaly)
        goalx_trans, goaly_trans = self.transfer_to_array_coordinate(
            self.goal_node)
        # print startx_trans, starty_trans, goalx_trans, goaly_trans
        self.goal_node = node(goalx_trans, goaly_trans)

        self.open.append(self.start_node)
        self.dis_epsilon = 100
        self.motion = [[1, 0, 1],
                       [0, 1, 1],
                       [-1, 0, 1],
                       [0, -1, 1],
                       [-1, -1, np.sqrt(2)],
                       [-1, 1, np.sqrt(2)],
                       [1, -1, np.sqrt(2)],
                       [1, 1, np.sqrt(2)]]

    def euclidean_distance(self, node_1, node_2):
        return np.sqrt((node_1.x-node_2.x)**2 + (node_1.y-node_2.y)**2)

    # def g_cost(self, node):
    #     g_cost = self.euclidean_distance(self.start_node, node)
    #     return int(g_cost*10)

    def h_cost(self, node):
        h_cost = self.euclidean_distance(self.goal_node, node)
        return h_cost

    def f_cost(self, node):
        g_cost = node.g_cost
        h_cost = self.h_cost(node)
        return g_cost+self.dis_epsilon*h_cost

    def transfer_to_array_coordinate(self, node):
        i = int(self.grid_map.shape[0]/2)-node.y if self.grid_map.shape[0] % 2 != 0 else int(
            self.grid_map.shape[0]/2)-node.y-1
        j = int(self.grid_map.shape[1]/2)+node.x if self.grid_map.shape[1] % 2 != 0 else int(
            self.grid_map.shape[1]/2)+node.x-1
        return np.ceil(i), np.ceil(j)

    def transfer_to_map_coordinates(self, node):
        y = (self.grid_map.shape[0]/2)-node.x if self.grid_map.shape[0] % 2 != 0 else (
            self.grid_map.shape[0]/2)-1-node.x
        # print (self.grid_map.shape[0]/2), node.x
        x = -(self.grid_map.shape[1]/2)+node.y if self.grid_map.shape[1] % 2 != 0 else -(
            self.grid_map.shape[1]/2)+node.y+1
        # print (self.grid_map.shape[1]/2), node.y
        return [x, y]

    def traversable(self, i, j):
        if 0 <= i < self.grid_map.shape[0] and 0 <= j < self.grid_map.shape[1] and self.grid_map[i, j] == 0:
            if (0 <= i-1 < self.grid_map.shape[0] and 0 <= j-1 < self.grid_map.shape[1] and self.grid_map[i-1, j] == 1 and self.grid_map[i, j-1] == 1):
                return False
            elif (0 <= i-1 < self.grid_map.shape[0] and 0 <= j+1 < self.grid_map.shape[1] and self.grid_map[i-1, j] == 1 and self.grid_map[i, j+1] == 1):
                return False
            elif (0 <= i+1 < self.grid_map.shape[0] and 0 <= j-1 < self.grid_map.shape[1] and self.grid_map[i+1, j] == 1 and self.grid_map[i, j-1] == 1):
                return False
            elif (0 <= i+1 < self.grid_map.shape[0] and 0 <= j+1 < self.grid_map.shape[1] and self.grid_map[i+1, j] == 1 and self.grid_map[i, j+1] == 1):
                return False
            else:
                return True
        else:
            return False

    def inside(self, node, node_list):
        x, y = node.x, node.y
        coordinate_list = [[node.x, node.y] for node in node_list]
        # print coordinate_list
        if [x, y] in coordinate_list:
            return True
        else:
            return False

    def route_plan(self):
        while True:
            self.open.sort(cmp=None, key=lambda x: x.f_cost, reverse=False)
            current = self.open[0]

            if current.x == self.goal_node.x and current.y == self.goal_node.y:
                # rospy.loginfo("A_Star alg finished")
                # print "A_Star alg finished"
                self.end_node = current
                return self.output_route()

            self.closed.append(current)
            self.open.pop(0)

            for _, j in enumerate(self.motion):
                neighbour = node(
                    x=current.x+j[0], y=current.y+j[1], g_cost=current.g_cost+j[2])

                if not self.traversable(neighbour.x, neighbour.y) or self.inside(neighbour, self.closed):
                    continue

                if neighbour.g_cost < current.g_cost or not self.inside(neighbour, self.open):
                    neighbour.f_cost = self.f_cost(neighbour)
                    neighbour.parent = current
                    if not self.inside(neighbour, self.open):
                        self.open.append(neighbour)

    def print_result_grid(self, nodelist):
        cunt = -1
        self.step[self.start_node.x, self.start_node.y] = "$"
        self.step[self.goal_node.x, self.goal_node.y] = "%"
        for i in nodelist:
            self.step[i[0], i[1]] = "*"
            self.step[i[0], i[1]] = (cunt)
            cunt = cunt - 1
        print self.step

    def output_route(self):
        last_point = self.end_node
        result = []
        # cunt = 0
        while last_point is not None:
            # print [last_point.x,last_point.y]
            result.append(self.transfer_to_map_coordinates(node(last_point.x,last_point.y)))
            last_point = last_point.parent
            # print cunt
            # cunt = cunt +1
        result.reverse()
        return result

class Move:

    def __init__(self, planned_route, startx, starty, goalx, goaly,grid_map):
        self.grid_map = grid_map
        self.planned_route = planned_route
        self.goalx = goalx
        self.goaly = goaly
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
        self.now_node = 0
        self.rotation = True
        self.adjust = False
        self.point_judge_thre = 0.2
        self.collision_thre = 0.7
        self.can_run = True
        self.can_routefollow = True
        self.route = self.planned_route[1:-1]
        self.route.append([self.goalx,self.goaly])
        self.adj_angle = 0
        self.can_set_adj_angle = True
        self.can_adj_rotation = True
        self.adj_end_point_cunt = 25

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
        self.ranges_min_val = min(self.ranges)
        self.ranges_min_idx = self.ranges.index(self.ranges_min_val)
        if self.startpoint_recorded == False:
            self.start_x = self.pos_x
            self.start_y = self.pos_y
            self.startpoint_recorded = True
        # print self.pos_x, self.ranges[:6]
        # print self.target_x

    def collect_data(self):
        rate = rospy.Rate(20)
        laser_sub = message_filters.Subscriber('/base_scan', LaserScan)
        odom_sub = message_filters.Subscriber(
            '/base_pose_ground_truth', Odometry)
        ts = message_filters.TimeSynchronizer(
            [odom_sub, laser_sub], 100)
        ts.registerCallback(self.callback)
        rate.sleep()

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

    def distance_to_point(self, node_1, node_2):
        return np.sqrt((node_1[0]-node_2[0])**2+(node_1[1]-node_2[1])**2)

    def go(self):
        self.collect_data()
        min_val = self.ranges_min_val
        min_idx = self.ranges_min_idx
        if min_val<self.collision_thre:
            rospy.loginfo(str(min_val)+","+str(min_idx))
            self.obstacle_avoidance(min_idx,min_val)
        elif self.can_routefollow:
            self.route_follow()

    def obstacle_avoidance(self,min_index,min_value):
        rospy.loginfo("0")
        self.can_routefollow = False
        if self.can_set_adj_angle:
            rospy.loginfo("-1")
            if 0<=min_index<=90:
                self.can_set_adj_angle=False
                self.adj_angle = math.radians(25)+self.eul_z
                rospy.loginfo("-1.1")
            elif 90<min_index<=180:
                self.can_set_adj_angle=False
                self.adj_angle = math.radians(50)+self.eul_z
                rospy.loginfo("-1.2")
            elif 180<min_index<=270:
                self.can_set_adj_angle=False
                self.adj_angle = -math.radians(25)+self.eul_z
                rospy.loginfo("-1.3")
            elif 270 < min_index<=380:
                self.can_set_adj_angle=False
                self.adj_angle = -math.radians(50)+self.eul_z
                rospy.loginfo("-1.4")
        now_ori = self.eul_z
        target_ori = self.adj_angle
        ori_diff = target_ori-now_ori
        positive_clock = False if  ori_diff >=0 else True
        if self.can_adj_rotation:
            if math.fabs(ori_diff) > math.radians(5):
                if positive_clock:
                    self.move(-0.45, 0)
                    rospy.loginfo("1")
                else:
                    self.move(0.45, 0)
                    rospy.loginfo("2")
            else:
                self.can_adj_rotation = False
                rospy.loginfo("3")
        else:
            rospy.loginfo("3.5")
            rospy.loginfo(str(min_value))
            if min_value<self.collision_thre+1:
                    rospy.loginfo("4")
                    self.move(0,0.45)
            else: 
                rospy.loginfo("5")
                self.can_routefollow = True
                self.can_adj_rotation = True
                self.can_set_adj_angle = True
                a_star = A_Star(self.pos_x,self.pos_y,self.goalx,self.goaly,self.grid_map)
                self.planned_route = a_star.route_plan()
                
                # rospy.signal_shutdown("trans")
        

    def route_follow(self):
        if self.can_run:
            target_node = self.route[self.now_node]
            # rospy.loginfo(
            #     "move from" + str([self.pos_x, self.pos_y])+" to " + str(target_node))
            target_x = target_node[0]
            target_y = target_node[1]
            # self.collect_data()
            now_ori = self.eul_z
            target_ori = math.atan2(target_y - self.pos_y, target_x - self.pos_x)
            ori_diff = target_ori-now_ori
            # rospy.loginfo(str(now_ori)+","+str(target_ori)+","+str(ori_diff))
            positive_clock = False if  ori_diff >=0 else True
            if self.rotation:
                if math.fabs(ori_diff) > math.radians(4):
                    if positive_clock:
                        self.move(-0.45, 0)
                    else:
                        self.move(0.45, 0)
                else:
                    self.rotation = False
            else:
                if self.distance_to_point([self.pos_x, self.pos_y], target_node) > self.point_judge_thre:
                    self.move(0,0.55)
                    # rospy.loginfo()
                    # rospy.loginfo(str(minn))
                else:
                    self.rotation = True
                    self.now_node = self.now_node+1
                    if self.now_node > len(self.route)-1:
                        # rospy.loginfo(str(self.now_node)+","+str(len(route)-1)+","+str(len(route)))
                        self.can_run = False
        else:
            rospy.loginfo("Done!")
            rospy.signal_shutdown("Done!")






class PA2():
    def __init__(self,gird):
        rospy.init_node('pa2_main', anonymous=True)
        rospy.set_param("/goalx", 4.5)
        rospy.set_param("/goaly", 9.0)
        rospy.set_param("/goalx", 0.0)
        rospy.set_param("/goaly", 5.0)
        self.map_grid= gird
        self.startx = -8.0
        self.starty = -2.0
        self.goalx = rospy.get_param("/goalx")
        self.goaly = rospy.get_param("/goaly")
        self.planned_route = None
        self.a_star = A_Star(self.startx, self.starty, self.goalx,
                        self.goaly, self.map_grid)
        self.planned_route = self.a_star.route_plan()

        rospy.loginfo("route planned!")
        self.move = Move(self.planned_route, self.startx,
                    self.starty, self.goalx, self.goaly,self.map_grid)
        

    def main(self):    
        # print self.planned_route
        while not rospy.is_shutdown():
            self.move.go()
            



if __name__ == "__main__":
    # address need be changed when run in terminal
    print os.getcwd()
    grid = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])
    pa2 = PA2(grid)
    pa2.main()
    

