#!/usr/bin/env python
import numpy as np
import heapq
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


grid = np.array([
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
    [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1]])


class Node:
    def __init__(self, x, y, blocked):
        self.is_blocked = blocked
        self.x = x
        self.y = y
        self.parent = None
        self.f_score = 0
        self.g_score = 0
        self.h_score = 0


class A_star:
    def __init__(self, grid_, src_, dst_):
        '''
        grid: 2D numpy array represent the map, where 0 is not blocked, 1 is blocked
        src: numpy array, [x, y] where x is the axis coordinate, y is y axis coordinate
        dst: numpy array, [x, y] similar to src
        '''
        self.hp = []  # the heap of nodes represents the nodes to be explored
        self.world_height, self.world_width = len(grid), len(grid[0])
        self.closed = set()  # the nodes that have already been explored
        self.src = Node(src_[0], src_[1], 0)
        self.dst = Node(dst_[0], dst_[1], 0)
        self.grid = grid_

    def find_path(self):
        heapq.heappush(self.hp, (self.src.f_score, self.src))
        while len(self.hp) > 0:
            _, node = heapq.heappop(self.hp)
            if node.x == self.dst.x and node.y == self.dst.y:
                path = []
                while node is not None:
                    path.append((node.x, node.y))
                    node = node.parent
                path.reverse()
                return path
            dirs = [[0, -1], [0, 1], [1, 0], [-1, 0],
                    [1, 1], [1, -1], [-1, 1], [-1, -1]]
            for d in dirs:
                next_x, next_y = node.x + d[0], node.y + d[1]
                if next_x >= 0 and next_x < self.world_width and next_y >= 0 and next_y < self.world_height and self.grid[next_y][next_x] == 0 and (next_x, next_y) not in self.closed:
                    new_node = Node(next_x, next_y, 0)
                    in_heap = False
                    for _, no in self.hp:
                        if new_node.x == no.x and new_node.y == no.y:  # new_node in hp
                            in_heap = True
                            if no.g_score > node.g_score+1:
                                no.g_score = node.g_score + 1
                                no.f_score = no.g_score + \
                                    np.sqrt((no.x - self.dst.x)**2 +
                                            (no.y - self.dst.y)**2)
                                no.parent = node
                    if not in_heap:
                        new_node.parent = node
                        new_node.g_score = node.g_score + 1
                        new_node.f_score = new_node.g_score + \
                            np.sqrt((new_node.x - self.dst.x)**2 +
                                    (new_node.y - self.dst.y)**2)
                        heapq.heappush(self.hp, (new_node.f_score, new_node))


def callback(odom_data, args):
    if not args['Done']:
        path = args['path']
        cur_idx = args['cur_idx']
        pub = args['pub']
        pos = odom_data.pose.pose.position  # the position of robot (x, y, z)
        # the orientation of robot, (x, y, z, w)
        ori = odom_data.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[
            2]  # the yaw angle in radian
        x, y = pos.x, pos.y  # the x, y position coordinate
        goalx, goaly = path[cur_idx][0] - 9 + 0.8, 10 - path[cur_idx][1] - 0.8
        # the orientation we want
        goal_theta = math.atan2(goaly - y, goalx - x)
        # the difference between the current orientation and goal orientation
        ori_diff = goal_theta - yaw
        cmd = Twist()
        # need rotation now
        if args['rotation']:
            if math.fabs(ori_diff) > math.radians(5):  # if difference larger than 5 degree
                if ori_diff < 0:
                    cmd.angular.z = -0.75
                else:
                    cmd.angular.z = 0.75
                pub.publish(cmd)
            else:
                args['rotation'] = False
        else:
            dist = math.sqrt((goalx - x)**2 + (goaly - y)**2)
            if dist > 0.3:
                cmd.linear.x = 0.75
                pub.publish(cmd)
            else:
                args['rotation'] = True
                if cur_idx == len(path) - 1:
                    args['Done'] = True
                else:
                    args['cur_idx'] += 1


if __name__ == '__main__':
    rospy.init_node("robot", anonymous=False)
    src = (1, 12)
    dst = (13, 1)
    args = {}
    args['Done'] = False
    args["rotation"] = True
    args['cur_idx'] = 0
    args['pub'] = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    a_star = A_star(grid, src, dst)
    args['path'] = a_star.find_path()
    print args['path']
    # robot_pos_pub = rospy.Subscriber(
    #     "/base_pose_ground_truth", Odometry, callback, args)
    # rospy.spin()
