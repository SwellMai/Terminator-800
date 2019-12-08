#!/usr/bin/env python
import os, math, sys, ipdb
import numpy as np
import rospy, rosbag, message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from markers_example import display_cube_list, display_line_list
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion


def read_bag(path):
    '''Read motion/observation data from .bag file.
    '''
    motions = []
    observations = []
    with rosbag.Bag(path, 'r') as bag:
        topics = bag.get_type_and_topic_info()[1].keys()

        for topic, msg, t in bag.read_messages(topics=topics):
            if topic == 'Movements':
                # translate quaternion to euler angles
                msg.rotation1 = (euler_from_quaternion((msg.rotation1.x,
                        msg.rotation1.y, msg.rotation1.z, msg.rotation1.w)))
                msg.rotation2 = (euler_from_quaternion((msg.rotation2.x, 
                        msg.rotation2.y, msg.rotation2.z, msg.rotation2.w)))
                # keep translation unchanged
                motions.append(msg)
            elif topic == 'Observations':
                # translate quaternion to euler angles
                msg.bearing = (euler_from_quaternion((msg.bearing.x, 
                        msg.bearing.y, msg.bearing.z, msg.bearing.w)))
                observations.append(msg)
            else:
                print "Unsupported Topic!"
                sys.exit(0)          
    return motions, observations


def parse_motion(motion_euler):
    # Two of the three euler angles are 0. The last one is what we need.
    rot1 = motion_euler.rotation1[-1]
    rot2 = motion_euler.rotation2[-1]
    trans = motion_euler.translation

    motion = (rot1, rot2, trans)
    return motion


def continuous_to_discrete(coordinates_with_meter, threshold=5):
    return [np.round(i*threshold) for i in coordinates_with_meter]


def discrete_to_continuous(coordinates, threshold=5):
    return [i/threshold for i in coordinates]


def get_curr_pose(prev_pose, motion):
    '''Compute current robot pose from given previous pose and motion data.
    pose: [x, y, theta]
    motion: (rotation1, rotation2, translation)
    '''
    # Two of the three euler angles are 0. The last one is what we need.
    rot1, rot2, trans = motion

    x = prev_pose[0] + trans * math.cos(rot1)
    y = prev_pose[1] + trans * math.cos(rot2)
    theta = prev_pose[2] + rot1 + rot2

    curr_pose = [x, y, theta]
    return curr_pose


def get_pose_hist(init_state, motions):
    '''Get poses of all time steps.
    '''
    prev_pose = init_state
    pose_hist = []
    pose_hist.append(prev_pose)
    
    for motion_euler in motions:
        motion = parse_motion(motion_euler)
        curr_pose = get_curr_pose(prev_pose, motion)
        pose_hist.append(curr_pose)
        prev_pose = curr_pose[:]
        
    return pose_hist


if __name__ == "__main__":
    # landmark config
    landmarks = [[1.25, 5.25],
                [1.25, 3.25],
                [1.25, 1.25],
                [4.25, 1.25],
                [4.25, 3.25],
                [4.25, 5.25]]
    landmark_discret = [continuous_to_discrete(i) for i in landmarks]

    # prepare data
    init_state = [12, 28, 3.5]
    motions, observations = read_bag("grid.bag")
    pose_hist = get_pose_hist(init_state, motions)

    # show moving trajectory
    line_points = [(pose[0], pose[1]) for pose in pose_hist]
    rospy.init_node('rviz_pub')
    pub_line_list = rospy.Publisher('line_list', Marker, queue_size=1)
    while not rospy.is_shutdown():
        display_line_list(line_points, pub_line_list)