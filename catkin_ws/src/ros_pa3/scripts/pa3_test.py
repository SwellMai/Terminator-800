#!/usr/bin/env python
import os
import rospy
import rosbag
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from tf.transformations import euler_from_quaternion
import math
from markers_example import display_cube_list, display_line_list, display_line
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def open_bag(path):
    # with open(path, mode='r') as f:
    #     for line in f.readlines():
    #         print line
    motion = []
    obser = []
    with rosbag.Bag(path, 'r') as bag:
        movement_len = bag.get_type_and_topic_info()[
            1]['Movements'].message_count
        observation_len = bag.get_type_and_topic_info()[
            1]['Observations'].message_count
        # ['Observations', 'Movements']
        topics = bag.get_type_and_topic_info()[1].keys()
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic == 'Movements':
                msg.rotation1 = (euler_from_quaternion(
                    (msg.rotation1.x, msg.rotation1.y, msg.rotation1.z, msg.rotation1.w)))
                msg.rotation2 = (euler_from_quaternion(
                    (msg.rotation2.x, msg.rotation2.y, msg.rotation2.z, msg.rotation2.w)))
                # msg.rotation1 = eul
                motion.append(msg)
            elif topic == 'Observations':
                msg.bearing = (euler_from_quaternion(
                    (msg.bearing.x, msg.bearing.y, msg.bearing.z, msg.bearing.w)))
                obser.append(msg)
            else:
                print "Unexcepted Topic Found!"
                break
        assert movement_len == len(
            motion) and observation_len == len(obser), 'ReadError'

    return motion, obser


def continuous_to_discrete(coordinates_with_meter):
    # have cancel the np.round
    return [np.round(i*5) for i in coordinates_with_meter]


def discrete_to_continuous(coordinates):
    return [i/5 for i in coordinates]


def traveraling(init_state, msg_motion):
    history_motion = []
    history_motion.append(init_state)
    # print history_motion, init_state
    for i in range(len(msg_motion)):
        now_angle = (history_motion[i][2]-1)*heading_angle_divide
        y_target = history_motion[i][1] + \
            math.sin(msg_motion[i].rotation1[2])*msg_motion[i].translation
        x_target = history_motion[i][0] + \
            math.cos(msg_motion[i].rotation1[2])*msg_motion[i].translation
        history_motion.append([x_target, y_target, int(
            (((now_angle+msg_motion[i].rotation1[2]+msg_motion[i].rotation2[2]) % (2*math.pi))/heading_angle_divide)+1)])
    return history_motion

# def discrete_bayes_filter(bel_x,d):
#     eta = 0
#     if isinstance(d,type_motion):

#     elif isinstance(d,type_observation):

#     else:
#         print "Type Error!"


if __name__ == "__main__":

    msg_motion, msg_observation = open_bag("./grid.bag")
    type_motion = type(msg_motion[0])
    type_observation = type(msg_observation[0])

    landmark = [[1.25, 5.25],
                [1.25, 3.25],
                [1.25, 1.25],
                [4.25, 1.25],
                [4.25, 3.25],
                [4.25, 5.25]]

    landmark_discreted = [continuous_to_discrete(i) for i in landmark]

    init_state = [12, 8, 3]
    heading_angle_divide = math.radians(90)
    history_motion = traveraling(init_state, msg_motion)


    rospy.init_node('rviz_pub')
    history_motion = [(i[0],i[1]) for i in history_motion]
    new = []
    for i in range(len(history_motion)):
        new.append(history_motion[i])
        new.append(history_motion[(i+1)%(len(history_motion))])
        
    new.pop()
    new.pop()
    pub_line_list = rospy.Publisher('line_list', Marker, queue_size=1)
    # print history_motion[:2]
    while not rospy.is_shutdown():
        display_line_list(new, pub_line_list)

    # discrete_bayes_filter(0,msg_motion[0])
