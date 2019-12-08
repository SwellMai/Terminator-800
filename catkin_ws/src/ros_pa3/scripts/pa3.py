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
from scipy.stats import norm
from tqdm import tqdm


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
    return [(i*5) for i in coordinates_with_meter]


def discrete_to_continuous(coordinates):
    return [i/5 for i in coordinates]


def trajtory_track(init_state, msg_motion):
    # init_state usually be continuous.
    history_motion = []
    history_motion.append(init_state)
    for i in range(len(msg_motion)):
        x = history_motion[i][0] + \
            math.cos(msg_motion[i].rotation1[2] +
                     history_motion[i][2])*msg_motion[i].translation
        y = history_motion[i][1] + \
            math.sin(msg_motion[i].rotation1[2] +
                     history_motion[i][2])*msg_motion[i].translation
        theta = history_motion[i][2] + \
            msg_motion[i].rotation1[2]+msg_motion[i].rotation2[2]
        history_motion.append([x, y, theta])
    return history_motion


def motion_model(point_predict, point_now, msg_motion):
    # point_predict is a dictionary
    rot1 = msg_motion.rotation1[2]
    rot2 = msg_motion.rotation2[2]
    trans = msg_motion.translation
    delta_rot1 = np.arctan2((point_predict['y']-point_now['y']),
                            (point_predict['x']-point_now['x'])) - (point_now['theta'])
    delta_trans = np.sqrt((point_predict['x']-point_now['x']) **
                          2+(point_predict['y']-point_now['y'])**2)
    delta_rot2 = (point_predict['theta']) - \
        (point_now['theta']) - delta_rot1
    p1 = norm.pdf(delta_rot1, rot1, 0.1)
    p2 = norm.pdf(delta_trans, trans, 0.1)
    p3 = norm.pdf(delta_rot2, rot2, 0.1)
    return p1 * p2 * p3

    

def sensor_model(point_predict, msg_observation):
    tag_num = msg_observation.tagNum
    pre_rang = np.sqrt((point_predict['x']-landmark[tag_num][0])**2+(
        point_predict['y']-landmark[tag_num][1])**2)
    pre_bearing = np.arctan2(
        (point_predict['y']-landmark[tag_num][1]), (point_predict['x']-landmark[tag_num][0]))
    p1 = norm.pdf(pre_rang, msg_observation.range, 0.1)
    # print p1
    p2 = norm.pdf(pre_bearing, msg_observation.bearing[2], 0.1)
    # print p2
    # assert p2==2000
    return p1*p2


def discrete_bayes_filter(bel_x, msg):
    # receive the data of u or z in each ste
    if isinstance(msg, type_motion):  # u step
        bel_x_prime = bel_x
        for i in range(len(bel_x)):
            for j in range(len(bel_x[0])):
                for k in range(len(bel_x[0, 0])):
                    if bel_x[i,j,k] > 0.01:
                        bel_x_prime_sum = 0
                        point_now = {}
                        point_now['x'] = (i+1)/5
                        point_now['y'] = (j+1)/5
                        point_now['theta'] = (k+1)*math.radians(90)
                        for i_prime in range(len(bel_x_prime)):
                            for j_prime in range(len(bel_x_prime[0])):
                                for k_prime in range(len(bel_x_prime[0, 0])):
                                    point_predict = {}
                                    point_predict['x'] = (i_prime+1)/5
                                    point_predict['y'] = (j_prime+1)/5
                                    point_predict['theta'] = (k_prime+1)*math.radians(90)
                                    bel_x_prime_sum = bel_x_prime_sum +  motion_model(point_predict, point_now, msg)*bel_x[i, j, k]
                        bel_x_prime[i,j,k] = bel_x_prime_sum
        print np.max(bel_x_prime)
        return bel_x_prime

    elif isinstance(msg, type_observation):  # z step
        for i in range(len(bel_x)):
            for j in range(len(bel_x[0])):
                for k in range(len(bel_x[0, 0])):
                    point_predict = {}
                    point_predict['x'] = (i+1)/5
                    point_predict['y'] = (j+1)/5
                    point_predict['theta'] = (k+1)*math.radians(90)
                    bel_x[i,j,k] = bel_x[i,j,k] * \
                        sensor_model(point_predict, msg)
        # print np.max(bel_x)
        # assert sum(bel_x)<0
        bel_x = bel_x/sum(bel_x)
        # print bel_x
        return bel_x

    else:
        print "Type Error!"


def bayes_loop(bel_x, msg_motion, msg_observation):
    history = []
    bel_x_process = []
    bel_x_process.append(bel_x),
    d_route = []
    for i in tqdm(range(len(msg_motion))):
        bestchoise_in_step = {}        
        # u_step
        bel_x_prime_p = discrete_bayes_filter(bel_x_process[i], msg_motion[i])
        bestchoise_in_step["p"] = bel_x_prime_p
        # z_step
        bel_x_prime_u = discrete_bayes_filter(bel_x_prime_p, msg_observation[i])
        bestchoise_in_step["u"] = bel_x_prime_u

        bel_x_process.append(bel_x_prime_u)

        d_chosen_point = np.unravel_index(np.argmax(bel_x_prime_u, axis=None), bel_x_prime_u.shape)
        d_route.append(d_chosen_point)

        history.append(bestchoise_in_step)

    return history, d_route


def generate_show_line(history_motion):
    new = []
    for i in range(len(history_motion)):
        new.append(history_motion[i])
        new.append(history_motion[(i+1) % (len(history_motion))])
    new.pop()
    new.pop()
    return new

def write_to_txt(p,u):
    assert len(p)==len(u)
    with open("./estimation.txt","w") as f:
        for i in range(len(p)):
            f.write("Step {}\n".format(str(i)))
            f.write("P:({},{},{})\n".format(str(p[i][0]),str(p[i][1]),str(p[i][2])))
            f.write("U:({},{},{})\n".format(str(u[i][0]),str(u[i][1]),str(u[i][2])))
            f.write("\n")
        f.write("End of file")
    rospy.loginfo("Txt file exported")

def display(line_list,cube_list):
    rospy.init_node('rviz_display')
    line_list_adv = generate_show_line(line_list)
    pub_line_list = rospy.Publisher('line_list', Marker, queue_size=1)
    pub_cube_list = rospy.Publisher('cube_list', Marker, queue_size=1)
    
    while not rospy.is_shutdown():
        display_line_list(line_list_adv, pub_line_list)
        display_cube_list(landmark, pub_cube_list)



if __name__ == "__main__":

    msg_motion, msg_observation = open_bag("./grid.bag")
    
    # type_motion = type(msg_motion[0])
    # type_observation = type(msg_observation[0])

    # landmark = [[1.25, 5.25],
    #             [1.25, 3.25],
    #             [1.25, 1.25],
    #             [4.25, 1.25],
    #             [4.25, 3.25],
    #             [4.25, 5.25]]
    # landmark_discreted = [continuous_to_discrete(i) for i in landmark]

    # init_state_cont = [12.0/5, 28.0/5, (math.radians(200.52))]
    # init_state_dist = [12, 28, 3]

    # test = generate_show_line(trajtory_track(init_state_cont, msg_motion))

    # bel_x = np.zeros((35, 35, 4))+1e-6

    # bel_x[11, 27, 2] = 1-(35*35*4-1)*1e-6
    # print bel_x[11, 27, 2]
    
    # history, d_route = bayes_loop(bel_x, msg_motion, msg_observation)

    
    # print(history,d_route)
    # display(d_route,landmark_discreted)
    # write_to_txt([i['p']for i in d_route],[i['u']for i in d_route])
    print msg_motion[0],msg_observation[0]

