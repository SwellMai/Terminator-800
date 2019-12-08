import numpy as np
import pa2_readTxt as read
import os
import rospy


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

    def __init__(self, startx, starty, goalx, goaly):
        self.open = []
        self.closed = []
        self.grid_map = np.array([
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
        self.step = self.grid_map.copy().astype(np.str)
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
        return x, y

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

            for i, j in enumerate(self.motion):
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
            result.append([last_point.x, last_point.y])
            last_point = last_point.parent
            # print cunt
            # cunt = cunt +1
        result.reverse()
        return result


if __name__ == "__main__":
    np.set_printoptions(linewidth=90)
    a_star = A_Star(-3, 4, 4.5, 9)
    route = a_star.route_plan()
    print route
    a_star.print_result_grid(route)
