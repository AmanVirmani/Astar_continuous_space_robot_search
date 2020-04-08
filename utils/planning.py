import numpy as np
from utils import const
from queue import PriorityQueue
import cv2
import imageio
import time

class PathPlanning:

    def __init__(self,start_node, goal_node, step_size, step_theta):
        self.step_theta = step_theta
        self.start_node = start_node[0], start_node[1], start_node[2]//self.step_theta
        self.goal_node = goal_node[0], goal_node[1]
        self.step_size = step_size
        # Store map dimensions
        self.map_size = const.map_size[0], const.map_size[1], (const.total_angle // step_theta)
        # Define 3-D arrays to store information about generated nodes and parent nodes
        self.parent = np.full(fill_value=const.no_parent, shape=self.map_size)
        # Define a 3-D array to store base cost of each node
        self.base_cost = np.full(fill_value=const.default_base_cost, shape=self.map_size)
        self.base_cost[self.start_node[0]][self.start_node[1]][self.start_node[2]] = 0

    def get_cost2go(self, curr_node):
        if const.heuristic == "euclidean":
            return np.sqrt(((self.goal_node[0] - curr_node[0]) ** 2) + ((self.goal_node[1] - curr_node[1]) ** 2))

    def get_cost2come(self, curr_node, child_node):
        return np.sqrt(((child_node[0]- curr_node[0])**2) + ((child_node[1]- curr_node[1])**2)) + \
                self.base_cost[curr_node[0]][curr_node[1]][curr_node[2]]

    def get_cost(self,curr_node, child_node):
        if curr_node == self.start_node:
            return self.get_cost2go(curr_node)
        return self.get_cost2come(curr_node,child_node) + self.get_cost2go(child_node)

    def get_child_node(self, curr_node, action):
        theta_new = (curr_node[2]+action-int(const.max_actions/2))*self.step_theta
        x_new = round(curr_node[0] + self.step_size*np.sin(np.radians(theta_new)))
        y_new = round(curr_node[1] + self.step_size*np.cos(np.radians(theta_new)))
        return int(x_new), int(y_new), int(theta_new // self.step_theta) % (12)

    def is_valid(self, map_, curr_node):
        if 0 < curr_node[0] < self.map_size[0] and 0 < curr_node[1] < self.map_size[1]\
                and (map_[curr_node[0]][curr_node[1]] == [255, 255, 255]).all():
            return True
        else:
            return False

    def Astar(self, map_):
        start_time = time.time()
        explore_queue = PriorityQueue()
        explore_queue.put((self.get_cost(self.start_node, None), self.start_node))
        img = cv2.circle(map_, (self.start_node[1], self.start_node[0]), 3, [0, 0, 0], -1)
        img = cv2.circle(img, (self.goal_node[1], self.goal_node[0]), 3, [0, 0, 0],  -1)
        #cv2.imshow('curr', map_); cv2.waitKey(0); cv2.destroyAllWindows()

        while explore_queue:
            curr_node = explore_queue.get()[1]
            if self.get_cost2go(curr_node) < const.goal_thresh:
                print("Found goal in {}s".format(time.time()-start_time))
                self.find_optimal_path(img, curr_node)
                break

            for action in range(const.max_actions):
                child_node = self.get_child_node(curr_node, action)
                if self.is_valid(map_, child_node) and self.get_cost2come(curr_node, child_node) \
                    < self.base_cost[child_node[0]][child_node[1]][child_node[2]]:
                    img[child_node[0]][child_node[1]] = [0, 255, 0]
                    #cv2.imshow('curr', map_); cv2.waitKey(1)#; cv2.destroyAllWindows()
                    self.base_cost[child_node[0]][child_node[1]][child_node[2]] = \
                        self.get_cost2come(curr_node, child_node)
                    explore_queue.put((self.get_cost(curr_node, child_node), child_node))
                    self.parent[child_node[0]][child_node[1]][child_node[2]] = \
                        np.ravel_multi_index([curr_node[0], curr_node[1], curr_node[2]], dims=self.map_size)

    def find_optimal_path(self, img, curr_node):
        nodes = []
        images = []
        output = '../outputs/output.gif'
        while self.parent[curr_node[0]][curr_node[1]][curr_node[2]] != const.no_parent:
            nodes.append(curr_node)
            curr_node = np.unravel_index(self.parent[curr_node[0]][curr_node[1]][curr_node[2]], dims=self.map_size)
        nodes = nodes[::-1]

        for i in range(len(nodes)-1):
            cv2.line(img, (nodes[i+1][1],nodes[i+1][0]), (nodes[i][1],nodes[i][0]), (0, 0, 255), thickness=1, lineType=8)
            images.append(np.uint8(img.copy()))
        imageio.mimsave(output, images, fps=55)

def cartesian_to_img(map_, node):
    rows, cols, ch = map_.shape
    x = const.scaling_factor*node[0]
    y = rows - const.scaling_factor*node[1] - 1
    if  0< x < cols and 0 < y < rows and (map_[y][x] == [255,255,255]).all():
        return y, x, node[2]
    else:
        return None

if __name__=='__main__':
    map_ = cv2.imread('../map.jpg')
    map_ = cv2.resize(map_, (600, 400))
    start_node = [50, 30, 60]
    goal_node = [295, 195, 0]
    start_node = cartesian_to_img(map_, start_node)
    goal_node = cartesian_to_img(map_, goal_node)
    if start_node is None or goal_node is None:
        exit(1)
    a_star = PathPlanning(start_node, goal_node, 10, 30)
    a_star.Astar(map_)