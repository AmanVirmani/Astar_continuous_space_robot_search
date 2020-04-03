import numpy as np
from utils import const
from queue import PriorityQueue

class PathPlanning:

    def __init__(self,start_node, goal_node, step_size, step_theta):
        self.start_node = start_node
        self.goal_node = goal_node
        self.step_size = step_size
        self.step_theta = step_theta
        # Store map dimensions
        self.map_size = const.map_size[0], const.map_size[1], (const.total_angle // step_theta)
        # Define an empty list to store all generated nodes and path nodes
        self.generated_nodes = []
        self.path_nodes = []
        # Define 3-D arrays to store information about generated nodes and parent nodes
        self.parent = np.full(fill_value=const.no_parent, shape=self.map_size)
        # Define a 3-D array to store base cost of each node
        self.base_cost = np.full(fill_value=const.no_parent, shape=self.map_size)
        # TODO: define a valid action set
        self.action_set = 1

    def get_cost2go(self, curr_node):
        if const.heuristic == "euclidean":
            return np.sqrt(((self.goal_node[0] - self.curr_node[0]) ** 2) + ((self.goal_node[1] - self.curr_node[1]) ** 2))

    def get_cost2come(self, curr_node, child_node):
        return np.sqrt(((self.child_node[0]- self.curr_node[0])**2) + ((self.child_node[1]- self.curr_node[1])**2)) + \
                self.base_cost[curr_node[0]][curr_node[1]][curr_node[2]]

    def get_cost(self,curr_node, child_node):
        if curr_node == self.start_node:
            return self.get_cost2go(curr_node)
        return self.get_cost2come(curr_node,child_node) + self.get_cost2go(child_node)

    # TODO: update get_child_node method
    def get_child_node(self, curr_node, action):
        child_node = curr_node
        return child_node

    def is_valid(self, curr_node):
        if 0 < curr_node[0] < self.map_size[0] and 0 < curr_node[1] < self.map_size[1]:
            return True
        else:
            return False

    def Astar(self, map_):
        explore_queue = PriorityQueue()
        explore_queue.put((self.get_cost(self.start_node, None), self.start_node))

        while explore_queue:
            curr_node = explore_queue.get()[1]
            if self.get_cost2go(curr_node) < const.goal_thresh:
                # TODO: write a traceback method
                print("Found goal")

            for action in self.action_set:
                child_node = self.get_child_node(curr_node, action)
                if self.is_valid(child_node) and self.get_cost2come(curr_node, child_node) \
                    < self.base_cost[[child_node[0]][child_node[1]][child_node[2]]]:
                    self.base_cost[[child_node[0]][child_node[1]][child_node[2]]] = \
                        self.get_cost2come(curr_node, child_node)
                    explore_queue.put((self.get_cost(curr_node, child_node), child_node))
