from utils import obstacle_map, const, planning
import cv2
import argparse
import numpy as np


# Function to create the map
def load_map(fname= './map.jpg', rc=0):

    map_ = cv2.imread('./map.jpg')
    if map_ is not None:
        print("map image found in the current directory")
        return map_

    # defining 200x300 pixel empty world
    world = 255 * np.ones((const.height, const.width, 3))

    # creating the obstacles
    obstacle_map.obstacle_circle(world)
    obstacle_map.obstacle_ellipse(world)
    obstacle_map.obstacle_rhombus(world)
    obstacle_map.obstacle_rectangle(world)
    obstacle_map.obstacle_polygon(world)

    obstacle_map.erode_image(world, rc)

    # save the image of the world
    cv2.imwrite('./outputs/map.jpg', world)

    return world


def is_valid(map_, curr_node):
    if 0 < curr_node[0] < map_.shape[0] and 0 < curr_node[1] < map_.shape[1] \
            and (map_[curr_node[0]][curr_node[1]] == [255, 255, 255]).all():
        return True
    else:
        return False


def main(args):
    rc = args['radius'] + args['clearance']
    map_ = load_map(rc=rc)
    h, w = map_.shape[:2]
    scaled_map = cv2.resize(map_.copy(), (w * const.scaling_factor, h * const.scaling_factor))

    start_node = planning.cartesian_to_img(scaled_map, args['start_node'])
    goal_node = planning.cartesian_to_img(scaled_map, args['goal_node'])

    if start_node is None or goal_node is None:
        exit(1)

    a_star = planning.PathPlanning(start_node, goal_node, const.scaling_factor * args["step_size"], 30)
    a_star.Astar(scaled_map)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--radius", required=False, help="Radius of the robot",
                    default=1, type=int)
    ap.add_argument("-c", "--clearance", required=False, help="desired clearance to avoid obstacles",
                    default=1, type=int)
    ap.add_argument("-d", "--step_size", required=False, help="step size robot takes in each move",
                    default=10, type=int)
    ap.add_argument("-s", "--start_node", required=True, help="Location (x, y, theta) of start node",
                    nargs='+', type=int)
    ap.add_argument("-g", "--goal_node", required=True, help="Location (x, y, theta) of goal node",
                    nargs='+', type=int)
    args = vars(ap.parse_args())
    main(args)
