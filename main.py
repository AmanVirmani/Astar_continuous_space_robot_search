import numpy as np
import cv2
import math
import time
import obstacle_map

def load_map(fname = None):
    if fname is not None :
        map_ = cv2.imread(fname)
        return map_
    world= 255*np.ones((200,300,3))
    rc=0
    obstacle_map.obstacle_circle(world)
    obstacle_map.obstacle_ellipse(world)
    obstacle_map.obstacle_rhombus(world)
    obstacle_map.obstacle_rectangle(world)
    obstacle_map.obstacle_polygon(world)

    cv2.imwrite('./map.jpg',world)
    return world


def getRC():
   r = int(input('Enter radius of the robot: '))
   c = int(input('Enter desired clearance: '))
   return r,c


def getStartNode(map_):
    print("Enter the start co-ordinates")
    rows, cols= map_.shape[:2]
    while True :
        ## Cartesian Form
        x = int(input("x_intial is: "))
        y = int(input("y_intial is: "))
        ## image coordinates
        row = rows-y-1 ; col = x
        if not isValidNode(map_, row, col):
            print('Input Node not within available map range. Please enter again!')
        else:
            break
    return (row, col)


def getGoalNode(map_):
    print("Enter the goal co-ordinates")
    rows, cols= map_.shape[:2]
    while True:
        ## Cartesian Form
        x = int(input("x_goal is: "))
        y = int(input("y_goal is: "))
        ## image coordinates
        row = rows-y-1 ; col = x
        if not isValidNode(map_, row, col):
            print('Input Node not within available map range. Please enter again! ')
        else:
            break;
    return (row, col)


def main():
    r,c = getRC()
    map_ = load_map()
    start_node = getStartNode(map_, r+c)
    goal_node = getGoalNode(map_, r+c)
    rows, cols = map_.shape[:2]
    arr = np.array([[Node() for j in range(cols)] for i in range(rows)])
    arr[start_node[0]][start_node[1]].visited = True
    arr[start_node[0]][start_node[1]].cost = 0
    queue = PriorityQueue()
    queue.put((arr[start_node[0]][start_node[1]].cost, start_node))
    start_time = time.time()
    img = map_.copy()
    img[goal_node[0]][goal_node[1]] = (0,0,255)
    while queue:
        curr_node = queue.get()[1]
        if (curr_node == goal_node):
            algo_time = time.time()
            print('found Goal in {}s at cost {}!!'.format(algo_time-start_time, arr[goal_node[0]][goal_node[1]].cost))
            tracePath(arr,map_,goal_node,r)
            break
        arr[curr_node[0]][curr_node[1]].visited = True
        arr = updateNeighbours(arr, map_, curr_node, queue, r+c)
        img[curr_node[0]][curr_node[1]] = (0, 255,  0)
        cv2.imshow('explored',img)
        cv2.waitKey(10)
        #exploredList.append(img)
    #saveVideo(exploredList,'explored.avi')
    cv2.destroyAllWindows()


if __name__=='__main__':
    main()
