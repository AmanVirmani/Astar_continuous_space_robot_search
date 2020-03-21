import obstacle_map
import numpy as np
import time
import math
import cv2
from queue import PriorityQueue


# Function to create the map
def load_map(fname = None):
    if fname is not None :
        map_ = cv2.imread(fname)
        return map_

    # defining 200x300 pixel empty world
    world = 255*np.ones((200,300,3))

    # creating the obstacles
    obstacle_map.obstacle_circle(world)
    obstacle_map.obstacle_ellipse(world)
    obstacle_map.obstacle_rhombus(world)
    obstacle_map.obstacle_rectangle(world)
    obstacle_map.obstacle_polygon(world)

    # save the image of the world
    cv2.imwrite('./outputs/map.jpg',world)

    return world


# Function to check if a node is valid
def isValidNode(map_, x, y, theta, r=0):
    rows, cols = map_.shape[:2]

    # make sure coordinates are within map boundaries
    if 0 <= x-r and x+r < rows and 0 <= y-r and y+r < cols:
        # check if node is colliding with any obstacle
        if not detectCollision(map_, (x, y), r):
            return True
        else:
            return False
    else:
        return False


# Function to check if node is in colliding with any obstacle
def detectCollision(img, center, radius):
    for i in range(2*radius+1):
        for j in range(2*radius+1):
            if i**2+j**2 <= radius**2:
                if not ((img[center[0]+i][center[1]+j]==(255,255,255)).all() and (img[center[0]+i][center[1]-j]==(255,255,255)).all()\
                        and (img[center[0]-i][center[1]-j]==(255,255,255)).all() and (img[center[0]-i][center[1]+j]==(255,255,255)).all()):
                    return True
    return False


# Function to calculate the Euclidean Distance between two nodes
def euclideanDistance(state1, state2):
    return np.sqrt(((state1[0] - state2[0]) ** 2) + ((state1[1] - state2[1]) ** 2))


# Function to take user input start coordinates, theta, radius, and clearance
def getStartNode(map_,radius):
    print("Enter the start coordinates")
    rows, cols = map_.shape[:2]
    while True :
        # Cartesian Form
        x = int(input("x_intial is: "))
        y = int(input("y_intial is: "))
        theta = int(int(input("theta_intial is (in degree): "))/30)
        # x, y, theta = (50, 30, 2)

        # converting to image coordinates
        row = rows-2*y-1 ; col = 2*x
        if not isValidNode(map_, row, col, theta, radius):
            print('Input Node not within available map range. Please enter again!')
        else:
            break
    return row, col, theta


# Function to take user input goal coordinates
def getGoalNode(map_,radius):
    print("Enter the goal coordinates")
    rows, cols= map_.shape[:2]
    while True:
        # Cartesian Form
        x = int(input("x_goal is: "))
        y = int(input("y_goal is: "))
        #x, y = 150, 150
        # image coordinates
        row = rows-2*y-1 ; col = 2*x
        if not isValidNode(map_, row, col, 0, radius):
            print('Input Node not within available map range. Please enter again! ')
        else:
            break
    return row, col


# A node structure for our search tree
class Node:
    # A utility function to create a new node
    # visited : flag to identify visited nodes
    # parent : coordinate location of the parent
    # cost : cost of reaching current node from start node
    # costCome : euclidean distance cost of reaching goal node from current node
    def __init__(self, visited=False,parent=None, costCome = float('inf'), cost = 0):
        self.visited = visited
        self.parent = parent
        self.costCome = costCome
        self.cost = cost


# Function to append valid neighbours of the current node in the search tree
def updateNeighbours(arr, map_, path_img, curr_node, queue, goal_node, step_size, radius):
    x, y, theta = curr_node
    theta_size = 30
    # iterate for each of the five directions it can go
    for angle in range(-2, 3, 1):
        theta_new = (theta+angle)
        if theta_new >= 12:
            theta_new -= 12
        if theta_new < 0:
            theta_new += 12
        x_new = round(x + step_size*math.sin(math.radians(theta_new*theta_size)))
        y_new = round(y + step_size*math.cos(math.radians(theta_new*theta_size)))
        if isValidNode(map_,x_new,y_new,theta_new, radius):
            # calculate the cost to come for the new node
            arr[x_new][y_new][theta_new].costCome = arr[x][y][theta].costCome + euclideanDistance((x_new,y_new),(x,y))
            # calculate the total cost for the new node
            arr[x_new][y_new][theta_new].cost = arr[x_new][y_new][theta_new].costCome + euclideanDistance((x_new,y_new),goal_node)
            # check if visited
            if arr[x_new][y_new][theta_new].visited is False:
                # mark the node as visited
                arr[x_new][y_new][theta_new].visited = True
                # add the node to the queue
                queue.put((arr[x_new][y_new][theta_new].cost,(x_new,y_new,theta_new)))
                arr[x_new][y_new][theta_new].parent = (x,y,theta)
                # draw the lines of the path on the map
                cv2.line(path_img, (y_new,x_new), (y, x), (0, 255, 0), thickness=1, lineType=8)
            else:
                # if the current node is better than the already visited node
                if arr[x_new][y_new][theta_new].cost > arr[x][y][theta].costCome + euclideanDistance((x_new,y_new),(x,y)) + euclideanDistance((x_new,y_new),goal_node):
                    queue.put((arr[x_new][y_new][theta_new].cost,(x_new,y_new,theta_new)))
                    arr[x_new][y_new][theta_new].parent = (x,y,theta)
    return arr, map_, path_img


def tracePath(arr, img, curr_node):
    img = np.uint8(img)
    nodes = []
    output = './outputs/output.avi'
    h, w = img.shape[:2]
    out = cv2.VideoWriter(output,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 1, (w, h))
    while curr_node is not None:
        nodes.append(curr_node)
        curr_node = arr[curr_node[0]][curr_node[1]][curr_node[2]].parent
    nodes = nodes[::-1]
    for i in range(len(nodes)-1):
        cv2.line(img, (nodes[i+1][1],nodes[i+1][0]), (nodes[i][1],nodes[i][0]), (0, 0, 255), thickness=1, lineType=8)
        out.write(img.copy())
    out.release()


def main():
    map_ = load_map('./map.jpg')
    img = map_.copy()
    threshold = 0.5
    img = cv2.resize(img,(600,400))
    path_img = img.copy()

    r = int(input("Robot's radius is: "))
    c = int(input("Desired clearance is: "))
    start_node = getStartNode(img,r+c)
    step_size = int(input("step size is: "))

    goal_node = getGoalNode(img,r+c)
    step_size = step_size*2
    rows, cols = map_.shape[:2]

    arr = np.array([[[Node() for k in range(int(360/30))]for j in range(int(cols/threshold))] for i in range(int(rows/threshold))])
    arr[start_node[0]][start_node[1]][start_node[2]].visited = True
    arr[start_node[0]][start_node[1]][start_node[2]].costCome = 0    
    
    queue = PriorityQueue()
    queue.put((arr[start_node[0]][start_node[1]][start_node[2]].cost, start_node))

    output = './outputs/explored.avi'
    h, w = img.shape[:2]
    out = cv2.VideoWriter(output, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 1, (w, h))
    start_time = time.time()

    path_img = cv2.circle(path_img, (start_node[1], start_node[0]), r+c, (0,0,0), 2)
    path_img = cv2.circle(path_img, (goal_node[1], goal_node[0]), r+c, (0,0,0), 2)
    while queue:
        curr_node = queue.get()[1]
        if euclideanDistance(curr_node[:2], goal_node) < 3:
            algo_time = time.time()
            print('found Goal in {}s at cost {}!!'.format(algo_time-start_time, arr[curr_node[0]][curr_node[1]][curr_node[2]].cost))
            tracePath(arr, path_img, curr_node)
            break
        arr, img, path_img = updateNeighbours(arr, img, path_img, curr_node, queue, goal_node, step_size,r+c)
        out.write(np.uint8(path_img))
        cv2.imshow('process', path_img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
    out.release()
    cv2.destroyAllWindows()


if __name__=='__main__':
    main()
