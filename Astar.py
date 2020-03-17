import numpy as np
import cv2
import math
import obstacle_map
import time
from Queue import PriorityQueue

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


def isValidNode(map_,x,y,r):
	rows,cols = map_.shape[:2]
	if 0 <= x-r and x+r < rows and 0 <= y-r and y+r < cols:
		if not detectCollision(map_, (x, y), r):
			return True
		else :
			return False
	else:
		return False

def detectCollision(img, center,radius):
	for i in range(2*radius+1):
		for j in range(2*radius+1):
			if i**2+j**2 <= radius**2:
				if not ((img[center[0]+i][center[1]+j]==(255,255,255)).all() and (img[center[0]+i][center[1]-j]==(255,255,255)).all()\
						and (img[center[0]-i][center[1]-j]==(255,255,255)).all() and (img[center[0]-i][center[1]+j]==(255,255,255)).all()):
					return True
	return False


def euclideanDistance(state1, state2):
	return np.sqrt(((state1[0] - state2[0]) ** 2) + ((state1[1] - state2[1]) ** 2))


def getStartNode(map_):
	print("Enter the start co-ordinates")
	rows, cols= map_.shape[:2]
	while True :
		## Cartesian Form
		x = int(input("x_intial is: "))
		y = int(input("y_intial is: "))
		theta = int(input("theta_intial is (in degree): "))/30
		## image coordinates
		row = rows-y-1 ; col = x
		if not isValidNode(map_, row, col,0):
			print('Input Node not within available map range. Please enter again!')
		else:
			break;
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
		if not isValidNode(map_, row, col,0):
			print('Input Node not within available map range. Please enter again! ')
		else:
			break;
	return (row, col)

# A node structure for our search tree
class Node:
	# A utility function to create a new node
	# visited : flag to identify visited nodes
	# parent : coordinate location of the parent
	# cost : cost of reaching current node from start node
	def __init__(self, visited=False,parent=None, cost = float('inf') ):
		self.visited = visited
		self.parent = parent
		self.cost = cost

def updateNeighbours(arr, map_, curr_node,queue,visited,goal_node,step_size=2,theta_size=30):
	x,y,theta = curr_node
	threshold = 0.5
	for angle in range(-2, 2,1):
		theta_new = (theta+angle)*theta_size
		x_new = x + step_size*math.cos(theta_new)
		x_new = int(round(x_new)/0.5)
		y_new = y + step_size*math.sin(theta_new)
		y_new = int(round(y_new)/0.5)
		theta_new = theta_new/theta_size
		if isValidNode(map_,x_new,y_new,theta_new,0) and not visited[x_new][y_new][theta_new]:
			arr[x_new][y_new] = arr[x][y].cost + euclideanDistance((x_new,y_new),goal_node)
			queue.put((arr[x_new][y_new].cost,(x_new,y_new,theta_new)))
			arr[x_new][y_new].parent = (x,y)


def tracePath(arr,map_,goal_node):
	images= []
	output = './output.avi'
	curr_node = goal_node
	img = map_.copy()
	while curr_node is not None:
		img[curr_node[0]][curr_node[1]] = (0,0,255)
		images.append(img)
		img[curr_node[0]][curr_node[1]] = (0,0,0)
		curr_node = arr[curr_node[0]][curr_node[1]].parent

	images = images[::-1]
	saveVideo(images,output)

def saveVideo(images,output='path.avi'):
	h,w = images[0].shape[:2]
	out = cv2.VideoWriter(output,cv2.VideoWriter_fourcc('M','J','P','G'), 1, (w,h))
	images= np.uint8(images)
	for img in images:
		cv2.imshow('path traced',img)
		cv2.waitKey(10)
		out.write(img)
	out.release()

def exploredPath(map_,arr):
	img = map_.copy()
	rows,cols = map_.shape[:2]
	for row in range(rows):
		for col in range(cols):
			if arr[row][col].visited:
				img[row][col]==(0,255,255)
	return img


def main():
	map_ = load_map()
	start_node = getStartNode(map_)
	goal_node = getGoalNode(map_)
	rows, cols = map_.shape[:2]
	depth = 12
	arr = np.array([[Node()for j in range(int(cols/0.5))] for i in range(int(rows/0.5))])
	visited = np.zeros((600,400,12))
	visited[start_node[0]][start_node[1]][start_node[2]] = True
	arr[start_node[0]][start_node[1]].cost = 0
	queue = PriorityQueue()
	queue.put((arr[start_node[0]][start_node[1]].cost, start_node))
	start_time = time.time()
	exploredList= []
	img = map_.copy()
	img[goal_node[0]][goal_node[1]] = (0,0,255)
	while queue:
		curr_node = queue.get()[1]
		if (curr_node[:2] == goal_node[:2]):
			algo_time = time.time()
			print('found Goal in {}s at cost {}!!'.format(algo_time-start_time, arr[goal_node[0]][goal_node[1]].cost))
			tracePath(arr,map_,goal_node)
			break
		arr[curr_node[0]][curr_node[1]].visited = True
		arr = updateNeighbours(arr, map_, curr_node,queue,visited = visited,goal_node=goal_node)
		img[curr_node[0]][curr_node[1]] = (0,255,0)
		#exploredList.append(img)
		cv2.imshow('explored',img)
		cv2.waitKey(10)
	#saveVideo(exploredList,'explored.avi')
	cv2.destroyAllWindows()

if __name__=='__main__':
	main()
