import obstacle_map
import numpy as np
import time
import math
import cv2
from queue import PriorityQueue


def load_map(fname = None):
	if fname is not None :
		map_ = cv2.imread(fname)
		return map_
	world= 255*np.ones((200,300,3))
	obstacle_map.obstacle_circle(world)
	obstacle_map.obstacle_ellipse(world)
	obstacle_map.obstacle_rhombus(world)
	obstacle_map.obstacle_rectangle(world)
	obstacle_map.obstacle_polygon(world)

	cv2.imwrite('./map.jpg',world)
	return world


def isValidNode(map_, x, y, theta, r=0):
	rows,cols = map_.shape[:2]
	if 0 <= x-r and x+r < rows and 0 <= y-r and y+r < cols:
		if not detectCollision(map_, (x, y), r):
			return True
		else :
			return False
	else:
		return False


def detectCollision(img, center,radius):
	# img = cv2.resize(map_,(int(map_.shape[1]/0.5),int(map_.shape[0]/0.5)))
	# center[0] = int(center[0]/0.5)
	# center[1] = int(center[1]/0.5)
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
		# x = int(input("x_intial is: "))
		# y = int(input("y_intial is: "))
		# theta = int(int(input("theta_intial is (in degree): "))/30)
		x, y, theta = (50, 50, 2)
		## image coordinates
		row = rows-2*y-1 ; col = 2*x
		if not isValidNode(map_, row, col,0):
			print('Input Node not within available map range. Please enter again!')
		else:
			break
	return row, col, theta

def getGoalNode(map_):
	print("Enter the goal co-ordinates")
	rows, cols= map_.shape[:2]
	while True:
		## Cartesian Form
		# x = int(input("x_goal is: "))
		# y = int(input("y_goal is: "))
		x, y = 150, 150
		## image coordinates
		row = rows-2*y-1 ; col = 2*x
		if not isValidNode(map_, row, col, 0, 0):
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
	def __init__(self, visited=False,parent=None, cost = float('inf') ):
		self.visited = visited
		self.parent = parent
		self.cost = cost

def updateNeighbours(arr, map_, curr_node,queue,goal_node,step_size=2,theta_size=30):
	x,y,theta = curr_node
	threshold = 1
	for angle in range(0, 12,1):
		theta_new = (theta+angle)%12
		x_new = int(x + step_size*math.cos(theta_new*theta_size))
		# x_new = round(x_new/threshold)
		y_new = int(y + step_size*math.sin(theta_new*theta_size))
		# y_new = round(y_new/threshold)
		if isValidNode(map_,x_new,y_new,theta_new, 0) and not arr[x_new][y_new][theta_new].visited:
			arr[x_new][y_new][theta_new].cost = arr[x][y][theta].cost + euclideanDistance((x_new,y_new),goal_node)
			queue.put((arr[x_new][y_new][theta_new].cost,(x_new,y_new,theta_new)))
			arr[x_new][y_new][theta_new].parent = (x,y,theta)

	return arr


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
	img = map_.copy()
	img = cv2.resize(img,(600,400))

	start_node = getStartNode(img)
	goal_node = getGoalNode(img)
	rows, cols = map_.shape[:2]

	arr = np.array([[[Node() for k in range(int(360/30))]for j in range(int(cols/0.5))] for i in range(int(rows/0.5))])
	arr[start_node[0]][start_node[1]][start_node[2]].visited = True
	arr[start_node[0]][start_node[1]][start_node[2]].cost = 0

	queue = PriorityQueue()
	queue.put((arr[start_node[0]][start_node[1]][start_node[2]].cost, start_node))

	start_time = time.time()
	exploredList= []

	img[goal_node[0]][goal_node[1]] = (0,0,255)
	while queue:
		curr_node = queue.get()[1]
		if euclideanDistance(curr_node[:2],goal_node) < 3:
			algo_time = time.time()
			print('found Goal in {}s at cost {}!!'.format(algo_time-start_time, arr[curr_node[0]][curr_node[1]][curr_node[2]].cost))
			#tracePath(arr,map_,goal_node)
			break
		arr[curr_node[0]][curr_node[1]][curr_node[2]].visited = True
		arr = updateNeighbours(arr, img, curr_node,queue,goal_node)
		img[curr_node[0]][curr_node[1]] = (0,255,0)
		#exploredList.append(img)
		cv2.imshow('explored', img)
		cv2.waitKey(1)
	#saveVideo(exploredList,'explored.avi')
	cv2.destroyAllWindows()

if __name__=='__main__':
	main()
