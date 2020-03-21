# ENPM661_Proj3_Phase2
A* implementation for Rigid Robot

The main file are main.py

## Installation
```bash
pip install opencv-contrib-python
pip install numpy
```

## Instructions to run Dijkstra for a Rigid Robot
```python
$ git clone https://github.com/AmanVirmani/Astar_continuous_space_robot_search.git
$ python codes\Dijkstra_point.py
```
The user will then be prompted to enter the start position in the cartesian coordinates, the start angle in degrees (0 is straigt down and it goes counterclockwise), the radius of the robot, the clearance, and finally the goal coordinates. Once the program finishes, the optimal path to the goal is displayed. Consider the example below.

```
Enter starting coordinates (x y): 5 5 
Enter goal coordinates (x y): 295 195
Reached Goal!
Time to run Dijkstra: 4.436329 seconds
```

## Execution time for the Algorithm (Point Robot)
The file `dijkstra_point_exploration.mp4` shows the animation for the exploration of the search space.

The file `dijkstra_point_optimal_path.mp4` shows the animation for the chosen optimal path after the exploration using the dijkstra algorithm.
