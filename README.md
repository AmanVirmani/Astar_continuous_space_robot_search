# ENPM661_Proj3_Phase2
A* implementation for Rigid Robot


## Installation
```bash
pip install opencv-contrib-python
pip install numpy
```

## Instructions to search goal using Astar algorithm for a Rigid Robot
```bash
git clone https://github.com/AmanVirmani/Astar_continuous_space_robot_search.git
python Astar_rigid.py
```
The user will then be prompted to enter the start position in the cartesian coordinates, the start angle in degrees (0 is straigt down and it goes counterclockwise), the radius of the robot, the clearance, and finally the goal coordinates. Once the program finishes, the optimal path to the goal is displayed. Consider the example below.

```
Enter the start coordinates
x_intial is: 50
y_intial is: 30
theta_intial is (in degree): 60
radius is: 1
clearance is:1
step size is: 1
Enter the goal coordinates
x_goal is: 150
y_goal is: 150
found Goal in 3246s
```

## Execution time for the Algorithm (Point Robot)
The file `explored.avi` shows the animation for the exploration of the search space.

The file `output.avi` shows the animation for the optimal path after the exploration.
