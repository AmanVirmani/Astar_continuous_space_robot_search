# ENPM661_Proj3_Phase2
[![Build Status](https://travis-ci.com/AmanVirmani/Astar_continuous_space_robot_search.svg?branch=master)](https://travis-ci.com/AmanVirmani/Astar_continuous_space_robot_search)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
A* implementation for Rigid Robot
<p align="center">
  <img src="https://github.com/AmanVirmani/Astar_continuous_space_robot_search/blob/master/outputs/output.gif">
  <br><b>Fig 1: Optimal path found for a rigid robot using A* Algorithm</b><br>
</p>


## Dependencies

The following dependencies must be installed.

1. python3.5 or above 
2. numpy 
3. opencv 3.4 or above
4. imagio 

Enter the given commands in bash terminal to install the dependencies.
```bash
sudo apt-get install python3
pip3 install numpy opencv-python imageio
```

## Build Instructions

Run the following command to do path planning for a rigid robot using A* algorithm

```bash
git clone https://github.com/AmanVirmani/Astar_continuous_space_robot_search.git
python main.py -s <start_node> -g <goal_node>
```

## Output

The file `outputs/output.gif` shows the animation for the optimal path after the exploration.
