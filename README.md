# GROMI SCAN PLANNING
This is based on the jackal velodyne simulation package.

## Introduction
####  Objective
The goal is to autonomously navigate and map an unknown area using SLAM and a frontier exploration algorithm based on laser data from a Velodyne VLP-16 LIDAR.

#### How it works
It is demostrated on the simulated environment.
First, it tries to find the frontier locations which is the border of the free space and unknown space.
It then decides the next goal location among the frontier locations and calculates the best scan location near the next goal location.
For your information, in the videos, the blue marks are the frontier locations and the red circles are the designated scan locations.

#### Installation
- Tested on Ubuntu 18.04 with ROS Melodic
- To run this code, please install the following packages:
```bash
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-tf2-sensor-msgs
sudo apt-get install ros-melodic-jackal-*
sudo apt-get install ros-melodic-velodyne-*
sudo apt-get install ros-melodic-octomap-mapping 
sudo apt-get install ros-melodic-pointcloud-to-laserscan
sudo apt-get install python-scipy

python -m pip install --upgrade pip
pip install scikit-build
pip install cmake
pip install opencv-python
```

## Run the Simulation
- Launch : `roslaunch gromi_scan_planning simulation_gromi.launch`

#### Nodes
##### Exploration Node
[`exlore.py`](src/explore.py)

This node is the brain that computes what the next goal position and scan location should be the robot should go to.

Subscribed Topics:
- `/map` for updating the current map
- `/move_base/global_costmap/costmap` for updating the current global costmap
- `/move_base/global_costmap/costmap_updates` for updating the current global costmap
- `/move_base/result` for determining when the robot has reached its goal position
- `/move_base/feedback` for determining where the robot is in the map at any given point in time
- `/scan_cloud` for subscribing the front 2d Lidar point cloud data
- `/octomap_point_cloud_centers' for subscribing octomap point cloud center data 

Published Topic: 
- `/move_base_simple/goal` for the next goal location
- '/visualization_marker' for visualizing the frontier locations (blue markers)
- '/scan_locations' for visualizing the scan locations (red circles)


## Demo Video
- A video of Jackal exploring a simulated environment (rectangular objects) in Gazebo and visualizing in RViZ [here](https://www.youtube.com/watch?v=RbIVOsBVjnk).
- A video of Jackal exploring a simulated environment (cylinder objects) in Gazebo and visualizing in RViZ [here](https://www.youtube.com/watch?v=74FLvJFuMgo).
