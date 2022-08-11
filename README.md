# GROMI SCAN PLANNING
This is based on the jackal velodyne simulation package.

## Introduction
####  Objective
The goal of this project is for [Jackal](https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) (a Clearpath UGV) to autonomously navigate and map an unknown area using SLAM and a custom made frontier exploration algorithm based on laser data from a [Velodyne VLP-16 LIDAR](http://velodynelidar.com/vlp-16.html).

#### How it works
- The  [NavFN](http://wiki.ros.org/navfn?distro=indigo) planner is used for global path planning
- The [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) is used for local path planning
- A custom made exploration planner is used for determining the next goal position to go to. My algorithm is based off a frontier exploration algorithm described in this [paper](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf)
- The [gmapping](http://wiki.ros.org/gmapping?distro=indigo) package is used for Simultaneous Localization and Mapping (SLAM)

## Installation
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

This node is the brain that computes what the next goal position should be that Jackal should go to. See my portfolio post that goes into detail about this [here](https://mechwiz.github.io/Portfolio/).

Subscribed Topics:
- `/map` for updating the current map
- `/move_base/global_costmap/costmap` for updating the current global costmap
- `/move_base/global_costmap/costmap_updates` for updating the current global costmap
- `move_base/result` for determining when Jackal has reached its goal position
- `move_base/feedback` for determining where Jackal is in the map at any given point in time

Published Topic: `/move_base_simple/goal`

#### Robot Localization
In the development process, several uncharacteristic behaviors were noticed:
- z-drift between the `odom` and `base_link` frames
- Pitch and Roll drift

The culprit seemed to be due to the default [robot_localization.yaml](https://github.com/jackal/jackal/blob/kinetic-devel/jackal_control/config/robot_localization.yaml) file which sets the parameters for sensor fusion of the odometry and IMU data. Roll, pitch, and yaw data were being considered from the IMU but were not set to be _relative_ (i.e. calibrated so that the first data point is the new zero). This explained the uncharacteristic pitch and roll behavior. Due to the fact that this project is only meant for a flat surface anyway, the roll and pitch parameters were changed such that the robot-ekf does not even consider them. Similarly, the parameter file by default considers velocity along the z-axis from the robot odometry which probably led to the z-drift problem, especially since the _relative_ parameter was initially set to false. As this project is only meant for a flat surface, the z-axis velocity is not considered. These changes can be seen in the updated [robot_localization.yaml](params/robot_localization.yaml). This fixed the uncharacterisitc behavior that was seen.

## Demo & Future Improvements
#### Video
- A video of Jackal exploring a simulated environment in Gazebo can be found [here](https://www.youtube.com/watch?v=x4oIJKmgQMc).
- A video of Jackal exploring an actual hallway in Northwestern can be found [here](https://www.youtube.com/watch?v=8slMv4ZIi4U).
#### Further Improvements
- Making the exploration algorithm more efficient and robust.
