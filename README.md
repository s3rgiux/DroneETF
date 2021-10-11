# Autonomous drone
<p align="center">
    <img src="/images/mirai_logo.png" alt= "Mirai logo" width="512"/>
</p>

## Requirements

- ROS Melodic
- Python 2.7
- MAVproxy 1.8
- Pymavlink 2.4
- Dronekit 2.9
- Dronekit-sitl 3.3
- OpenCV Contrib
- Ardupilot: cloned from the repository and followe the instructions. [Link to Ardupilot page](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux).

## Plugins
Gazebo world downloaded from [Ardupilot page](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html).

## Gazebo
At the path `~/ardupilot_gazebo/models/`, there is a folder called `iris_with_ardupilot`.
Change this folder with the one in this repository (make sure this files are also in the path `/usr/share/gazebo-9/models`) and add the `gimbal_2d`, `lidar_h`, `lidar_v` and `aruco` folders to get the model with lidar and camera. 

At the path `~/ardupilot_gazebo/worlds/`, we need to create a world file. This file is attached to this repository.

## ROS
Remember create a catkin workspace and a ROS package. This files are already include in this repository. Nevertheless, don't forget to source your package and workspace with `nano ~/.bashrc`.

At the path `/opt/ros/melodic/share/gazebo_ros/launch` we need to create a launch file for our world. This file is attached to this repository.

### MAVROS and rqt
To get the ROS topics from MAVlink we need to instal MAVROS. Follow the steps from the [Ardupilot page](https://ardupilot.org/dev/docs/ros-sitl.html). (Already done in this repository)

### YOLO
Follow instructions for YOLO in ROS by [ETH Zurich](https://github.com/leggedrobotics/darknet_ros).
**NOTE:** this only works with a Nvidia graphics card and CUDA enabled.

## How to run
1. In one terminal...
```python
# For run only Gazebo
$ gazebo --verbose worlds/drone.world
```
```python
# For Gazebo with ROS
$ roslaunch gazebo_ros drone_world.launch
```

2. In a new terminal...
```python
$ cd ~/ardupilot/ArduCopter
$ sim_vehicle.py -f gazebo-iris --console --map
```

3. In a new terminal...
```python
$ cd ~/Drone_EFT/src/ros_drone_node/launch
$ roslaunch apm.launch
```

4. In a new terminal...
```python
$ cd ~/Drone_EFT/src/src/darknet_ros/darknet_ros/launch
$ roslaunch darknet_ros.launch
```

5. In a new terminal...
```python
# Search for Visualization -> Image View and select the topic /detected_markers
$ rqt
```

6. In a new terminal...
```python
# Using rosrun
$ cd ~/Drone_EFT/src/ros_drone_node/scripts
$ rosrun ros_drone_node ros_main.py
```
```python
# Using dronekit
$ cd ~/Drone_EFT/src/ros_drone_node/scripts
$ python main.py --connect 127.0.0.1:14550
```

## Resources
- [Ardupilot page](https://ardupilot.org)
- [Dronekit Docs](https://dronekit-python.readthedocs.io/en/latest/)
- [Gazebo model index](http://models.gazebosim.org/)
- [Gazebo plugins in ROS](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [MAVROS Messages and Services](http://wiki.ros.org/mavros_msgs)
- [ROS python library](http://wiki.ros.org/rospy)
- [IQ Python API](https://github.com/Intelligent-Quads/iq_gnc/blob/master/docs/py_gnc_functions.md)
- [OpenCV Contrib Docs](https://github.com/opencv/opencv_contrib)

## Reference images
Added code lines to bash:
![Image of bash](/images/bashrc_sources.png)

## TODO
- [X] Dronekit code
- [X] Gazebo connected to Ardupilot
- [X] Drone moving in Gazebo environment
- [X] Add camera and lidar to drone model
- [X] Connect with ROS
- [X] Visualize camera view and lidar raw data
- [X] Create ROS code
- [X] Collision avoidance algorithm
- [X] OpenCV algorithm

##Creator

- Domenico Morales

##Support

- Sergio Reyes 
