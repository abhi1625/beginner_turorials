# ROS beginner tutorials - Publisher and Subscriber nodes
<a href='https://github.com/abhi1625/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
## Overview
This project provides steps to create a basic publisher and subscriber node in ROS. The process followed is as given in the official ROS tutorials [wiki](http://wiki.ros.org/ROS/Tutorials).

The nodes created are as follows:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)

## Dependencies
The following dependencies are required to run this package:

- ROS kinetic
- catkin
- Ubuntu 16.04
For installing ROS, follow the process given [here](http://wiki.ros.org/kinetic/Installation)

For installing catkin, follow the process given [here](http://wiki.ros.org/catkin#Installing_catkin)

**Note:** catkin is usually installed by default when ROS is installed.

## Build the package
To build the given project first create a catkin workspace, clone this repository and then build by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/abhi1625/beginner_tutorials.git
cd ..
catkin_make
```
**Note:** To checkout the Week10_HW branch use this command once the repo is cloned.
```
git checkout Week10_HW
```

## Running the package
Follow the given steps to run the project:

1. Open a new terminal and start roscore
```
roscore
```
2. Source the setup.bash file of your catkin_ws:
```
cd ~/catkin_ws/
source devel/setup.bash
```
3. Run the publisher node using rosrun 
```
rosrun beginner_tutorials talker
```
4. Run the subscriber node using rosrun 
```
rosrun beginner_tutorials listener
```
**Note:** From each new terminal source devel/setup.bash file of your workspace in order to execute any ros commands linked to the package.

## Run the nodes using launch file
After building the project you can also use the launch file to run the two nodes. The steps are as follows:
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
You can also specify publishing rate as an argument(The default rate is set as 10) 
```
roslaunch beginner_tutorials beginner_tutorial.launch rate:=5
```
## Using the modify string service
A service to modify string has been added to the project which modifies the base string being published by the talker. Once both the nodes are running, you can use the `rosservice` command to change the string message as follows:
```
rosservice call /modify_string "New string"
``` 
Now the talker will start publishing "New string"

## Checking the log messages
The output of the rqt_console with info and warn logger levels is added in the results folder. To check the logs using the GUI use:
```
rqt_console
```


