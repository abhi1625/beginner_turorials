# ROS beginner tutorials
<a href='https://github.com/abhi1625/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
## Overview
This project provides tutorials to create a basic publisher and subscriber node, set TF frames, Unit testing and creating bag files in ROS. The process followed is as given in the official ROS tutorials [wiki](http://wiki.ros.org/ROS/Tutorials).

The nodes created are as follows:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)
3. talkerTest - test/talkerTest (Unit tests)

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
**Note:** To checkout the Week11_HW branch use this command once the repo is cloned.
```
git checkout Week11_HW
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
To enable recording of all topics, you can specify argument `record:=true`(By default it is set as false).
```
roslaunch beginner_tutorials beginner_tutorial.launch record:=true
```
To examine the recorded bag file, use:
```
rosbag info results/allTopics.bag
```
This outputs the info of the bag file as:
```
path:        results/allTopics.bag
version:     2.0
duration:    14.9s
start:       Nov 11 2019 01:31:09.83 (1573453869.83)
end:         Nov 11 2019 01:31:24.73 (1573453884.73)
size:        187.7 KB
messages:    893
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      148 msgs    : std_msgs/String   
             /rosout       301 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   296 msgs    : rosgraph_msgs/Log 
             /tf           148 msgs    : tf2_msgs/TFMessage
```
Here it can be seen that the messages from the topic `/chatter` were collected in the rosbag. To play the rosbag use:
```
rosbag play results/allTopics.bag
``` 
In a new terminal run only the subscriber node, to read messages from the `/chatter` topic which is being published in the bag, using:
```
rosrun beginner_tutorials listener
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
## TF Frames
The talker node broadcasts a static /talk child frame with respect to the parent /world. To inspect the tf frame, launch the talker and listener nodes as follows:
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
Now in a new terminal, run the tf_echo command to verify the tf frames:
```
rosrun tf tf_echo
```
To further verify and visualize the tf frames, we can also use the `rqt_tf_tree` command:
```
rosrun rqt_tf_tree rqt_tf_tree
```
This generates a pdf showing the transmission of tf frame from the child to the parent.

## Running the tests
A rostest and gtest based framework has been utilized to write unit test cases for the project. The tests can be run in the following two ways:
- Using catkin_make
```
catkin_make run_tests beginner_tutorials
```
- Using rostest
```
rostest beginner_tutorials talkerTest.launch
```


