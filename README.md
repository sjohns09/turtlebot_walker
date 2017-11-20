# turtlebot_walker
A walker algorithm using the turtlebot and gazebo

## Overview 


## Dependencies

 - ROS Kinetic is installed on machine
 - "beginner_tutorials" is cloned in a configured catkin workspace

## Build

 - Clone repo (https://github.com/sjohns09/turtlebot_walker.git) to catkin workspace
 - In catkin workspace root directory, run catkin_make
 
 ```
 cd ~/catkin_ws/src
 git clone https://github.com/sjohns09/turtlebot_walker.git
 cd ~/catkin_ws
 catkin_make
 ```

## Run (Launch File)

 - In a new terminal source the setup file ($ source ./devel/setup.bash)
 - Call the turtlebot\_walker.launch launch file which requires a command line argument setting record\_bag = true or false.

```
roslaunch turtlebot_walker turtlebot_walker.launch record_bag:=false
```
## Bag File

To record a bag file set record\_bag = true when the launch file is ran. The bagfile will record for a maximum of 30 seconds.

To examine the recorded bag file run the command shown below:

```
rosbag info ~/.ros/turtlebot_walker.bag
```
To play back the bag file and show it working with the listener node:

  - Launch the turtlebot_gazebo worl sim
 
 ```
 roslaunch turtlebot_gazebo turtlebot_world.launch
 ```
 - In another terminal play the bag file
 
 ```
 rosbag play ~/.ros/turtlebot_walker.bag
 ```
The turtlebot will navigate the world even though the turtlebot_control node is not running.

## License

BSD 3-Clause (See LICENSE file for details)
