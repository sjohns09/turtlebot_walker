/*
 * Walker.h
 *
 *  Created on: Nov 19, 2017
 *      Author: sammie
 */
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#ifndef TURTLEBOT_WALKER_SRC_WALKER_H_
#define TURTLEBOT_WALKER_SRC_WALKER_H_

class Walker {
 public:
  geometry_msgs::Twist walk_commands(float angleMin, float angleMax, float inc, std::vector<float> ranges);
};

#endif /* TURTLEBOT_WALKER_SRC_WALKER_H_ */
