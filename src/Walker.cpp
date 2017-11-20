/*
 * walker.cpp
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
#include "../include/Walker.h"


geometry_msgs::Twist Walker::walk_commands(float angleMin, float angleMax, float inc,
                           std::vector<float> ranges) {
  // If any of the values in range are < 1.5 rotate until not and then continue

  geometry_msgs::Twist moveMsg;
  bool straight = true;

  for (auto r : ranges) {
    if (r < 0.7) {
      moveMsg.linear.x = 0.0;
      moveMsg.linear.y = 0.0;
      moveMsg.linear.z = 0.0;

      moveMsg.angular.x = 0.0;
      moveMsg.angular.y = 0.0;
      moveMsg.angular.z = -0.2;
      straight = false;
      ROS_INFO("Rotating");
      break;
    }
  }

  if (straight == true) {
    moveMsg.linear.x = 0.2;
    moveMsg.linear.y = 0.0;
    moveMsg.linear.z = 0.0;

    moveMsg.angular.x = 0.0;
    moveMsg.angular.y = 0.0;
    moveMsg.angular.z = 0.0;
    ROS_INFO("Straight");
  }

  return moveMsg;
}
