/** @file turtlebot_control.cpp
 * @brief TODO
 *
 * @author Samantha Johnson
 * @date November 21, 2017
 * @license BSD 3-Clause License
 * @copyright (c) 2017, Samantha Johnson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @details TODO
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "../include/Walker.h"




class SubAndPub {
 public:
  SubAndPub() {
    control_pub = n.advertise<geometry_msgs::Twist>(
        "cmd_vel_mux/input/teleop", 100);

    sensor_sub = n.subscribe("scan", 10, &SubAndPub::sensorCallback, this);
  }

  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg) {

    Walker turtleWalker;

    float angle_min = sensorMsg->angle_min;
    float angle_max = sensorMsg->angle_max;
    float angle_inc = sensorMsg->angle_increment;

    std::vector<float> ranges = sensorMsg->ranges;

    moveMsg = turtleWalker.walk_commands(angle_min, angle_max, angle_inc, ranges);

    control_pub.publish(moveMsg);
    ROS_INFO("Publishing Command");

  }

 private:
  ros::NodeHandle n;
  geometry_msgs::Twist moveMsg;
  ros::Publisher control_pub;
  ros::Subscriber sensor_sub;
};



int main(int argc, char **argv) {

  ros::init(argc, argv, "turtlebot_control");

  SubAndPub turtlebotWalker;

  ros::spin();


  return 0;
}

