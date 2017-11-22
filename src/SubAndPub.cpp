/** @file SubAndPub.cpp
 * @brief A navigation controller algorithm for the turtlebot
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
 * @details Creates the publisher and subscriber for the turtlebot topics
 * to allow for walker navigation
 */
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "Walker.h"
#include "SubAndPub.h"

SubAndPub::SubAndPub() {
  control_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",
                                                  100);

  sensor_sub = n.subscribe("scan", 10, &SubAndPub::sensorCallback, this);
}

void SubAndPub::sensorCallback(
    const sensor_msgs::LaserScan::ConstPtr& sensorMsg) {
  Walker turtleWalker;

  std::vector<float> ranges = sensorMsg->ranges;

  moveMsg = turtleWalker.walk_commands(ranges);

  control_pub.publish(moveMsg);
}

