/** @file Walker.cpp
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
 * @details Algorithm for determining whether the turtlebot is in close proximity
 * to an object. If the turtlebot is near an obstacle it instructs the turtlebot
 * to rotate, in hopes that a clear path will be discovered which will allow the
 * turtlebot to proceed moving forward.
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "Walker.h"


geometry_msgs::Twist Walker::walk_commands(std::vector<float> ranges) {
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
