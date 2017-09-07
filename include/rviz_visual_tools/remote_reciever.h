/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Object for wrapping remote control functionality
*/

#ifndef RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
#define RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace rviz_visual_tools
{
class RemoteReciever
{
public:
  RemoteReciever()
  {
    joy_publisher_ = nh_.advertise<sensor_msgs::Joy>("/rviz_visual_tools_gui", 1);
  }

  void publishNext()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Next");
    sensor_msgs::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[1] = 1;
    joy_publisher_.publish(msg);
  }

  void publishContinue()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Continue");
    sensor_msgs::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[2] = 1;
    joy_publisher_.publish(msg);
  }

  void publishBreak()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Break (not implemented yet)");

    sensor_msgs::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[3] = 1;
    joy_publisher_.publish(msg);
  }

  void publishStop()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Stop (not implemented yet)");

    sensor_msgs::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[4] = 1;
    joy_publisher_.publish(msg);
  }

protected:
  // The ROS publishers
  ros::Publisher joy_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
