// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Dave Coleman
   Desc:   Object for wrapping remote control functionality
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace rviz_visual_tools
{
class RemoteReciever : public rclcpp::Node
{
public:
  RemoteReciever(const std::string& nodeName) : rclcpp::Node(nodeName)
  {
    joy_publisher_ =
        this->create_publisher<sensor_msgs::msg::Joy>("/rviz_visual_tools_gui", rclcpp::QoS(100));
  }

  void publishNext()
  {
    RCLCPP_DEBUG(this->get_logger(), "Next");
    sensor_msgs::msg::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[1] = 1;
    joy_publisher_->publish(msg);
  }

  void publishContinue()
  {
    RCLCPP_DEBUG(this->get_logger(), "Continue");
    sensor_msgs::msg::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[2] = 1;
    joy_publisher_->publish(msg);
  }

  void publishBreak()
  {
    RCLCPP_DEBUG(this->get_logger(), "Break");
    sensor_msgs::msg::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[3] = 1;
    joy_publisher_->publish(msg);
  }

  void publishStop()
  {
    RCLCPP_DEBUG(this->get_logger(), "Stop");
    sensor_msgs::msg::Joy msg;
    msg.buttons.resize(9);
    msg.buttons[4] = 1;
    joy_publisher_->publish(msg);
  }

protected:
  // The ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
};

}  // end namespace rviz_visual_tools
