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
 *   * Neither the name of the PickNik Consulting nor the names of its
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

/* Author: Dave Coleman <dave@picknik.ai>
   Desc:   Contains all hooks for debug interface
*/

// C++
#include <string>

#include <rviz_visual_tools/remote_control.hpp>

#define CONSOLE_COLOR_RESET "\033[0m"
#define CONSOLE_COLOR_CYAN "\033[96m"
#define CONSOLE_COLOR_BROWN "\033[93m"

namespace rviz_visual_tools
{
using namespace std::chrono_literals;

/**
 * \brief Constructor
 */
RemoteControl::RemoteControl(
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface)
  : topics_interface_(topics_interface)
  , logger_(logging_interface->get_logger().get_child("remote_control"))
  , next_step_ready_(nullptr)
{
  // Subscribe to Rviz Dashboard
  std::string rviz_dashboard_topic = "/rviz_visual_tools_gui";
  const std::size_t button_queue_size = 10;
  const rclcpp::QoS update_sub_qos(button_queue_size);
  rviz_dashboard_sub_ = rclcpp::create_subscription<sensor_msgs::msg::Joy>(
    topics_interface_,
    rviz_dashboard_topic,
    update_sub_qos,
    std::bind(&RemoteControl::rvizDashboardCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "RemoteControl Ready.");
}

void RemoteControl::rvizDashboardCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[1] != 0)
  {
    setReadyForNextStep();
  }
  else if (msg->buttons[2] != 0)
  {
    setAutonomous();
  }
  else if (msg->buttons[3] != 0)
  {
    setFullAutonomous();
  }
  else if (msg->buttons[4] != 0)
  {
    setStop();
  }
  else
  {
    RCLCPP_ERROR(logger_, "Unknown input button");
  }
}

bool RemoteControl::setReadyForNextStep()
{
  stop_ = false;

  if (next_step_ready_)
  {
    next_step_ready_->set_value(true);
  }
  return true;
}

void RemoteControl::setAutonomous(bool autonomous)
{
  autonomous_ = autonomous;
  if (next_step_ready_)
  {
    next_step_ready_->set_value(true);
  }
  stop_ = false;
}

void RemoteControl::setFullAutonomous(bool autonomous)
{
  full_autonomous_ = autonomous;
  if (full_autonomous_)
    setAutonomous(full_autonomous_);
}

void RemoteControl::setStop(bool stop)
{
  stop_ = stop;
  if (stop)
  {
    autonomous_ = false;
    full_autonomous_ = false;
  }
}

bool RemoteControl::getStop()
{
  return stop_;
}

bool RemoteControl::getAutonomous()
{
  return autonomous_;
}

bool RemoteControl::getFullAutonomous()
{
  return full_autonomous_;
}

bool RemoteControl::waitForNextStep(const std::string& caption)
{
  // Check if we really need to wait
  if (next_step_ready_ || autonomous_ || !rclcpp::ok())
  {
    return true;
  }

  // Show message
  std::cout << std::endl;
  std::cout << CONSOLE_COLOR_CYAN << "Waiting to continue: " << caption << CONSOLE_COLOR_RESET << std::flush;

  if (displayWaitingState_)
  {
    displayWaitingState_(true);
  }

  next_step_ready_ = std::make_shared<std::promise<bool>>();
  // Wait until next step is ready
  std::future<bool> future_next_step_ready = next_step_ready_->get_future();
  std::future_status status;
  while (!autonomous_ && rclcpp::ok() && future_next_step_ready.valid())
  {
    status = future_next_step_ready.wait_for(std::chrono::milliseconds(250));
    if (status == std::future_status::ready)
    {
      break;
    }
  }
  if (!future_next_step_ready.valid())
  {
    RCLCPP_ERROR(logger_, "Future became invalid");
    return false;
  }
  if (!rclcpp::ok())
  {
    exit(0);
  }
  next_step_ready_.reset();

  std::cout << CONSOLE_COLOR_CYAN << "... continuing" << CONSOLE_COLOR_RESET << std::endl;

  if (displayWaitingState_)
  {
    displayWaitingState_(false);
  }
  return true;
}

bool RemoteControl::waitForNextFullStep(const std::string& caption)
{
  // Check if we really need to wait
  if (next_step_ready_ || full_autonomous_ || !rclcpp::ok())
  {
    return true;
  }

  // Show message
  std::cout << std::endl;
  std::cout << CONSOLE_COLOR_CYAN << "Waiting to continue: " << caption << CONSOLE_COLOR_RESET << std::flush;

  if (displayWaitingState_)
  {
    displayWaitingState_(true);
  }

  next_step_ready_ = std::make_shared<std::promise<bool>>();
  // Wait until next step is ready
  std::future<bool> future_next_step_ready = next_step_ready_->get_future();
  std::future_status status;
  while (!full_autonomous_ && rclcpp::ok() && future_next_step_ready.valid())
  {
    status = future_next_step_ready.wait_for(std::chrono::milliseconds(250));
    if (status == std::future_status::ready)
    {
      break;
    }
  }
  if (!future_next_step_ready.valid())
  {
    RCLCPP_ERROR(logger_, "Future became invalid");
    return false;
  }
  if (!rclcpp::ok())
  {
    exit(0);
  }
  next_step_ready_.reset();

  std::cout << CONSOLE_COLOR_CYAN << "... continuing" << CONSOLE_COLOR_RESET << std::endl;

  if (displayWaitingState_)
  {
    displayWaitingState_(false);
  }
  return true;
}

}  // namespace rviz_visual_tools
