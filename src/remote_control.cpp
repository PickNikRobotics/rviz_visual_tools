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
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface)
  : node_base_interface_(node_base_interface)
  , topics_interface_(topics_interface)
  , logger_(logging_interface->get_logger().get_child("remote_control"))
{
  // Subscribe to Rviz Dashboard
  std::string rviz_dashboard_topic = "/rviz_visual_tools_gui";
  rviz_dashboard_sub_ = rclcpp::create_subscription<sensor_msgs::msg::Joy>(
      topics_interface_, rviz_dashboard_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&RemoteControl::rvizDashboardCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "RemoteControl Ready.");
}

void RemoteControl::rvizDashboardCallback(
    const sensor_msgs::msg::Joy::ConstSharedPtr msg)  // NOLINT
{
  if (msg->buttons.size() > 1 && msg->buttons[1] != 0)
  {
    setReadyForNextStep();
  }
  else if (msg->buttons.size() > 2 && msg->buttons[2] != 0)
  {
    setAutonomous();
  }
  else if (msg->buttons.size() > 3 && msg->buttons[3] != 0)
  {
    setFullAutonomous();
  }
  else if (msg->buttons.size() > 4 && msg->buttons[4] != 0)
  {
    stopAllAutonomous();
  }
  else
  {
    RCLCPP_ERROR(logger_, "Unknown input button");
  }
}

void RemoteControl::setReadyForNextStep()
{
  {
    std::lock_guard<std::mutex> wait_lock(mutex_);
    if (is_waiting_)
    {
      next_step_ready_ = true;
    }
  }
  wait_next_step_.notify_all();
}

void RemoteControl::setAutonomous()
{
  autonomous_ = true;
}

void RemoteControl::setFullAutonomous()
{
  full_autonomous_ = true;
  setAutonomous();
}

void RemoteControl::stopAllAutonomous()
{
  autonomous_ = false;
  full_autonomous_ = false;
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
  return (waitForNextStepCommon(caption, autonomous_));
}

bool RemoteControl::waitForNextFullStep(const std::string& caption)
{
  return (waitForNextStepCommon(caption, full_autonomous_));
}

bool RemoteControl::waitForNextStepCommon(const std::string& caption, bool autonomous)
{
  std::unique_lock<std::mutex> wait_lock(mutex_);

  // Check if we really need to wait
  if (next_step_ready_ || autonomous || !rclcpp::ok())
  {
    return true;
  }

  // Show message
  {
    RCLCPP_INFO_STREAM(logger_, CONSOLE_COLOR_CYAN << "Waiting to continue: " << caption
                                                   << CONSOLE_COLOR_RESET);
  }

  if (displayWaitingState_)
  {
    displayWaitingState_(true);
  }

  is_waiting_ = true;

  // Wait until next step is ready
  wait_next_step_.wait(wait_lock, [this]() { return next_step_ready_ || !rclcpp::ok(); });
  RCLCPP_INFO_STREAM(logger_, CONSOLE_COLOR_CYAN << "... continuing" << CONSOLE_COLOR_RESET);

  if (displayWaitingState_)
  {
    displayWaitingState_(false);
  }

  next_step_ready_ = false;
  is_waiting_ = false;

  return true;
}

}  // namespace rviz_visual_tools
