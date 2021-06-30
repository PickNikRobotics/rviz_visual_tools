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
    const rclcpp::Executor::SharedPtr& executor,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface)
  : executor_(executor)
  , node_base_interface_(node_base_interface)
  , topics_interface_(topics_interface)
  , logger_(logging_interface->get_logger().get_child("remote_control"))
  , next_step_ready_(nullptr)
{
  // Subscribe to Rviz Dashboard
  std::string rviz_dashboard_topic = "/rviz_visual_tools_gui";
  const std::size_t button_queue_size = 10;
  const rclcpp::QoS update_sub_qos(button_queue_size);
  rviz_dashboard_sub_ = rclcpp::create_subscription<sensor_msgs::msg::Joy>(
      topics_interface_, rviz_dashboard_topic, update_sub_qos,
      std::bind(&RemoteControl::rvizDashboardCallback, this, std::placeholders::_1));

  if (!node_base_interface_->get_associated_with_executor_atomic().load())
  {
    executor_->add_node(node_base_interface_);
  }
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
  if (next_step_ready_ != nullptr)
  {
    next_step_ready_->set_value();
  }
}

void RemoteControl::setAutonomous()
{
  autonomous_ = true;
  if (next_step_ready_ != nullptr)
  {
    next_step_ready_->set_value();
  }
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
  // Check if we really need to wait
  if (next_step_ready_ != nullptr || autonomous || !rclcpp::ok())
  {
    return true;
  }

  // Show message
  {
    std::stringstream ss;
    ss << CONSOLE_COLOR_CYAN << "Waiting to continue: " << caption << CONSOLE_COLOR_RESET;
    RCLCPP_ERROR(logger_, ss.str().c_str());
  }

  if (displayWaitingState_)
  {
    displayWaitingState_(true);
  }

  next_step_ready_ = std::make_unique<std::promise<void>>();
  // Wait until next step is ready
  std::shared_future<void> future_next_step_ready = next_step_ready_->get_future();
  while (!autonomous && rclcpp::ok())
  {
    /* TODO(mlautman): Pending https://github.com/ros2/rclcpp/issues/520 the only way to spin is by
     * having access to the executor. Thus, the remote control must have an executor. Once the issue
     * has been resolved, it would be nice to remove the executor from this class */
    rclcpp::FutureReturnCode status = executor_->spin_until_future_complete(
        future_next_step_ready, std::chrono::milliseconds(125));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
      break;
    }
    else if (status == rclcpp::FutureReturnCode::INTERRUPTED)
    {
      RCLCPP_INFO(logger_, "Spinning was interrupted.");
      next_step_ready_ = nullptr;
      if (!rclcpp::ok())
      {
        exit(0);
      }
    }
  }

  {
    std::stringstream ss;
    ss << CONSOLE_COLOR_CYAN << "... continuing" << CONSOLE_COLOR_RESET;
    RCLCPP_ERROR(logger_, ss.str().c_str());
  }

  if (displayWaitingState_)
  {
    displayWaitingState_(false);
  }

  next_step_ready_ = nullptr;

  return true;
}

}  // namespace rviz_visual_tools
