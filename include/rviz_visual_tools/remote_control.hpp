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
   Desc:   Tool for creating break points and user verification points through
           manipulation pipelines or other live robotic tool.
           Think GDB for robots, or like, a state machine.
*/

#pragma once

// C++
#include <chrono>
#include <string>
#include <mutex>
#include <condition_variable>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <utility>

namespace rviz_visual_tools
{
typedef std::function<void(bool)> DisplayWaitingState;

class RemoteControl
{
public:
  /**
   * \brief Constructor for passing in NodeOptions
   * \param executor - An rclcpp executor
   * \param node - A node
   */
  [[deprecated("RemoteControl(const rclcpp::Executor::SharedPtr&, const rclcpp::NodeOptions&) is "
               "deprecated Use RemoteControl(const rclcpp::NodeOptions&) constructor "
               "instead")]] explicit RemoteControl(const rclcpp::Executor::SharedPtr& executor,
                                                   const rclcpp::NodeOptions& node_options)
    : RemoteControl(std::make_shared<rclcpp::Node>("remote_control", node_options))
  {
  }

  /**
   * \brief Constructor for passing in a Node
   * \param node - A node
   */
  template <typename NodePtr>
  explicit RemoteControl(NodePtr node)
    : RemoteControl(node->get_node_base_interface(), node->get_node_topics_interface(),
                    node->get_node_logging_interface())
  {
  }

  /**
   * \brief Constructor for passing in Node components for a node that is already added to an
   * executor.
   * \param topics_iterface - An interface for publishing and subscribing to topics
   * \param logging_iterface - An interface for publishing to the rosconsole
   */
  explicit RemoteControl(
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node_base_interface,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface);

  /**
   * \brief Callback from ROS topic
   */
  void rvizDashboardCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);  // NOLINT

  /**
   * \brief Get the autonomous mode
   * \return true if is in autonomous mode
   */
  bool getAutonomous();
  bool getFullAutonomous();

  /**
   * \brief See if we are in stop mode
   */
  bool getStop();

  /**
   * \brief Wait until user presses a button
   * \return true on success
   */
  bool waitForNextStep(const std::string& caption = "go to next step");
  bool waitForNextFullStep(const std::string& caption = "go to next full step");

  void setDisplayWaitingState(DisplayWaitingState displayWaitingState)
  {
    displayWaitingState_ = std::move(displayWaitingState);
  }

private:
  /**
   * \brief Step to next step
   * \return true on success
   */
  void setReadyForNextStep();

  /**
   * \brief Enable autonomous mode
   */
  void setAutonomous();
  void setFullAutonomous();

  /**
   * \brief Stop autonomous and full autonomous modes
   */
  void stopAllAutonomous();

  /**
   * \brief Wait until user presses a button
   * \return true on success
   */
  bool waitForNextStepCommon(const std::string& caption, bool autonomous);

  // Node Interfaces
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  rclcpp::Logger logger_;

  // Short name for this class
  std::string name_ = "remote_control";

  // Input
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rviz_dashboard_sub_;

  bool is_waiting_ = false;
  bool next_step_ready_ = false;
  bool autonomous_ = false;
  bool full_autonomous_ = false;

  std::mutex mutex_;
  std::condition_variable wait_next_step_;

  // Callback to visualize waiting state
  DisplayWaitingState displayWaitingState_;
};  // end class

typedef std::shared_ptr<RemoteControl> RemoteControlPtr;
typedef std::shared_ptr<const RemoteControl> RemoteControlConstPtr;

}  // namespace rviz_visual_tools
