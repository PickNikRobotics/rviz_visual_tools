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
   Desc:   Helps debug and visualize transforms via the TF infrastructure
   Note:   We shouldn't have to publish the transforms at interval since they are static, but we do
   because of https://github.com/ros/geometry_experimental/issues/108
*/

#pragma once

// C++
#include <vector>
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Eigen
#include <Eigen/Geometry>

#include <tf2_ros/transform_broadcaster.h>

// namespace tf2_ros
// {
// class StaticTransformBroadcaster;
// };

namespace rviz_visual_tools
{
class TFVisualTools
{
public:
  /**
   * \brief Constructor
   * \param node - a pointer to a rclcpp::Node
   * \param loop_hz - how often tf is published
   */
  TFVisualTools(const rclcpp::Node::SharedPtr& node, double loop_hz = 2);
  /**
   * \brief Visualize transforms in Rviz, etc
   * \return true on success
   */
  bool publishTransform(const Eigen::Isometry3d& transform, const std::string& from_frame,
                        const std::string& to_frame);

  /**
   * \brief Clear all transforms
   */
  void clearAllTransforms();

private:
  /**
   * \brief At a certain frequency update the tf transforms that we are tracking
   *        This is called internally by a clock, you should not need to use this
   */
  void publishAllTransforms();

  // Node Interfaces
  // rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  // rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface_;
  // rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  rclcpp::Logger logger_;

  // Send tf messages
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Separate thread to publish transforms
  rclcpp::TimerBase::SharedPtr non_realtime_loop_;

  // Collect the transfroms
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};  // end class

// Create boost pointers for this class
typedef std::shared_ptr<TFVisualTools> TFVisualToolsPtr;
typedef std::shared_ptr<const TFVisualTools> TFVisualToolsConstPtr;

}  // namespace rviz_visual_tools
