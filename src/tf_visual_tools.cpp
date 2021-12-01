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
*/

#include <rviz_visual_tools/tf_visual_tools.hpp>
#include <rclcpp/create_timer.hpp>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// TF
#include <tf2/convert.h>

// C++
#include <string>

namespace rviz_visual_tools
{
TFVisualTools::TFVisualTools(const rclcpp::Node::SharedPtr& node, double loop_hz)
  : node_base_interface_(node->get_node_base_interface())
  , timers_interface_(node->get_node_timers_interface())
  , clock_interface_(node->get_node_clock_interface())
  , logger_(node->get_node_logging_interface()->get_logger().get_child("tf_visual_tools"))
{
  rclcpp::Duration update_period = rclcpp::Duration::from_seconds(1.0 / loop_hz);

  // non_realtime_loop_ = nh_.createTimer(update_freq, &TFVisualTools::publishAllTransforms, this);
  non_realtime_loop_ =
      rclcpp::create_timer(node_base_interface_, timers_interface_, clock_interface_->get_clock(),
                           update_period, std::bind(&TFVisualTools::publishAllTransforms, this));
  // , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  RCLCPP_INFO(logger_, "TFVisualTools Ready.");
}

bool TFVisualTools::publishTransform(const Eigen::Isometry3d& transform,
                                     const std::string& from_frame, const std::string& to_frame)
{
  std::stringstream ss;
  ss << "Publishing transform from " << from_frame << " to " << to_frame;
  RCLCPP_DEBUG(logger_, ss.str().c_str());

  // Create transform msg
  geometry_msgs::msg::TransformStamped tf2_msg = tf2::eigenToTransform(transform);
  tf2_msg.header.stamp = clock_interface_->get_clock()->now();

  // Prevent TF_DENORMALIZED_QUATERNION errors in TF2 from happening.
  double quat_norm;

  // Normalizing the Quaternion
  quat_norm = 1 / sqrt(tf2_msg.transform.rotation.x * tf2_msg.transform.rotation.x +
                       tf2_msg.transform.rotation.y * tf2_msg.transform.rotation.y +
                       tf2_msg.transform.rotation.z * tf2_msg.transform.rotation.z +
                       tf2_msg.transform.rotation.w * tf2_msg.transform.rotation.w);
  tf2_msg.transform.rotation.x *= quat_norm;
  tf2_msg.transform.rotation.y *= quat_norm;
  tf2_msg.transform.rotation.z *= quat_norm;
  tf2_msg.transform.rotation.w *= quat_norm;

  tf2_msg.header.frame_id = from_frame;
  tf2_msg.child_frame_id = to_frame;

  // Check if this transform has already been added
  for (auto& transform : transforms_)
  {
    if (transform.child_frame_id == to_frame && transform.header.frame_id == from_frame)
    {
      transform.transform = tf2_msg.transform;
      return true;
    }
  }
  // This transform is unique, add
  transforms_.push_back(tf2_msg);

  return true;
}

void TFVisualTools::clearAllTransforms()
{
  transforms_.clear();
}

void TFVisualTools::publishAllTransforms()
{
  RCLCPP_DEBUG(logger_, "Publishing transforms");

  // Update timestamps
  for (auto& transform : transforms_)
  {
    transform.header.stamp = clock_interface_->get_clock()->now();
  }
  // Publish
  tf_broadcaster_->sendTransform(transforms_);
}

}  // namespace rviz_visual_tools
