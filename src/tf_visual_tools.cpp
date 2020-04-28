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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
   Desc:   Helps debug and visualize transforms via the TF infrastructure
*/

#include <rviz_visual_tools/tf_visual_tools.h>

// TF2
#include <tf2_eigen/tf2_eigen.h>

// C++
#include <string>

namespace rviz_visual_tools
{
TFVisualTools::TFVisualTools(double loop_hz)
{
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz);
  non_realtime_loop_ = nh_.createTimer(update_freq, &TFVisualTools::publishAllTransforms, this);

  ROS_INFO_STREAM_NAMED("tf_visual_tools", "TFVisualTools Ready.");
}

bool TFVisualTools::publishTransform(const Eigen::Isometry3d& transform, const std::string& from_frame,
                                     const std::string& to_frame)
{
  ROS_DEBUG_STREAM_NAMED("tf_visual_tools", "Publishing transform from " << from_frame << " to " << to_frame);

  // Create transform msg
  geometry_msgs::TransformStamped tf2_msg = tf2::eigenToTransform(transform);
  tf2_msg.header.stamp = ros::Time::now();

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
      // ROS_WARN_STREAM_NAMED("tf_visual_tools", "This transform has already been added, updating");
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

void TFVisualTools::publishAllTransforms(const ros::TimerEvent& /*e*/)
{
  ROS_DEBUG_STREAM_NAMED("tf_visual_tools", "Publishing transforms");

  // Update timestamps
  for (auto& transform : transforms_)
  {
    transform.header.stamp = ros::Time::now();
  }
  // Publish
  tf_pub_.sendTransform(transforms_);
}

}  // namespace rviz_visual_tools
