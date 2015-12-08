/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helps debug and visualize transforms via the TF infrastructure
*/

#include <rviz_visual_tools/tf_visual_tools.h>

// TF
#include <eigen_conversions/eigen_msg.h>

// C++
#include <string>

namespace rviz_visual_tools
{
TFVisualTools::TFVisualTools()
{
  double loop_hz = 2;  // hz
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz);
  non_realtime_loop_ = nh_.createTimer(update_freq, &TFVisualTools::publishAllTransforms, this);

  ROS_INFO_STREAM_NAMED("tf_visual_tools", "TFVisualTools Ready.");
}

bool TFVisualTools::publishTransform(const Eigen::Affine3d& transform, const std::string& from_frame,
                                     const std::string& to_frame)
{
  ROS_DEBUG_STREAM_NAMED("tf_visual_tools", "Publishing transform from " << from_frame << " to " << to_frame);

  // Create transform msg
  geometry_msgs::TransformStamped tf2_msg;
  tf2_msg.header.stamp = ros::Time::now();
  tf::transformEigenToMsg(transform, tf2_msg.transform);
  tf2_msg.header.frame_id = from_frame;
  tf2_msg.child_frame_id = to_frame;

  // Check if this transform has already been added
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    if (transforms_[i].child_frame_id == to_frame && transforms_[i].header.frame_id == from_frame)
    {
      // ROS_WARN_STREAM_NAMED("tf_visual_tools", "This transform has already been added, updating");
      transforms_[i].transform = tf2_msg.transform;
      return true;
    }
  }
  // This transform is unique, add
  transforms_.push_back(tf2_msg);

  return true;
}

void TFVisualTools::publishAllTransforms(const ros::TimerEvent& e)
{
  ROS_DEBUG_STREAM_NAMED("tf_visual_tools", "Publishing transforms");

  // Update timestamps
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    transforms_[i].header.stamp = ros::Time::now();
  }
  // Publish
  tf_pub_.sendTransform(transforms_);
}

}  // namespace rviz_visual_tools
