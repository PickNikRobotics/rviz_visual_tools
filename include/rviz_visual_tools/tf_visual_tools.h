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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helps debug and visualize transforms via the TF infrastructure
   Note:   We shouldn't have to publish the transforms at interval since they are static, but we do
   because of https://github.com/ros/geometry_experimental/issues/108
*/

#ifndef RVIZ_VISUAL_TOOLS_TF_VISUAL_TOOLS_H
#define RVIZ_VISUAL_TOOLS_TF_VISUAL_TOOLS_H

// C++
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

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
   * \param loop_hz - how often tf is published
   */
  explicit TFVisualTools(double loop_hz = 2);

  /**
   * \brief Visualize transforms in Rviz, etc
   * \return true on success
   */
  bool publishTransform(const Eigen::Affine3d& transform, const std::string& from_frame, const std::string& to_frame);

  /**
   * \brief At a certain frequency update the tf transforms that we are tracking
   *        This is called internally by a clock, you should not need to use this
   *        TODO: make private in next release?
   */
  void publishAllTransforms(const ros::TimerEvent& e);

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Send tf messages
  tf2_ros::TransformBroadcaster tf_pub_;

  // Separate thread to publish transforms
  ros::Timer non_realtime_loop_;

  // Collect the transfroms
  std::vector<geometry_msgs::TransformStamped> transforms_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TFVisualTools> TFVisualToolsPtr;
typedef boost::shared_ptr<const TFVisualTools> TFVisualToolsConstPtr;

}  // namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS_TF_VISUAL_TOOLS_H
