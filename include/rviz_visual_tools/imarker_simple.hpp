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
 *   * Neither the name of PickNik Consulting nor the names of its
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

/* Author: Dave Coleman
   Desc:   Use interactive markers in a C++ class
*/

#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/PoseStamped.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <Eigen/Geometry>

// C++
#include <string>

namespace rviz_visual_tools
{
typedef std::function<void(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&)> IMarkerCallback;

namespace
{
geometry_msgs::msg::Pose getIdentityPose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}
}

class IMarkerSimple
{
public:
  explicit IMarkerSimple(const std::string& name = "imarker", double scale = 0.2,
                         const geometry_msgs::msg::Pose& initial_pose = getIdentityPose());

  geometry_msgs::msg::Pose& getPose();

  void setPose(const Eigen::Isometry3d& pose);

  void setPose(const geometry_msgs::msg::Pose& pose);

  void iMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  void setIMarkerCallback(IMarkerCallback callback)
  {
    imarker_callback_ = callback;
  }

private:
  void sendUpdatedIMarkerPose();

  void make6DofMarker(const geometry_msgs::msg::Pose& pose, double scale = 0.2);

  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "imarker_simple";

  // A shared node handle
  ros::NodeHandle nh_;

  geometry_msgs::msg::Pose latest_pose_;

  // Interactive markers
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;

  // Interactive markers
  // interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::msg::InteractiveMarker int_marker_;

  // Hook to parent class
  IMarkerCallback imarker_callback_;
};  // end class

// Create std pointers for this class
typedef std::shared_ptr<IMarkerSimple> IMarkerSimplePtr;
typedef std::shared_ptr<const IMarkerSimple> IMarkerSimpleConstPtr;

}  // namespace rviz_visual_tools
