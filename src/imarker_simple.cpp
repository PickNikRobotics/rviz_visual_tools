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
   Desc:   Use interactive markers in a C++ class via the external python node
*/

#include <rviz_visual_tools/imarker_simple.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <string>

namespace rviz_visual_tools
{
IMarkerSimple::IMarkerSimple(const std::string& name, double scale, const geometry_msgs::Pose& initial_pose,
                             const std::string& parent_frame)
  : nh_("~"), latest_pose_(initial_pose)
{
  // Create Marker Server
  const std::string imarker_topic = nh_.getNamespace() + "/" + name;
  imarker_server_.reset(new interactive_markers::InteractiveMarkerServer(imarker_topic, "", false));

  // ros::Duration(2.0).sleep();

  // Create imarker
  make6DofMarker(latest_pose_, scale, parent_frame);

  // Send imarker to Rviz
  imarker_server_->applyChanges();
}

geometry_msgs::Pose& IMarkerSimple::getPose()
{
  return latest_pose_;
}

void IMarkerSimple::setPose(const Eigen::Isometry3d& pose)
{
  geometry_msgs::Pose pose_msg;
  rviz_visual_tools::RvizVisualTools::convertPoseSafe(pose, pose_msg);
  setPose(pose_msg);
}

void IMarkerSimple::setPose(const geometry_msgs::Pose& pose)
{
  latest_pose_ = pose;
  sendUpdatedIMarkerPose();
}

void IMarkerSimple::iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Ignore if not pose update
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    return;
  }

  latest_pose_ = feedback->pose;

  // Redirect to base class
  if (imarker_callback_)
    imarker_callback_(feedback);
}

void IMarkerSimple::sendUpdatedIMarkerPose()
{
  imarker_server_->setPose(int_marker_.name, latest_pose_);
  imarker_server_->applyChanges();
}

void IMarkerSimple::make6DofMarker(const geometry_msgs::Pose& pose, double scale, const std::string& parent_frame)
{
  ROS_INFO_STREAM_NAMED(name_, "Making 6dof interactive marker named " << name_);

  int_marker_.header.frame_id = parent_frame;
  int_marker_.pose = pose;
  int_marker_.scale = scale;

  int_marker_.name = name_;

  // int_marker_.controls[0].interaction_mode = InteractiveMarkerControl::MENU;

  InteractiveMarkerControl control;
  control.orientation.w = 0.7071;
  control.orientation.x = 0.7071;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 0.7071;
  control.orientation.x = 0;
  control.orientation.y = 0.7071;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 0.7071;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.7071;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  imarker_server_->insert(int_marker_);
  imarker_server_->setCallback(int_marker_.name, boost::bind(&IMarkerSimple::iMarkerCallback, this, _1));
  // menu_handler_.apply(*imarker_server_, int_marker_.name);
}

}  // namespace rviz_visual_tools
