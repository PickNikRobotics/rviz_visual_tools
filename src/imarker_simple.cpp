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

/* Author: Dave Coleman
   Desc:   Use interactive markers in a C++ class via the external python node
*/

#include <rviz_visual_tools/imarker_simple.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <string>

namespace rviz_visual_tools
{
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;

const std::string IMarkerSimple::IMARKER_NAME = "imarker_simple";

IMarkerSimple::IMarkerSimple(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node_base_interface,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_interface,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr& services_interface,
    const std::string& imarker_topic_name, double scale,
    const geometry_msgs::msg::Pose& initial_pose, const std::string& parent_frame,
    const rclcpp::QoS& update_pub_qos, const rclcpp::QoS& feedback_sub_qos)
  : node_base_interface_(node_base_interface)
  , clock_interface_(clock_interface)
  , logging_interface_(logging_interface)
  , topics_interface_(topics_interface)
  , services_interface_(services_interface)
  , logger_(logging_interface_->get_logger().get_child("imarker_simple"))
  , latest_pose_(initial_pose)
{
  // Create Marker Server
  std::string name_space = node_base_interface_->get_namespace();
  const std::string imarker_topic =
      (name_space == "/" ? imarker_topic_name : name_space + "/" + imarker_topic_name);

  imarker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      imarker_topic, node_base_interface_, clock_interface_, logging_interface_, topics_interface_,
      services_interface_, update_pub_qos, feedback_sub_qos);

  // ros::Duration(2.0).sleep();

  // Create imarker
  make6DofMarker(latest_pose_, scale, parent_frame);

  // Send imarker to Rviz
  imarker_server_->applyChanges();
}

geometry_msgs::msg::Pose& IMarkerSimple::getPose()
{
  return latest_pose_;
}

void IMarkerSimple::setPose(const Eigen::Isometry3d& pose)
{
  geometry_msgs::msg::Pose pose_msg;
  rviz_visual_tools::RvizVisualTools::convertPoseSafe(pose, pose_msg);
  setPose(pose_msg);
}

void IMarkerSimple::setPose(const geometry_msgs::msg::Pose& pose)
{
  latest_pose_ = pose;
  sendUpdatedIMarkerPose();
}

void IMarkerSimple::iMarkerCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  // Ignore if not pose update
  if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
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

void IMarkerSimple::make6DofMarker(const geometry_msgs::msg::Pose& pose, double scale,
                                   const std::string& parent_frame)
{
  std::stringstream ss;
  ss << "Making 6dof interactive marker named " << IMARKER_NAME;
  RCLCPP_INFO(logger_, ss.str().c_str());

  int_marker_.header.frame_id = parent_frame;
  int_marker_.pose = pose;
  int_marker_.scale = scale;

  int_marker_.name = IMARKER_NAME;

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

  imarker_server_->insert(int_marker_,
                          std::bind(&IMarkerSimple::iMarkerCallback, this, std::placeholders::_1));
  imarker_server_->applyChanges();
}

}  // namespace rviz_visual_tools
