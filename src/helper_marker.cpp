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

/* Author: Dave Coleman
   Desc:   Helper class for publishing arrow markers
*/

#ifndef RVIZ_VISUAL_TOOLS__ARROW_MARKER_H_
#define RVIZ_VISUAL_TOOLS__ARROW_MARKER_H_

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rviz_visual_tools
{

class RvizVisualTools;

class HelperMarker
{
public:
  visualization_msgs::Marker marker_;
  RvizVisualTools* rvt_;
  
}; // end class

class ArrowMarker : HelperMarker
{
public:

  ArrowMarker(const Eigen::Affine3d &pose, const RvizVisualTools* rvt)
  {
    rvt->convertPoseSafe(pose, marker_.pose);
    ArrowMarker(frame_id, id);
  }

  ArrowMarker(const geometry_msgs::Pose &pose, const RvizVisualTools* rvt)
  {
    marker_.pose = pose;
    ArrowMarker(frame_id, id);
  }

  ArrowMarker(const RvizVisualTools* rvt)
  {
    // Set the frame ID and timestamp.
    marker_.header.stamp = ros::Time::now();
    marker_.header.frame_id = frame_id;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker_.ns = "Arrow";
    marker_.id = id;

    // Other properites
    marker_.type = visualization_msgs::Marker::ARROW;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.lifetime = ros::Duration(0.0);
    marker_.color = getColor(rvt::BLUE);
    marker_.scale = getScale(rvt::REGULAR, true);
    marker_.scale.x = 0.1;  // overrides previous x scale specified
  }

  void setPose(const Eigen::Affine3d &pose) { marker_.pose = convertPose(pose); };
  //  void setColor(const rviz_visual_tools::colors &color) { marker_.color = getColor(color); };
  //  void setScale(const rviz_visual_tools::scales &scale) { marker_.scale = getScale(scale, true); }
  void setLength(double length) { marker_.scale.x = length; }
  void setID(int id) { marker_.id = id; }
  void setLifetime(double lifetime) {marker_.lifetime = lifetime; }
  void setAction(int action) {marker_.action = action; }
  void setNamespace(const std::string& ns) {marker_.ns = ns; }
  void setFrameID(const std::string& frame_id) {marker_.frame_id = frame_id; }
}; // end class

} // end namespace

#endif
