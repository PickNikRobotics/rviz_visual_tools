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

/* Author: Dave Coleman <dave@dav.ee>, Andy McEvoy
   Desc:   Helper functions for displaying basic shape markers in Rviz
*/

#include <rviz_visual_tools/rviz_visual_tools.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// C++
#include <utility>
#include <string>
#include <vector>
#include <set>
#include <cmath>  // for random poses

namespace rviz_visual_tools
{
RvizVisualTools::RvizVisualTools(const std::string &base_frame, const std::string &marker_topic)
  : nh_("~")
  , name_("visual_tools")
  , marker_topic_(marker_topic)
  , base_frame_(base_frame)
  , batch_publishing_enabled_(false)
  , pub_rviz_markers_connected_(false)
  , pub_rviz_markers_waited_(false)
  , psychedelic_mode_(false)
{
  initialize();
  double bug;
}

void RvizVisualTools::initialize()
{
  marker_lifetime_ = ros::Duration(0.0);  // 0 - unlimited
  alpha_ = 1.0;
  global_scale_ = 1.0;
  // Cache the reusable markers
  loadRvizMarkers();
}

bool RvizVisualTools::deleteAllMarkers()
{
  // Helper for publishing rviz markers
  return publishMarker(reset_marker_);
}

void RvizVisualTools::resetMarkerCounts()
{
  arrow_marker_.id++;
  sphere_marker_.id++;
  block_marker_.id++;
  cylinder_marker_.id++;
  mesh_marker_.id++;
  text_marker_.id++;
  cuboid_marker_.id++;
  line_strip_marker_.id++;
  line_list_marker_.id++;
  spheres_marker_.id++;
  triangle_marker_.id++;
}

bool RvizVisualTools::loadRvizMarkers()
{
  // Load reset marker -------------------------------------------------
  reset_marker_.header.frame_id = base_frame_;
  reset_marker_.header.stamp = ros::Time();
  reset_marker_.action = 3;  // TODO(davetcoleman): In ROS-J set to visualization_msgs::Marker::DELETEALL;

  // Load arrow ----------------------------------------------------

  arrow_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  arrow_marker_.ns = "Arrow";
  // Set the marker type.
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  // Set the marker action.  Options are ADD and DELETE
  arrow_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  arrow_marker_.lifetime = marker_lifetime_;

  // Load cuboid ----------------------------------------------------

  cuboid_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  cuboid_marker_.ns = "Cuboid";
  // Set the marker type.
  cuboid_marker_.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  cuboid_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  cuboid_marker_.lifetime = marker_lifetime_;

  // Load line ----------------------------------------------------

  line_strip_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  line_strip_marker_.ns = "Line";
  // Set the marker type.
  line_strip_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  // Set the marker action.  Options are ADD and DELETE
  line_strip_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_strip_marker_.lifetime = marker_lifetime_;

  // Load path ----------------------------------------------------

  line_list_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  line_list_marker_.ns = "Line_List";
  // Set the marker type.
  line_list_marker_.type = visualization_msgs::Marker::LINE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  line_list_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_list_marker_.lifetime = marker_lifetime_;
  // Constants
  line_list_marker_.pose.position.x = 0.0;
  line_list_marker_.pose.position.y = 0.0;
  line_list_marker_.pose.position.z = 0.0;

  line_list_marker_.pose.orientation.x = 0.0;
  line_list_marker_.pose.orientation.y = 0.0;
  line_list_marker_.pose.orientation.z = 0.0;
  line_list_marker_.pose.orientation.w = 1.0;

  // Load sphers ----------------------------------------------------

  spheres_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  spheres_marker_.ns = "Spheres";
  // Set the marker type.
  spheres_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  spheres_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  spheres_marker_.lifetime = marker_lifetime_;
  // Constants
  spheres_marker_.pose.position.x = 0.0;
  spheres_marker_.pose.position.y = 0.0;
  spheres_marker_.pose.position.z = 0.0;

  spheres_marker_.pose.orientation.x = 0.0;
  spheres_marker_.pose.orientation.y = 0.0;
  spheres_marker_.pose.orientation.z = 0.0;
  spheres_marker_.pose.orientation.w = 1.0;

  // Load Block ----------------------------------------------------
  block_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  block_marker_.ns = "Block";
  // Set the marker action.  Options are ADD and DELETE
  block_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  block_marker_.type = visualization_msgs::Marker::CUBE;
  // Lifetime
  block_marker_.lifetime = marker_lifetime_;

  // Load Cylinder ----------------------------------------------------
  cylinder_marker_.header.frame_id = base_frame_;
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  // Lifetime
  cylinder_marker_.lifetime = marker_lifetime_;

  // Load Mesh ----------------------------------------------------
  mesh_marker_.header.frame_id = base_frame_;

  // Set the marker action.  Options are ADD and DELETE
  mesh_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  mesh_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  // Lifetime
  mesh_marker_.lifetime = marker_lifetime_;

  // Load Sphere -------------------------------------------------
  sphere_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  sphere_marker_.ns = "Sphere";
  // Set the marker type.
  sphere_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  sphere_marker_.action = visualization_msgs::Marker::ADD;
  // Marker group position and orientation
  sphere_marker_.pose.position.x = 0;
  sphere_marker_.pose.position.y = 0;
  sphere_marker_.pose.position.z = 0;
  sphere_marker_.pose.orientation.x = 0.0;
  sphere_marker_.pose.orientation.y = 0.0;
  sphere_marker_.pose.orientation.z = 0.0;
  sphere_marker_.pose.orientation.w = 1.0;
  // Create a sphere point
  geometry_msgs::Point point_a;
  // Add the point pair to the line message
  sphere_marker_.points.push_back(point_a);
  sphere_marker_.colors.push_back(getColor(BLUE));
  // Lifetime
  sphere_marker_.lifetime = marker_lifetime_;

  // Load Text ----------------------------------------------------
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  text_marker_.ns = "Text";
  // Set the marker action.  Options are ADD and DELETE
  text_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // Lifetime
  text_marker_.lifetime = marker_lifetime_;

  // Load Triangle List -------------------------------------------
  // Set the namespace and id for this marker. This serves to create a unique ID
  triangle_marker_.header.frame_id = base_frame_;
  triangle_marker_.ns = "Triangle";
  // Set the marker action. Options are ADD and DELETE
  triangle_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type
  triangle_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
  // Lifetime
  triangle_marker_.lifetime = marker_lifetime_;

  return true;
}

void RvizVisualTools::loadMarkerPub(bool wait_for_subscriber, bool latched)
{
  if (pub_rviz_markers_)
    return;

  // Rviz marker publisher
  pub_rviz_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10, latched);
  ROS_DEBUG_STREAM_NAMED(name_, "Publishing Rviz markers on topic " << pub_rviz_markers_.getTopic());

  if (wait_for_subscriber)
    waitForSubscriber(pub_rviz_markers_);
}

bool RvizVisualTools::waitForSubscriber(const ros::Publisher &pub, const double &wait_time)
{
  // Will wait at most this amount of time
  ros::Time max_time(ros::Time::now() + ros::Duration(wait_time));

  // This is wrong. It returns only the number of subscribers that have already
  // established their direct connections to this publisher
  int num_existing_subscribers = pub.getNumSubscribers();

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  // Wait for subsriber
  while (num_existing_subscribers == 0)
  {
    // Check if timed out
    if (ros::Time::now() > max_time)
    {
      ROS_WARN_STREAM_NAMED(name_, "Topic '" << pub.getTopic() << "' unable to connect to any subscribers within "
                                             << wait_time << " sec. It is possible initially published visual messages "
                                                             "will be lost.");
      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    // Check again
    num_existing_subscribers = pub.getNumSubscribers();
    // std::cout << "num_existing_subscribers " << num_existing_subscribers << std::endl;
  }
  pub_rviz_markers_connected_ = true;

  return true;
}

void RvizVisualTools::setFloorToBaseHeight(double floor_to_base_height)
{
  ROS_WARN_STREAM_NAMED(name_, "Deperecated function");
}

void RvizVisualTools::setLifetime(double lifetime)
{
  marker_lifetime_ = ros::Duration(lifetime);

  // Update cached markers
  arrow_marker_.lifetime = marker_lifetime_;
  cuboid_marker_.lifetime = marker_lifetime_;
  line_strip_marker_.lifetime = marker_lifetime_;
  sphere_marker_.lifetime = marker_lifetime_;
  block_marker_.lifetime = marker_lifetime_;
  mesh_marker_.lifetime = marker_lifetime_;
  cylinder_marker_.lifetime = marker_lifetime_;
  text_marker_.lifetime = marker_lifetime_;
}

colors RvizVisualTools::getRandColor()
{
  std::vector<colors> all_colors;

  all_colors.push_back(RED);
  all_colors.push_back(GREEN);
  all_colors.push_back(BLUE);
  all_colors.push_back(GREY);
  all_colors.push_back(DARK_GREY);
  all_colors.push_back(WHITE);
  all_colors.push_back(ORANGE);
  // all_colors.push_back(BLACK);
  all_colors.push_back(YELLOW);
  all_colors.push_back(BROWN);
  all_colors.push_back(PINK);
  all_colors.push_back(LIME_GREEN);
  all_colors.push_back(PURPLE);
  all_colors.push_back(CYAN);
  all_colors.push_back(MAGENTA);

  int rand_num = iRand(0, all_colors.size() - 1);
  return all_colors[rand_num];
}

std_msgs::ColorRGBA RvizVisualTools::getColor(const colors &color)
{
  std_msgs::ColorRGBA result;

  switch (color)
  {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = alpha_;
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = alpha_;
      break;
    case GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      result.a = alpha_;
    case DARK_GREY:
      result.r = 0.6;
      result.g = 0.6;
      result.b = 0.6;
      result.a = alpha_;
      break;
    case WHITE:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = alpha_;
      break;
    case ORANGE:
      result.r = 1.0;
      result.g = 0.5;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case TRANSLUCENT_LIGHT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.1;
      break;
    case TRANSLUCENT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.25;
      break;
    case TRANSLUCENT_DARK:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.5;
      break;
    case BLACK:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case BROWN:
      result.r = 0.597;
      result.g = 0.296;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case PINK:
      result.r = 1.0;
      result.g = 0.4;
      result.b = 1;
      result.a = alpha_;
      break;
    case LIME_GREEN:
      result.r = 0.6;
      result.g = 1.0;
      result.b = 0.2;
      result.a = alpha_;
      break;
    case CLEAR:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 0.0;
      break;
    case PURPLE:
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      result.a = alpha_;
      break;
    case CYAN:
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = alpha_;
      break;
    case MAGENTA:
      result.r = 1.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = alpha_;
      break;
    case RAND:
      result = createRandColor();
      break;
    case DEFAULT:
      ROS_WARN_STREAM_NAMED(name_, "The 'DEFAULT' color should probably not "
                                   "be used with getColor(). Defaulting to "
                                   "blue.");
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
      result.a = alpha_;
  }

  return result;
}

std_msgs::ColorRGBA RvizVisualTools::createRandColor()
{
  std_msgs::ColorRGBA result;

  const std::size_t MAX_ATTEMPTS = 10;  // bound the performance
  std::size_t attempts = 0;

  // Make sure color is not *too* dark
  do
  {
    result.r = fRand(0.0, 1.0);
    result.g = fRand(0.0, 1.0);
    result.b = fRand(0.0, 1.0);
    //ROS_DEBUG_STREAM_NAMED(name_, "Looking for random color that is not too light, current value is "
    //<< (result.r + result.g + result.b) << " attempt #" << attempts);
    attempts++;
    if (attempts > MAX_ATTEMPTS)
    {
      ROS_WARN_STREAM_NAMED(name_, "Unable to find appropriate random color after " << MAX_ATTEMPTS << " attempts");
      break;
    }
  } while (result.r + result.g + result.b < 1.5);  // 3 would be white

  // Set alpha value
  result.a = alpha_;

  return result;
}

double RvizVisualTools::slerp(double start, double end, double range, double value)
{
  // std::cout << "start: " << start << " end: " << end << " range: " << range << " value: " << value << std::endl;
  return start + (((end - start) / range) * value);
}

std_msgs::ColorRGBA RvizVisualTools::getColorScale(double value)
{
  // User warning
  if (value < 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "Intensity value for color scale is below range [0,1], value: " << value);
    value = 0;
  }
  else if (value > 1)
  {
    ROS_WARN_STREAM_NAMED(name_, "Intensity value for color scale is above range [0,1], value: " << value);
    value = 1;
  }

  std_msgs::ColorRGBA start;
  std_msgs::ColorRGBA end;

  // For second half of color range move towards RED
  if (value == 0.0)
    return getColor(RED);
  else if (value == 1.0)
    return getColor(GREEN);
  else if (value <= 0.5)
  {
    start = getColor(RED);
    end = getColor(YELLOW);
  }
  else
  {
    start = getColor(YELLOW);
    end = getColor(GREEN);
    value = fmod(value, 0.5);
  }

  std_msgs::ColorRGBA result;
  result.r = slerp(start.r, end.r, 0.5, value);
  result.g = slerp(start.g, end.g, 0.5, value);
  result.b = slerp(start.b, end.b, 0.5, value);
  result.a = alpha_;

  return result;
}

geometry_msgs::Vector3 RvizVisualTools::getScale(const scales &scale, bool arrow_scale, double marker_scale)
{
  geometry_msgs::Vector3 result;
  double val(0.0);
  switch (scale)
  {
    case XXSMALL:
      val = 0.005;
      break;
    case XSMALL:
      val = 0.01;
      break;
    case SMALL:
      val = 0.03;
      break;
    case REGULAR:
      val = 0.05;
      break;
    case LARGE:
      val = 0.1;
      break;
    case xLARGE:
      val = 0.2;
      break;
    case xxLARGE:
      val = 0.3;
      break;
    case xxxLARGE:
      val = 0.4;
      break;
    case XLARGE:
      val = 0.5;
      break;
    case XXLARGE:
      val = 1.0;
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Not implemented yet");
      break;
  }

  // Allows an individual marker size factor and a size factor for all markers
  result.x = val * marker_scale * global_scale_;
  result.y = val * marker_scale * global_scale_;
  result.z = val * marker_scale * global_scale_;

  // The y and z scaling is smaller for arrows
  if (arrow_scale)
  {
    result.y *= 0.1;
    result.z *= 0.1;
  }

  return result;
}

Eigen::Vector3d RvizVisualTools::getCenterPoint(Eigen::Vector3d a, Eigen::Vector3d b)
{
  Eigen::Vector3d center;
  center[0] = (a[0] + b[0]) / 2;
  center[1] = (a[1] + b[1]) / 2;
  center[2] = (a[2] + b[2]) / 2;
  return center;
}

Eigen::Affine3d RvizVisualTools::getVectorBetweenPoints(Eigen::Vector3d a, Eigen::Vector3d b)
{
  // TODO(davetcoleman): handle the error case when a & b are the same point.
  // currently it returns nan for the quaternion

  // from
  // http://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/

  // Goal pose:
  Eigen::Quaterniond q;

  Eigen::Vector3d axis_vector = b - a;
  axis_vector.normalize();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
  right_axis_vector.normalized();
  double theta = axis_vector.dot(up_vector);
  double angle_rotation = -1.0 * acos(theta);

  //-------------------------------------------
  // Method 1 - TF - works
  // Convert to TF
  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);

  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);
  //-------------------------------------------
  // std::cout << q.toRotationMatrix() << std::endl;

  //-------------------------------------------
  // Method 2 - Eigen - broken TODO(davetcoleman)
  // q = Eigen::AngleAxis<double>(angle_rotation, right_axis_vector);
  //-------------------------------------------
  // std::cout << q.toRotationMatrix() << std::endl;

  // Rotate so that vector points along line
  Eigen::Affine3d pose;
  q.normalize();
  pose = q * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose.translation() = a;

  return pose;
}

bool RvizVisualTools::publishMarker(visualization_msgs::Marker &marker)
{
  // Add single marker to array
  markers_.markers.push_back(marker);

  // Determine if we should publish now
  if (!batch_publishing_enabled_ && !internal_batch_publishing_enabled_)
  {
    return triggerBatchPublish();
  }

  return true;
}

void RvizVisualTools::enableBatchPublishing(bool enable)
{
  batch_publishing_enabled_ = enable;
}

bool RvizVisualTools::triggerBatchPublish()
{
  bool result = publishMarkers(markers_);

  markers_.markers.clear();  // remove all cached markers
  return result;
}

bool RvizVisualTools::triggerBatchPublishAndDisable()
{
  triggerBatchPublish();
  batch_publishing_enabled_ = false;
}

bool RvizVisualTools::publishMarkers(visualization_msgs::MarkerArray &markers)
{
  if (!pub_rviz_markers_)  // always check this before publishing
    loadMarkerPub();

  // Check if connected to a subscriber
  if (!pub_rviz_markers_waited_ && !pub_rviz_markers_connected_)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Waiting for subscribers before publishing markers...");
    waitForSubscriber(pub_rviz_markers_);

    // Only wait for the publisher once, after that just ignore the lack of connection
    pub_rviz_markers_waited_ = true;
  }

  // Check if any actual markers exist to publish
  if (markers.markers.empty())
    return false;

  // Change all markers to be inverted in color
  if (psychedelic_mode_)
  {
    for (std::size_t i = 0; i < markers.markers.size(); ++i)
    {
      markers.markers[i].color.r = 1.0 - markers.markers[i].color.r;
      markers.markers[i].color.g = 1.0 - markers.markers[i].color.g;
      markers.markers[i].color.b = 1.0 - markers.markers[i].color.b;
      for (std::size_t j = 0; j < markers.markers[i].colors.size(); ++j)
      {
        markers.markers[i].colors[j].r = 1.0 - markers.markers[i].colors[j].r;
        markers.markers[i].colors[j].g = 1.0 - markers.markers[i].colors[j].g;
        markers.markers[i].colors[j].b = 1.0 - markers.markers[i].colors[j].b;
      }
    }
  }

  // Publish
  pub_rviz_markers_.publish(markers);
  ros::spinOnce();
  return true;
}

bool RvizVisualTools::publishCone(const Eigen::Affine3d &pose, double angle, const colors &color, double scale)
{
  return publishCone(convertPose(pose), angle, color, scale);
}

bool RvizVisualTools::publishCone(const geometry_msgs::Pose &pose, double angle, const colors &color, double scale)
{
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[3];
  static const double delta_theta = M_PI / 16.0;
  double theta = 0;

  for (std::size_t i = 0; i < 32; i++)
  {
    p[0].x = 0;
    p[0].y = 0;
    p[0].z = 0;

    p[1].x = scale;
    p[1].y = scale * cos(theta);
    p[1].z = scale * sin(theta);

    p[2].x = scale;
    p[2].y = scale * cos(theta + delta_theta);
    p[2].z = scale * sin(theta + delta_theta);

    triangle_marker_.points.push_back(p[0]);
    triangle_marker_.points.push_back(p[1]);
    triangle_marker_.points.push_back(p[2]);

    theta += delta_theta;
  }

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  return publishMarker(triangle_marker_);
}

bool RvizVisualTools::publishXYPlane(const Eigen::Affine3d &pose, const colors &color, double scale)
{
  return publishXYPlane(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishXYPlane(const geometry_msgs::Pose &pose, const colors &color, double scale)
{
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[4];
  p[0].x = 1.0 * scale;
  p[0].y = 1.0 * scale;
  p[0].z = 0.0;

  p[1].x = -1.0 * scale;
  p[1].y = 1.0 * scale;
  p[1].z = 0.0;

  p[2].x = -1.0 * scale;
  p[2].y = -1.0 * scale;
  p[2].z = 0.0;

  p[3].x = 1.0 * scale;
  p[3].y = -1.0 * scale;
  p[3].z = 0.0;

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.push_back(p[0]);
  triangle_marker_.points.push_back(p[1]);
  triangle_marker_.points.push_back(p[2]);

  triangle_marker_.points.push_back(p[2]);
  triangle_marker_.points.push_back(p[3]);
  triangle_marker_.points.push_back(p[0]);

  return publishMarker(triangle_marker_);
}

bool RvizVisualTools::publishXZPlane(const Eigen::Affine3d &pose, const colors &color, double scale)
{
  return publishXZPlane(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishXZPlane(const geometry_msgs::Pose &pose, const colors &color, double scale)
{
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[4];
  p[0].x = 1.0 * scale;
  p[0].y = 0;
  p[0].z = 1.0 * scale;

  p[1].x = -1.0 * scale;
  p[1].y = 0;
  p[1].z = 1.0 * scale;

  p[2].x = -1.0 * scale;
  p[2].y = 0;
  p[2].z = -1.0 * scale;

  p[3].x = 1.0 * scale;
  p[3].y = 0;
  p[3].z = -1.0 * scale;

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.push_back(p[0]);
  triangle_marker_.points.push_back(p[1]);
  triangle_marker_.points.push_back(p[2]);

  triangle_marker_.points.push_back(p[2]);
  triangle_marker_.points.push_back(p[3]);
  triangle_marker_.points.push_back(p[0]);

  return publishMarker(triangle_marker_);
}

bool RvizVisualTools::publishYZPlane(const Eigen::Affine3d &pose, const colors &color, double scale)
{
  return publishYZPlane(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishYZPlane(const geometry_msgs::Pose &pose, const colors &color, double scale)
{
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[4];
  p[0].x = 0;
  p[0].y = 1.0 * scale;
  p[0].z = 1.0 * scale;

  p[1].x = 0;
  p[1].y = -1.0 * scale;
  p[1].z = 1.0 * scale;

  p[2].x = 0;
  p[2].y = -1.0 * scale;
  p[2].z = -1.0 * scale;

  p[3].x = 0;
  p[3].y = 1.0 * scale;
  p[3].z = -1.0 * scale;

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.push_back(p[0]);
  triangle_marker_.points.push_back(p[1]);
  triangle_marker_.points.push_back(p[2]);

  triangle_marker_.points.push_back(p[2]);
  triangle_marker_.points.push_back(p[3]);
  triangle_marker_.points.push_back(p[0]);

  return publishMarker(triangle_marker_);
}

bool RvizVisualTools::publishSphere(const Eigen::Affine3d &pose, const colors &color, const scales &scale,
                                    const std::string &ns, const std::size_t &id)
{
  return publishSphere(convertPose(pose), color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d &point, const colors &color, const scales &scale,
                                    const std::string &ns, const std::size_t &id)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d &point, const colors &color, const double scale,
                                    const std::string &ns, const std::size_t &id)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Point &point, const colors &color, const scales &scale,
                                    const std::string &ns, const std::size_t &id)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position = point;
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const colors &color, const scales &scale,
                                    const std::string &ns, const std::size_t &id)
{
  return publishSphere(pose, color, getScale(scale, false, 0.1), ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const colors &color, double scale,
                                    const std::string &ns, const std::size_t &id)
{
  geometry_msgs::Vector3 scale_msg;
  scale_msg.x = scale;
  scale_msg.y = scale;
  scale_msg.z = scale;
  return publishSphere(pose, color, scale_msg, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const colors &color,
                                    const geometry_msgs::Vector3 scale, const std::string &ns, const std::size_t &id)
{
  return publishSphere(pose, getColor(color), scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Affine3d &pose, const std_msgs::ColorRGBA &color,
                                    const geometry_msgs::Vector3 scale, const std::string &ns, const std::size_t &id)
{
  return publishSphere(convertPose(pose), color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const std_msgs::ColorRGBA &color,
                                    const geometry_msgs::Vector3 scale, const std::string &ns, const std::size_t &id)
{
  // Set the frame ID and timestamp
  sphere_marker_.header.stamp = ros::Time::now();

  // Overwrite ID or increment?
  if (id == 0)
    sphere_marker_.id++;
  else
    sphere_marker_.id = id;

  sphere_marker_.color = color;
  sphere_marker_.scale = scale;
  sphere_marker_.ns = ns;

  // Update the single point with new pose
  sphere_marker_.points[0] = pose.position;
  sphere_marker_.colors[0] = color;

  // Helper for publishing rviz markers
  return publishMarker(sphere_marker_);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::PoseStamped &pose, const colors &color,
                                    const geometry_msgs::Vector3 scale, const std::string &ns, const std::size_t &id)
{
  // Set the frame ID and timestamp
  sphere_marker_.header = pose.header;

  if (id == 0)
    sphere_marker_.id++;
  else
    sphere_marker_.id = id;

  sphere_marker_.color = getColor(color);
  sphere_marker_.scale = scale;
  sphere_marker_.ns = ns;

  // Update the single point with new pose
  sphere_marker_.points[0] = pose.pose.position;
  sphere_marker_.colors[0] = getColor(color);

  // Helper for publishing rviz markers
  publishMarker(sphere_marker_);

  sphere_marker_.header.frame_id = base_frame_;  // restore default frame
  return true;
}

bool RvizVisualTools::publishXArrow(const Eigen::Affine3d &pose, const colors &color, const scales &scale,
                                    double length)
{
  return publishArrow(convertPose(pose), color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::Pose &pose, const colors &color, const scales &scale,
                                    double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::PoseStamped &pose, const colors &color, const scales &scale,
                                    double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishYArrow(const Eigen::Affine3d &pose, const colors &color, const scales &scale,
                                    double length)
{
  Eigen::Affine3d arrow_pose = pose * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::Pose &pose, const colors &color, const scales &scale,
                                    double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::PoseStamped &pose, const colors &color, const scales &scale,
                                    double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishZArrow(const Eigen::Affine3d &pose, const colors &color, const scales &scale,
                                    double length, const std::size_t &id)
{
  Eigen::Affine3d arrow_pose = pose * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length, id);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::Pose &pose, const colors &color, const scales &scale,
                                    double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::PoseStamped &pose, const colors &color, const scales &scale,
                                    double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::PoseStamped &pose, const colors &color, const scales &scale,
                                    double length, const std::size_t &id)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length, id);
}

bool RvizVisualTools::publishArrow(const Eigen::Affine3d &pose, const colors &color, const scales &scale, double length,
                                   const std::size_t &id)
{
  return publishArrow(convertPose(pose), color, scale, length, id);
}

bool RvizVisualTools::publishArrow(const geometry_msgs::Pose &pose, const colors &color, const scales &scale,
                                   double length, const std::size_t &id)
{
  // Set the frame ID and timestamp.
  arrow_marker_.header.stamp = ros::Time::now();
  arrow_marker_.header.frame_id = base_frame_;

  if (id == 0)
    arrow_marker_.id++;
  else
    arrow_marker_.id = id;

  arrow_marker_.pose = pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale, true);
  arrow_marker_.scale.x = length;  // overrides previous x scale specified

  // Helper for publishing rviz markers
  return publishMarker(arrow_marker_);
}

bool RvizVisualTools::publishArrow(const geometry_msgs::PoseStamped &pose, const colors &color, const scales &scale,
                                   double length, const std::size_t &id)
{
  // Set the frame ID and timestamp.
  arrow_marker_.header = pose.header;

  if (id == 0)
    arrow_marker_.id++;
  else
    arrow_marker_.id = id;

  arrow_marker_.pose = pose.pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale, true);
  arrow_marker_.scale.x = length;  // overrides previous x scale specified

  // Helper for publishing rviz markers
  publishMarker(arrow_marker_);

  arrow_marker_.header.frame_id = base_frame_;  // restore default frame
}

bool RvizVisualTools::publishBlock(const geometry_msgs::Pose &pose, const colors &color, const double &block_size)
{
  // Set the timestamp
  block_marker_.header.stamp = ros::Time::now();

  block_marker_.id++;

  // Set the pose
  block_marker_.pose = pose;

  // Set marker size
  block_marker_.scale.x = block_size;
  block_marker_.scale.y = block_size;
  block_marker_.scale.z = block_size;

  // Set marker color
  block_marker_.color = getColor(color);

  // Helper for publishing rviz markers
  return publishMarker(block_marker_);
}

bool RvizVisualTools::publishBlock(const Eigen::Affine3d &pose, const colors &color, const double &block_size)
{
  return publishBlock(convertPose(pose), color, block_size);
}

bool RvizVisualTools::publishAxisLabeled(const Eigen::Affine3d &pose, const std::string &label, const scales &scale)
{
  return publishAxisLabeled(convertPose(pose), label, scale);
}

bool RvizVisualTools::publishAxisLabeled(const geometry_msgs::Pose &pose, const std::string &label, const scales &scale)
{
  publishText(pose, label, rviz_visual_tools::BLACK, rviz_visual_tools::SMALL,
              false);  // TODO(davetcoleman): change size based on passed in scale
  publishAxis(pose, 0.1, 0.01, label);
  return true;
}

bool RvizVisualTools::publishAxis(const geometry_msgs::Pose &pose, double length, double radius, const std::string &ns)
{
  return publishAxis(convertPose(pose), length, radius, ns);
}

bool RvizVisualTools::publishAxis(const Eigen::Affine3d &pose, double length, double radius, const std::string &ns)
{
  // Batch publish, unless it is already enabled by user
  enableInternalBatchPublishing(true);

  // Publish x axis
  Eigen::Affine3d x_pose =
      Eigen::Translation3d(length / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  x_pose = pose * x_pose;
  publishCylinder(x_pose, rviz_visual_tools::RED, length, radius, ns);

  // Publish y axis
  Eigen::Affine3d y_pose =
      Eigen::Translation3d(0, length / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  y_pose = pose * y_pose;
  publishCylinder(y_pose, rviz_visual_tools::GREEN, length, radius, ns);

  // Publish z axis
  Eigen::Affine3d z_pose = Eigen::Translation3d(0, 0, length / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  z_pose = pose * z_pose;
  publishCylinder(z_pose, rviz_visual_tools::BLUE, length, radius, ns);

  // Batch publish
  return triggerInternalBatchPublishAndDisable();
}

bool RvizVisualTools::publishCylinder(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                                      const colors &color, double radius, const std::string &ns)
{
  return publishCylinder(point1, point2, getColor(color), radius, ns);
}

bool RvizVisualTools::publishCylinder(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                                      const std_msgs::ColorRGBA &color, double radius, const std::string &ns)
{
  // Distance between two points
  double height = (point1 - point2).lpNorm<2>();

  // Find center point
  Eigen::Vector3d pt_center = getCenterPoint(point1, point2);

  // Create vector
  Eigen::Affine3d pose;
  pose = getVectorBetweenPoints(pt_center, point2);

  // Convert pose to be normal to cylindar axis
  Eigen::Affine3d rotation;
  rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;

  // Turn into msg
  publishCylinder(convertPose(pose), color, height, radius);
}

bool RvizVisualTools::publishCylinder(const Eigen::Affine3d &pose, const colors &color, double height, double radius,
                                      const std::string &ns)
{
  return publishCylinder(convertPose(pose), color, height, radius, ns);
}

bool RvizVisualTools::publishCylinder(const geometry_msgs::Pose &pose, const colors &color, double height,
                                      double radius, const std::string &ns)
{
  return publishCylinder(pose, getColor(color), height, radius, ns);
}

bool RvizVisualTools::publishCylinder(const geometry_msgs::Pose &pose, const std_msgs::ColorRGBA &color, double height,
                                      double radius, const std::string &ns)
{
  // Set the timestamp
  cylinder_marker_.header.stamp = ros::Time::now();
  cylinder_marker_.ns = ns;
  cylinder_marker_.id++;

  // Set the pose
  cylinder_marker_.pose = pose;

  // Set marker size
  cylinder_marker_.scale.x = radius;
  cylinder_marker_.scale.y = radius;
  cylinder_marker_.scale.z = height;

  // Set marker color
  cylinder_marker_.color = color;

  // Helper for publishing rviz markers
  return publishMarker(cylinder_marker_);
}

bool RvizVisualTools::publishMesh(const Eigen::Affine3d &pose, const std::string &file_name, const colors &color,
                                  double scale, const std::string &ns, const std::size_t &id)
{
  return publishMesh(convertPose(pose), file_name, color, scale, ns, id);
}

bool RvizVisualTools::publishMesh(const geometry_msgs::Pose &pose, const std::string &file_name, const colors &color,
                                  double scale, const std::string &ns, const std::size_t &id)
{
  // Set the timestamp
  mesh_marker_.header.stamp = ros::Time::now();

  if (id == 0)
    mesh_marker_.id++;
  else
    mesh_marker_.id = id;

  // Set the mesh
  mesh_marker_.mesh_resource = file_name;
  mesh_marker_.mesh_use_embedded_materials = true;

  // Set the pose
  mesh_marker_.pose = pose;

  // Set marker size
  mesh_marker_.scale.x = scale;
  mesh_marker_.scale.y = scale;
  mesh_marker_.scale.z = scale;

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  mesh_marker_.ns = ns;

  // Set marker color
  mesh_marker_.color = getColor(color);

  // Helper for publishing rviz markers
  return publishMarker(mesh_marker_);
}

bool RvizVisualTools::publishGraph(const graph_msgs::GeometryGraph &graph, const colors &color, double radius)
{
  // Track which pairs of nodes we've already connected since graph is
  // bi-directional
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator, bool> return_value;
  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert(node_ids(i, j));
      if (return_value.second == false)
      {
        // Element already existed in set, so don't add a new collision object
      }
      else
      {
        // Create a cylinder from two points
        a = convertPoint(graph.nodes[i]);
        b = convertPoint(graph.nodes[graph.edges[i].node_ids[j]]);

        publishCylinder(a, b, color, radius);
      }
    }
  }

  return true;
}

bool RvizVisualTools::publishCuboid(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2, const colors &color)
{
  return publishCuboid(convertPoint(point1), convertPoint(point2), color);
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                    const colors &color, const std::string &ns, const std::size_t &id)
{
  // Set the timestamp
  cuboid_marker_.header.stamp = ros::Time::now();
  cuboid_marker_.ns = ns;

  if (id == 0)  // Provide a new id every call to this function
    cuboid_marker_.id++;
  else  // allow marker to be overwritten
    cuboid_marker_.id = id;

  cuboid_marker_.color = getColor(color);

  // Calculate center pose
  geometry_msgs::Pose pose;
  pose.position.x = (point1.x - point2.x) / 2.0 + point2.x;
  pose.position.y = (point1.y - point2.y) / 2.0 + point2.y;
  pose.position.z = (point1.z - point2.z) / 2.0 + point2.z;
  cuboid_marker_.pose = pose;

  // Calculate scale
  cuboid_marker_.scale.x = fabs(point1.x - point2.x);
  cuboid_marker_.scale.y = fabs(point1.y - point2.y);
  cuboid_marker_.scale.z = fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (!cuboid_marker_.scale.x)
    cuboid_marker_.scale.x = SMALL_SCALE;
  if (!cuboid_marker_.scale.y)
    cuboid_marker_.scale.y = SMALL_SCALE;
  if (!cuboid_marker_.scale.z)
    cuboid_marker_.scale.z = SMALL_SCALE;

  // Helper for publishing rviz markers
  return publishMarker(cuboid_marker_);
}

bool RvizVisualTools::publishCuboid(const Eigen::Affine3d &pose, const double depth, const double width,
                                    const double height, const colors &color)
{
  return publishCuboid(convertPose(pose), depth, width, height, color);
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Pose &pose, const double depth, const double width,
                                    const double height, const colors &color)
{
  cuboid_marker_.header.stamp = ros::Time::now();

  cuboid_marker_.id++;
  cuboid_marker_.color = getColor(color);

  cuboid_marker_.pose = pose;

  // Prevent scale from being zero
  if (depth <= 0)
    cuboid_marker_.scale.x = SMALL_SCALE;
  else
    cuboid_marker_.scale.x = depth;

  if (width <= 0)
    cuboid_marker_.scale.y = SMALL_SCALE;
  else
    cuboid_marker_.scale.y = width;

  if (height <= 0)
    cuboid_marker_.scale.z = SMALL_SCALE;
  else
    cuboid_marker_.scale.z = height;

  return publishMarker(cuboid_marker_);
}

bool RvizVisualTools::publishLine(const Eigen::Affine3d &point1, const Eigen::Affine3d &point2, const colors &color,
                                  const scales &scale)
{
  return publishLine(convertPoseToPoint(point1), convertPoseToPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2, const colors &color,
                                  const scales &scale)
{
  return publishLine(convertPoint(point1), convertPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                                  const std_msgs::ColorRGBA &color, const scales &scale)
{
  return publishLine(convertPoint(point1), convertPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                                  const std_msgs::ColorRGBA &color, const double &radius)
{
  geometry_msgs::Vector3 scale;
  scale.x = radius;
  scale.y = radius;
  scale.z = radius;
  return publishLine(convertPoint(point1), convertPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                  const colors &color, const scales &scale)
{
  return publishLine(point1, point2, getColor(color), scale);
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                  const std_msgs::ColorRGBA &color, const scales &scale)
{
  return publishLine(point1, point2, color, getScale(scale, false, 0.1));
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                  const std_msgs::ColorRGBA &color, const geometry_msgs::Vector3 &scale)
{
  // Set the timestamp
  line_strip_marker_.header.stamp = ros::Time::now();

  line_strip_marker_.id++;
  line_strip_marker_.color = color;
  line_strip_marker_.scale = scale;

  line_strip_marker_.points.clear();
  line_strip_marker_.points.push_back(point1);
  line_strip_marker_.points.push_back(point2);

  // Helper for publishing rviz markers
  return publishMarker(line_strip_marker_);
}

bool RvizVisualTools::publishPath(const std::vector<geometry_msgs::Point> &path, const colors &color,
                                  const scales &scale, const std::string &ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(name_, "Skipping path because " << path.size() << " points passed in.");
    return true;
  }

  line_strip_marker_.header.stamp = ros::Time();
  line_strip_marker_.ns = ns;

  // Provide a new id every call to this function
  line_strip_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_strip_marker_.scale = getScale(scale, false, 0.25);
  line_strip_marker_.color = this_color;
  line_strip_marker_.points.clear();
  line_strip_marker_.colors.clear();

  for (std::size_t i = 1; i < path.size(); ++i)
  {
    // Add the point pair to the line message
    line_strip_marker_.points.push_back(path[i - 1]);
    line_strip_marker_.points.push_back(path[i]);
    line_strip_marker_.colors.push_back(this_color);
    line_strip_marker_.colors.push_back(this_color);
  }

  // Helper for publishing rviz markers
  return publishMarker(line_strip_marker_);
}

bool RvizVisualTools::publishPath(const std::vector<Eigen::Vector3d> &path, const colors &color,
                                  const double radius, const std::string &ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(name_, "Skipping path because " << path.size() << " points passed in.");
    return true;
  }

  // Batch publish, unless it is already enabled by user
  enableInternalBatchPublishing(true);

  // Create the cylinders
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    publishCylinder(path[i - 1], path[i], color, radius, ns);
  }

  // Batch publish
  return triggerInternalBatchPublishAndDisable();
}

bool RvizVisualTools::publishPolygon(const geometry_msgs::Polygon &polygon, const colors &color, const scales &scale,
                                     const std::string &ns)
{
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point temp;
  geometry_msgs::Point first;  // remember first point because we will connect
                               // first and last points
                               // for last line
  for (std::size_t i = 0; i < polygon.points.size(); ++i)
  {
    temp.x = polygon.points[i].x;
    temp.y = polygon.points[i].y;
    temp.z = polygon.points[i].z;
    if (i == 0)
      first = temp;
    points.push_back(temp);
  }
  points.push_back(first);  // connect first and last points for last line

  publishPath(points, color, scale, ns);
}

bool RvizVisualTools::publishWireframeCuboid(const Eigen::Affine3d &pose, double depth, double width, double height,
                                             const colors &color, const std::string &ns, const std::size_t &id)
{
  Eigen::Vector3d min_point, max_point;
  min_point << -depth / 2, -width / 2, -height / 2;
  max_point << depth / 2, width / 2, height / 2;
  return publishWireframeCuboid(pose, min_point, max_point, color, ns, id);
}

bool RvizVisualTools::publishWireframeCuboid(const Eigen::Affine3d &pose, const Eigen::Vector3d &min_point,
                                             const Eigen::Vector3d &max_point, const colors &color,
                                             const std::string &ns, const std::size_t &id)
{
  // Extract 8 cuboid vertices
  Eigen::Vector3d p1(min_point[0], min_point[1], min_point[2]);
  Eigen::Vector3d p2(min_point[0], min_point[1], max_point[2]);
  Eigen::Vector3d p3(max_point[0], min_point[1], max_point[2]);
  Eigen::Vector3d p4(max_point[0], min_point[1], min_point[2]);
  Eigen::Vector3d p5(min_point[0], max_point[1], min_point[2]);
  Eigen::Vector3d p6(min_point[0], max_point[1], max_point[2]);
  Eigen::Vector3d p7(max_point[0], max_point[1], max_point[2]);
  Eigen::Vector3d p8(max_point[0], max_point[1], min_point[2]);

  p1 = pose * p1;
  p2 = pose * p2;
  p3 = pose * p3;
  p4 = pose * p4;
  p5 = pose * p5;
  p6 = pose * p6;
  p7 = pose * p7;
  p8 = pose * p8;

  // Setup marker
  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = ns;

  if (id == 0)  // Provide a new id every call to this function
    line_list_marker_.id++;
  else  // allow marker to be overwritten
    line_list_marker_.id = id;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_list_marker_.scale = getScale(XXSMALL);
  line_list_marker_.color = this_color;
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();

  // Add each point pair to the line message
  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.points.push_back(convertPoint(p5));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p5));
  line_list_marker_.points.push_back(convertPoint(p6));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p5));
  line_list_marker_.points.push_back(convertPoint(p8));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.points.push_back(convertPoint(p6));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p6));
  line_list_marker_.points.push_back(convertPoint(p7));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p7));
  line_list_marker_.points.push_back(convertPoint(p8));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.points.push_back(convertPoint(p8));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.points.push_back(convertPoint(p7));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  // Helper for publishing rviz markers
  return publishMarker(line_list_marker_);
}

bool RvizVisualTools::publishWireframeRectangle(const Eigen::Affine3d &pose, const double &height, const double &width,
                                                const colors &color, const scales &scale)
{
  // Extract 8 cuboid vertices
  Eigen::Vector3d p1(-width / 2.0, -height / 2.0, 0.0);
  Eigen::Vector3d p2(-width / 2.0, height / 2.0, 0.0);
  Eigen::Vector3d p3(width / 2.0, height / 2.0, 0.0);
  Eigen::Vector3d p4(width / 2.0, -height / 2.0, 0.0);

  p1 = pose * p1;
  p2 = pose * p2;
  p3 = pose * p3;
  p4 = pose * p4;

  // Setup marker
  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = "Wireframe Rectangle";

  // Provide a new id every call to this function
  line_list_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_list_marker_.scale = getScale(scale, false, 0.25);
  line_list_marker_.color = this_color;
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();

  // Add each point pair to the line message
  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  // Helper for publishing rviz markers
  return publishMarker(line_list_marker_);
}

bool RvizVisualTools::publishWireframeRectangle(const Eigen::Affine3d &pose, const Eigen::Vector3d &p1_in,
                                                const Eigen::Vector3d &p2_in, const Eigen::Vector3d &p3_in,
                                                const Eigen::Vector3d &p4_in, const colors &color, const scales &scale)
{
  Eigen::Vector3d p1;
  Eigen::Vector3d p2;
  Eigen::Vector3d p3;
  Eigen::Vector3d p4;

  // Transform to pose
  p1 = pose * p1_in;
  p2 = pose * p2_in;
  p3 = pose * p3_in;
  p4 = pose * p4_in;

  // Setup marker
  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = "Wireframe Rectangle";

  // Provide a new id every call to this function
  line_list_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_list_marker_.scale = getScale(scale, false, 0.25);
  line_list_marker_.color = this_color;
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();

  // Add each point pair to the line message
  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p2));
  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p3));
  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  line_list_marker_.points.push_back(convertPoint(p4));
  line_list_marker_.points.push_back(convertPoint(p1));
  line_list_marker_.colors.push_back(this_color);
  line_list_marker_.colors.push_back(this_color);

  // Helper for publishing rviz markers
  return publishMarker(line_list_marker_);
}

bool RvizVisualTools::publishSpheres(const std::vector<Eigen::Vector3d> &points, const colors &color,
                                     const double scale, const std::string &ns)
{
  std::vector<geometry_msgs::Point> points_msg;
  // geometry_msgs::Point temp;

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    // tf::pointEigenToMsg(points[i], temp);
    points_msg.push_back(convertPoint(points[i]));
  }

  return publishSpheres(points_msg, color, scale, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const colors &color,
                                     const double scale, const std::string &ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  publishSpheres(points, color, scale_vector, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const colors &color,
                                     const scales &scale, const std::string &ns)
{
  publishSpheres(points, color, getScale(scale, false, 0.25), ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const colors &color,
                                     const geometry_msgs::Vector3 &scale, const std::string &ns)
{
  spheres_marker_.header.stamp = ros::Time();
  spheres_marker_.ns = ns;

  // Provide a new id every call to this function
  spheres_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  spheres_marker_.scale = scale;
  spheres_marker_.color = this_color;
  // spheres_marker_.points.clear();
  spheres_marker_.colors.clear();

  spheres_marker_.points = points;  // straight copy

  // Convert path coordinates
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    spheres_marker_.colors.push_back(this_color);
  }

  // Helper for publishing rviz markers
  return publishMarker(spheres_marker_);
}

bool RvizVisualTools::publishText(const Eigen::Affine3d &pose, const std::string &text, const colors &color,
                                  const scales &scale, bool static_id)
{
  return publishText(convertPose(pose), text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const Eigen::Affine3d &pose, const std::string &text, const colors &color,
                                  const geometry_msgs::Vector3 scale, bool static_id)
{
  return publishText(convertPose(pose), text, color, scale, static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const colors &color,
                                  const scales &scale, bool static_id)
{
  return publishText(pose, text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const colors &color,
                                  const geometry_msgs::Vector3 scale, bool static_id)
{
  // Save the ID if this is a static ID or keep incrementing ID if not static
  double temp_id = text_marker_.id;
  if (static_id)
  {
    text_marker_.id = 0;
  }
  else
  {
    text_marker_.id++;
  }

  text_marker_.header.stamp = ros::Time::now();
  text_marker_.header.frame_id = base_frame_;
  text_marker_.text = text;
  text_marker_.pose = pose;
  text_marker_.color = getColor(color);
  text_marker_.scale = scale;

  // Helper for publishing rviz markers
  publishMarker(text_marker_);

  // Restore the ID count if needed
  if (static_id)
    text_marker_.id = temp_id;

  return true;
}

bool RvizVisualTools::publishTests()
{
  // Create pose
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;

  // Test color range
  ROS_INFO_STREAM_NAMED(name_, "Publising range of colors red->green");
  generateRandomPose(pose1);
  for (double i = 0; i < 1.0; i += 0.01)
  {
    // std::cout << "Publishing sphere with intensity " << i << std::endl;
    geometry_msgs::Vector3 scale = getScale(XLARGE, false, 0.1);
    std_msgs::ColorRGBA color = getColorScale(i);
    std::size_t id = 1;
    publishSphere(pose1, color, scale, "Sphere", id);
    ros::Duration(0.1).sleep();

    if (!ros::ok())
      return false;
  }

  // Test all shapes ----------

  ROS_INFO_STREAM_NAMED(name_, "Publishing Axis");
  generateRandomPose(pose1);
  publishAxis(pose1);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Arrow");
  generateRandomPose(pose1);
  publishArrow(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Sphere");
  generateRandomPose(pose1);
  publishSphere(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Rectangular Cuboid");
  // TODO(davetcoleman): use generateRandomCuboid()
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishCuboid(pose1.position, pose2.position, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Line");
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishLine(pose1.position, pose2.position, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  // Deprecated
  // ROS_INFO_STREAM_NAMED(name_, "Publishing Block");
  // generateRandomPose(pose1);
  // publishBlock(pose1, rviz_visual_tools::RAND);
  // ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Cylinder");
  generateRandomPose(pose1);
  publishCylinder(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Text");
  generateRandomPose(pose1);
  publishText(pose1, "Test", rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Wireframe Cuboid");
  Eigen::Vector3d min_point, max_point;
  // TODO(davetcoleman): use generateRandomCuboid()
  min_point << -0.1, -.25, -.3;
  max_point << .3, .2, .1;
  generateRandomPose(pose1);
  publishWireframeCuboid(convertPose(pose1), min_point, max_point);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing depth/width/height Wireframe Cuboid");
  double depth = 0.5, width = 0.25, height = 0.125;
  generateRandomPose(pose1);
  publishWireframeCuboid(convertPose(pose1), depth, width, height);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publishing Planes");
  generateRandomPose(pose1);
  publishXYPlane(pose1, rviz_visual_tools::RED, 0.1);
  ros::Duration(1.0).sleep();
  publishXZPlane(pose1, rviz_visual_tools::GREEN, 0.1);
  ros::Duration(1.0).sleep();
  publishYZPlane(pose1, rviz_visual_tools::BLUE, 0.1);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(name_, "Publising Axis Cone");
  generateRandomPose(pose1);
  publishCone(pose1, M_PI / 6.0, rviz_visual_tools::RAND, 0.2);
  ros::Duration(1.0).sleep();

  return true;
}

geometry_msgs::Pose RvizVisualTools::convertPose(const Eigen::Affine3d &pose)
{
  tf::poseEigenToMsg(pose, shared_pose_msg_);
  return shared_pose_msg_;
}

void RvizVisualTools::convertPoseSafe(const Eigen::Affine3d &pose, geometry_msgs::Pose &pose_msg)
{
  tf::poseEigenToMsg(pose, pose_msg);
}

Eigen::Affine3d RvizVisualTools::convertPose(const geometry_msgs::Pose &pose)
{
  tf::poseMsgToEigen(pose, shared_pose_eigen_);
  return shared_pose_eigen_;
}

Eigen::Affine3d RvizVisualTools::convertPoint32ToPose(const geometry_msgs::Point32 &point)
{
  shared_pose_eigen_ = Eigen::Affine3d::Identity();
  shared_pose_eigen_.translation().x() = point.x;
  shared_pose_eigen_.translation().y() = point.y;
  shared_pose_eigen_.translation().z() = point.z;
  return shared_pose_eigen_;
}

geometry_msgs::Pose RvizVisualTools::convertPointToPose(const geometry_msgs::Point &point)
{
  shared_pose_msg_.orientation.x = 0.0;
  shared_pose_msg_.orientation.y = 0.0;
  shared_pose_msg_.orientation.z = 0.0;
  shared_pose_msg_.orientation.w = 1.0;
  shared_pose_msg_.position = point;
  return shared_pose_msg_;
}

Eigen::Affine3d RvizVisualTools::convertPointToPose(const Eigen::Vector3d &point)
{
  shared_pose_eigen_ = Eigen::Affine3d::Identity();
  shared_pose_eigen_.translation() = point;
  return shared_pose_eigen_;
}

geometry_msgs::Point RvizVisualTools::convertPoseToPoint(const Eigen::Affine3d &pose)
{
  tf::poseEigenToMsg(pose, shared_pose_msg_);
  return shared_pose_msg_.position;
}

Eigen::Vector3d RvizVisualTools::convertPoint(const geometry_msgs::Point &point)
{
  shared_point_eigen_[0] = point.x;
  shared_point_eigen_[1] = point.y;
  shared_point_eigen_[2] = point.z;
  return shared_point_eigen_;
}

Eigen::Vector3d RvizVisualTools::convertPoint32(const geometry_msgs::Point32 &point)
{
  shared_point_eigen_[0] = point.x;
  shared_point_eigen_[1] = point.y;
  shared_point_eigen_[2] = point.z;
  return shared_point_eigen_;
}

geometry_msgs::Point32 RvizVisualTools::convertPoint32(const Eigen::Vector3d &point)
{
  shared_point32_msg_.x = point[0];
  shared_point32_msg_.y = point[1];
  shared_point32_msg_.z = point[2];
  return shared_point32_msg_;
}

geometry_msgs::Point RvizVisualTools::convertPoint(const geometry_msgs::Vector3 &point)
{
  shared_point_msg_.x = point.x;
  shared_point_msg_.y = point.y;
  shared_point_msg_.z = point.z;
  return shared_point_msg_;
}

geometry_msgs::Point RvizVisualTools::convertPoint(const Eigen::Vector3d &point)
{
  shared_point_msg_.x = point.x();
  shared_point_msg_.y = point.y();
  shared_point_msg_.z = point.z();
  return shared_point_msg_;
}

Eigen::Affine3d RvizVisualTools::convertFromXYZRPY(std::vector<double> transform6)
{
  if (transform6.size() != 6)
  {
    // note: cannot use name_ var b/c this func is static
    ROS_ERROR_STREAM_NAMED("visual_tools", "Incorrect number of variables passed for 6-size transform");
    throw;
  }

  return convertFromXYZRPY(transform6[0], transform6[1], transform6[2], transform6[3], transform6[4], transform6[5]);
}

Eigen::Affine3d RvizVisualTools::convertFromXYZRPY(const double &x, const double &y, const double &z,
                                                   const double &roll, const double &pitch, const double &yaw)
{
  // R-P-Y / X-Y-Z / 0-1-2 Euler Angle Standard
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

  return Eigen::Translation3d(x, y, z) * quaternion;
}

void RvizVisualTools::convertToXYZRPY(const Eigen::Affine3d &pose, std::vector<double> &xyzrpy)
{
  xyzrpy.resize(6);
  convertToXYZRPY(pose, xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
}

void RvizVisualTools::convertToXYZRPY(const Eigen::Affine3d &pose, double &x, double &y, double &z, double &roll,
                                      double &pitch, double &yaw)
{
  x = pose(0, 3);
  y = pose(1, 3);
  z = pose(2, 3);

  // R-P-Y / X-Y-Z / 0-1-2 Euler Angle Standard
  Eigen::Vector3d vec = pose.rotation().eulerAngles(0, 1, 2);
  roll = vec[0];
  pitch = vec[1];
  yaw = vec[2];
}

void RvizVisualTools::generateRandomPose(geometry_msgs::Pose &pose, RandomPoseBounds pose_bounds)
{
  generateRandomPose(shared_pose_eigen_, pose_bounds);
  pose = convertPose(shared_pose_eigen_);
}

void RvizVisualTools::generateRandomCuboid(geometry_msgs::Pose &cuboid_pose, double &depth, double &width,
                                           double &height, RandomPoseBounds pose_bounds,
                                           RandomCuboidBounds cuboid_bounds)
{
  // Size
  depth = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);
  width = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);
  height = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);

  // Orientation
  generateRandomPose(cuboid_pose, pose_bounds);
}

void RvizVisualTools::generateRandomPose(Eigen::Affine3d &pose, RandomPoseBounds pose_bounds)
{
  // Error check elevation & azimuth angles
  // 0 <= elevation <= pi
  // 0 <= azimuth   <= 2 * pi
  if (pose_bounds.elevation_min_ < 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "min elevation bound < 0, setting equal to 0");
    pose_bounds.elevation_min_ = 0;
  }

  if (pose_bounds.elevation_max_ > M_PI)
  {
    ROS_WARN_STREAM_NAMED(name_, "max elevation bound > pi, setting equal to pi ");
    pose_bounds.elevation_max_ = M_PI;
  }

  if (pose_bounds.azimuth_min_ < 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "min azimuth bound < 0, setting equal to 0");
    pose_bounds.azimuth_min_ = 0;
  }

  if (pose_bounds.azimuth_max_ > 2 * M_PI)
  {
    ROS_WARN_STREAM_NAMED(name_, "max azimuth bound > 2 pi, setting equal to 2 pi ");
    pose_bounds.azimuth_max_ = 2 * M_PI;
  }

  // Position
  pose.translation().x() = dRand(pose_bounds.x_min_, pose_bounds.x_max_);
  pose.translation().y() = dRand(pose_bounds.y_min_, pose_bounds.y_max_);
  pose.translation().z() = dRand(pose_bounds.z_min_, pose_bounds.z_max_);

  // Random orientation (random rotation axis from unit sphere and random angle)
  double angle = dRand(pose_bounds.angle_min_, pose_bounds.angle_max_);
  double elevation = dRand(pose_bounds.elevation_min_, pose_bounds.elevation_max_);
  double azimuth = dRand(pose_bounds.azimuth_min_, pose_bounds.azimuth_max_);

  Eigen::Vector3d axis;
  axis[0] = sin(elevation) * cos(azimuth);
  axis[1] = sin(elevation) * sin(azimuth);
  axis[2] = cos(elevation);

  Eigen::Quaterniond quaternion(Eigen::AngleAxis<double>(static_cast<double>(angle), axis));
  pose = Eigen::Translation3d(pose.translation().x(), pose.translation().y(), pose.translation().z()) * quaternion;
}

void RvizVisualTools::generateEmptyPose(geometry_msgs::Pose &pose)
{
  // Position
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
}

bool RvizVisualTools::posesEqual(const Eigen::Affine3d &pose1, const Eigen::Affine3d &pose2, const double& threshold)
{
  static const std::size_t NUM_VARS = 16;  // size of eigen matrix

  for (std::size_t i = 0; i < NUM_VARS; ++i)
  {
    if (fabs(pose1.data()[i] - pose2.data()[i]) > threshold)
    {
      return false;
    }
  }

  return true;
}

double RvizVisualTools::dRand(double dMin, double dMax)
{
  double d = static_cast<double>(rand()) / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

float RvizVisualTools::fRand(float dMin, float dMax)
{
  float d = static_cast<float>(rand()) / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

int RvizVisualTools::iRand(int min, int max)
{
  int n = max - min + 1;
  int remainder = RAND_MAX % n;
  int x;
  do
  {
    x = rand();
  } while (x >= RAND_MAX - remainder);
  return min + x % n;
}

void RvizVisualTools::enableInternalBatchPublishing(bool enable)
{
  // Don't interfere with external batch publishing
  if (batch_publishing_enabled_)
  {
    return;
  }
  internal_batch_publishing_enabled_ = true;
}

bool RvizVisualTools::triggerInternalBatchPublishAndDisable()
{
  internal_batch_publishing_enabled_ = false;

  bool result = publishMarkers(markers_);

  markers_.markers.clear();  // remove all cached markers
  return result;
}

void RvizVisualTools::printTransform(const Eigen::Affine3d &transform)
{
  Eigen::Quaterniond q(transform.rotation());
  std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
            << transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", "
            << q.w() << "]" << std::endl;
}

void RvizVisualTools::printTransformRPY(const Eigen::Affine3d &transform)
{
  double x, y, z, r, p, yaw;
  convertToXYZRPY(transform, x, y, z, r, p, yaw);
  std::cout << "transform: [" << x << ", " << y << ", " << z << ", " << r << ", " << p << ", " << yaw << "]\n";
}

}  // namespace rviz_visual_tools
