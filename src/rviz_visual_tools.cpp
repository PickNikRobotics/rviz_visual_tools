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

/* Author: Dave Coleman <dave@picknik.ai>, Andy McEvoy
   Desc:   Helper functions for displaying basic shape markers in Rviz
*/

#include <rviz_visual_tools/rviz_visual_tools.h>

// Conversions
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

// C++
#include <chrono>
#include <cmath>  // for random poses
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rviz_visual_tools
{
const std::string LOGNAME = "visual_tools";

// DEPRECATED, remove in Melodic after Dec 2018 release or so
const std::string RvizVisualTools::NAME = "visual_tools";

const std::array<colors, 14> RvizVisualTools::ALL_RAND_COLORS = { RED,        GREEN,  BLUE,   GREY,   DARK_GREY,
                                                                  WHITE,      ORANGE, YELLOW, BROWN,  PINK,
                                                                  LIME_GREEN, PURPLE, CYAN,   MAGENTA };

std::mt19937 RvizVisualTools::mt_random_engine_(std::chrono::system_clock::now().time_since_epoch().count());

RvizVisualTools::RvizVisualTools(std::string base_frame, std::string marker_topic, ros::NodeHandle nh)
  : nh_(nh), marker_topic_(std::move(marker_topic)), base_frame_(std::move(base_frame))
{
  initialize();
}

void RvizVisualTools::initialize()
{
  marker_lifetime_ = ros::Duration(0.0);  // 0 - unlimited

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
  reset_marker_.ns = "deleteAllMarkers";  // helps during debugging
  reset_marker_.action = 3;               // TODO(davetcoleman): In ROS-J set to visualization_msgs::Marker::DELETEALL;
  reset_marker_.pose.orientation.w = 1;

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
  // Constants
  arrow_marker_.pose = getIdentityPose();

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
  // Constants
  cuboid_marker_.pose = getIdentityPose();

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
  // Constants
  line_strip_marker_.pose = getIdentityPose();

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
  line_list_marker_.pose = getIdentityPose();

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
  spheres_marker_.pose = getIdentityPose();

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
  // Constants
  block_marker_.pose = getIdentityPose();

  // Load Cylinder ----------------------------------------------------
  cylinder_marker_.header.frame_id = base_frame_;
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  // Lifetime
  cylinder_marker_.lifetime = marker_lifetime_;
  // Constants
  cylinder_marker_.pose = getIdentityPose();

  // Load Mesh ----------------------------------------------------
  mesh_marker_.header.frame_id = base_frame_;

  // Set the marker action.  Options are ADD and DELETE
  mesh_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  mesh_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  // Lifetime
  mesh_marker_.lifetime = marker_lifetime_;
  // Constants
  mesh_marker_.pose = getIdentityPose();

  // Load Sphere -------------------------------------------------
  sphere_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  sphere_marker_.ns = "Sphere";
  // Set the marker type.
  sphere_marker_.type = visualization_msgs::Marker::SPHERE;
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
  // Add the point pair to the line message
  sphere_marker_.points.resize(1);
  sphere_marker_.colors.resize(1);
  // Lifetime
  sphere_marker_.lifetime = marker_lifetime_;
  // Constants
  sphere_marker_.pose = getIdentityPose();

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
  // Constants
  text_marker_.pose = getIdentityPose();

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
  // Constants
  triangle_marker_.pose = getIdentityPose();

  return true;
}

void RvizVisualTools::loadMarkerPub(bool wait_for_subscriber, bool latched)
{
  if (pub_rviz_markers_ != nullptr)
  {
    return;
  }

  // Rviz marker publisher
  pub_rviz_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10, latched);
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Publishing Rviz markers on topic " << pub_rviz_markers_.getTopic());

  if (wait_for_subscriber)
  {
    waitForSubscriber(pub_rviz_markers_);
  }
}

void RvizVisualTools::waitForMarkerPub()
{
  bool blocking = true;
  waitForSubscriber(pub_rviz_markers_, 0, blocking);
}

void RvizVisualTools::waitForMarkerPub(double wait_time)
{
  bool blocking = false;
  waitForSubscriber(pub_rviz_markers_, wait_time, blocking);
}

bool RvizVisualTools::waitForSubscriber(const ros::Publisher& pub, double wait_time, bool blocking)
{
  // Will wait at most this amount of time
  ros::Time max_time(ros::Time::now() + ros::Duration(wait_time));

  // This is wrong. It returns only the number of subscribers that have already
  // established their direct connections to this publisher
  int num_existing_subscribers = pub.getNumSubscribers();

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  if (pub.getTopic().empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "loadMarkerPub() has not been called yet, unable to wait for subscriber.");
  }

  if (blocking && num_existing_subscribers == 0)
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Topic '" << pub.getTopic() << "' waiting for subscriber...");
  }

  // Wait for subscriber
  while (num_existing_subscribers == 0)
  {
    if (!blocking && ros::Time::now() > max_time)  // Check if timed out
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Topic '" << pub.getTopic() << "' unable to connect to any subscribers within "
                                               << wait_time
                                               << " sec. It is possible initially published visual messages "
                                                  "will be lost.");
      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    // Check again
    num_existing_subscribers = pub.getNumSubscribers();

    if (!ros::ok())
    {
      return false;
    }
  }
  pub_rviz_markers_connected_ = true;

  return true;
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
  const int rand_num = iRand(0, ALL_RAND_COLORS.size() - 1);
  return ALL_RAND_COLORS[rand_num];
}

std_msgs::ColorRGBA RvizVisualTools::getColor(colors color) const
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
      break;
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
      ROS_WARN_STREAM_NAMED(LOGNAME, "The 'DEFAULT' color should probably not "
                                     "be used with getColor(). Defaulting to "
                                     "blue.");
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
      result.a = alpha_;
      break;
  }

  return result;
}

colors RvizVisualTools::intToRvizColor(std::size_t color)
{
  // clang-format off
  switch (color)
  {
    case 0: return BLACK; break;
    case 1: return BROWN; break;
    case 2: return BLUE; break;
    case 3: return CYAN; break;
    case 4: return GREY; break;
    case 5: return DARK_GREY; break;
    case 6: return GREEN; break;
    case 7: return LIME_GREEN; break;
    case 8: return MAGENTA; break;
    case 9: return ORANGE; break;
    case 10: return PURPLE; break;
    case 11: return RED; break;
    case 12: return PINK; break;
    case 13: return WHITE; break;
    case 14: return YELLOW; break;
    case 15: return TRANSLUCENT; break;
    case 16: return TRANSLUCENT_LIGHT; break;
    case 17: return TRANSLUCENT_DARK; break;
    case 18: return RAND; break;
    case 19: return CLEAR; break;
    case 20: return DEFAULT; break;
  }
  // clang-format on
  return DEFAULT;
}

scales RvizVisualTools::intToRvizScale(std::size_t scale)
{
  // clang-format off
  switch (scale)
  {
    case 1: return XXXXSMALL; break;
    case 2: return XXXSMALL; break;
    case 3: return XXSMALL; break;
    case 4: return XSMALL; break;
    case 5: return SMALL; break;
    case 6: return MEDIUM; break;
    case 7: return LARGE; break;
    case 8: return XLARGE; break;
    case 9: return XXLARGE; break;
    case 10: return XXXLARGE; break;
    case 11: return XXXXLARGE; break;
    default: throw std::runtime_error("Unknown size");
  }
  // clang-format on
  return MEDIUM;  // dumy value
}

std::string RvizVisualTools::scaleToString(scales scale)
{
  // clang-format off
  switch (scale)
  {
    case XXXXSMALL: return "XXXXSMALL"; break;
    case XXXSMALL: return "XXXSMALL"; break;
    case XXSMALL: return "XXSMALL"; break;
    case XSMALL: return "XSMALL"; break;
    case SMALL: return "SMALL"; break;
    case MEDIUM: return "MEDIUM"; break;
    case LARGE: return "LARGE"; break;
    case XLARGE: return "XLARGE"; break;
    case XXLARGE: return "XXLARGE"; break;
    case XXXLARGE: return "XXXLARGE"; break;
    case XXXXLARGE: return "XXXXLARGE"; break;
    default: throw std::runtime_error("Unknown size");
  }
  // clang-format on
  return "MEDIUM";  // dumy value
}

std_msgs::ColorRGBA RvizVisualTools::createRandColor() const
{
  std_msgs::ColorRGBA result;

  const std::size_t max_attempts = 20;  // bound the performance
  std::size_t attempts = 0;

  // Make sure color is not *too* dark
  do
  {
    result.r = fRand(0.0, 1.0);
    result.g = fRand(0.0, 1.0);
    result.b = fRand(0.0, 1.0);
    // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Looking for random color that is not too light, current value is "
    //<< (result.r + result.g + result.b) << " attempt #" << attempts);
    attempts++;
    if (attempts > max_attempts)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Unable to find appropriate random color after " << max_attempts << " attempts");
      break;
    }
  } while (result.r + result.g + result.b < 1.5);  // 3 would be white

  // Set alpha value
  result.a = alpha_;

  return result;
}

double RvizVisualTools::slerp(double start, double end, double range, double value)
{
  return start + (((end - start) / range) * value);
}

std_msgs::ColorRGBA RvizVisualTools::getColorScale(double value) const
{
  // User warning
  if (value < 0)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Intensity value for color scale is below range [0,1], value: " << value);
    value = 0;
  }
  else if (value > 1)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Intensity value for color scale is above range [0,1], value: " << value);
    value = 1;
  }

  std_msgs::ColorRGBA start;
  std_msgs::ColorRGBA end;

  // For second half of color range move towards RED
  if (value == 0.0)
  {
    return getColor(RED);
  }
  else if (value == 1.0)
  {
    return getColor(GREEN);
  }
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

geometry_msgs::Vector3 RvizVisualTools::getScale(scales scale, double marker_scale) const
{
  geometry_msgs::Vector3 result;
  double val(0.0);
  switch (scale)
  {
    case XXXXSMALL:
      val = 0.001;
      break;
    case XXXSMALL:
      val = 0.0025;
      break;
    case XXSMALL:
      val = 0.005;
      break;
    case XSMALL:
      val = 0.0065;
      break;
    case SMALL:
      val = 0.0075;
      break;
    case MEDIUM:
      val = 0.01;
      break;
    case LARGE:
      val = 0.025;
      break;
    case XLARGE:
      val = 0.05;
      break;
    case XXLARGE:
      val = 0.075;
      break;
    case XXXLARGE:
      val = 0.1;
      break;
    case XXXXLARGE:
      val = 0.5;
      break;
    default:
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Not implemented yet");
      break;
  }

  // Allows an individual marker size factor and a size factor for all markers
  result.x = val * marker_scale * global_scale_;
  result.y = val * marker_scale * global_scale_;
  result.z = val * marker_scale * global_scale_;

  return result;
}

Eigen::Vector3d RvizVisualTools::getCenterPoint(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
  Eigen::Vector3d center;
  center[0] = (a[0] + b[0]) / 2.0;
  center[1] = (a[1] + b[1]) / 2.0;
  center[2] = (a[2] + b[2]) / 2.0;
  return center;
}

Eigen::Isometry3d RvizVisualTools::getVectorBetweenPoints(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
  bool verbose = false;

  // TODO(davetcoleman): handle the error case when a & b are the same point.
  // currently it returns nan for the quaternion

  // from
  // http://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/

  // Solution pose
  Eigen::Isometry3d result_pose = Eigen::Isometry3d::Identity();

  // Goal pose:
  Eigen::Quaterniond q;
  Eigen::Vector3d axis_vector = b - a;

  if (verbose)
  {
    std::cout << std::endl;
    std::cout << "axis_vector " << std::endl;
    printTranslation(axis_vector);
  }

  axis_vector.normalize();

  if (verbose)
  {
    std::cout << std::endl;
    std::cout << "axis_vector after normalize " << std::endl;
    printTranslation(axis_vector);
  }

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);

  if (verbose)
  {
    std::cout << "right_axis_vector " << std::endl;
    printTranslation(right_axis_vector);
  }

  if (right_axis_vector[0] == 0.0 && right_axis_vector[1] == 0.0 && right_axis_vector[2] == 0.0)
  {
    if (verbose)
    {
      std::cout << "right axis vector is zero " << std::endl;
    }
    result_pose = Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY());
    result_pose.translation() = a;
    return result_pose;
  }

  right_axis_vector.normalize();

  if (verbose)
  {
    std::cout << "right_axis_vector normalized " << std::endl;
    printTranslation(right_axis_vector);
  }

  double theta = axis_vector.dot(up_vector);
  double angle_rotation = -1.0 * acos(theta);

  q = Eigen::AngleAxis<double>(angle_rotation, right_axis_vector);

  q.normalize();

  if (verbose)
  {
    std::cout << "rotation matrix after normalize: " << std::endl;
    std::cout << q.toRotationMatrix() << std::endl;
  }

  // Rotate so that vector points along line
  result_pose = q * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY());
  result_pose.translation() = a;

  return result_pose;
}

bool RvizVisualTools::publishMarker(visualization_msgs::Marker& marker)
{
  // Add single marker to array
  marker.frame_locked = frame_locking_enabled_;
  markers_.markers.push_back(marker);

  // Determine if we should publish now
  if (!batch_publishing_enabled_)
  {
    return trigger();
  }

  return true;
}

void RvizVisualTools::enableBatchPublishing(bool enable)
{
  batch_publishing_enabled_ = enable;
}

void RvizVisualTools::enableFrameLocking(bool enable)
{
  frame_locking_enabled_ = enable;
}

bool RvizVisualTools::triggerEvery(std::size_t queueSize)
{
  if (markers_.markers.size() >= queueSize || queueSize == 0)
  {
    return trigger();
  }
  return false;
}

bool RvizVisualTools::trigger()
{
  if (!batch_publishing_enabled_)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Batch publishing triggered but it was not enabled (unnecessary function call)");
  }
  if (markers_.markers.empty())
  {
    // ROS_WARN_STREAM_NAMED(LOGNAME, "Batch publishing triggered but queue is empty (unnecessary function call)");
    return false;
  }

  bool result = publishMarkers(markers_);

  markers_.markers.clear();  // remove all cached markers
  return result;
}

bool RvizVisualTools::publishMarkers(visualization_msgs::MarkerArray& markers)
{
  if (pub_rviz_markers_ == nullptr)
  {  // always check this before publishing
    loadMarkerPub();
  }

  // Check if connected to a subscriber
  if (!pub_rviz_markers_waited_ && !pub_rviz_markers_connected_)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Waiting for subscribers before publishing markers...");
    waitForSubscriber(pub_rviz_markers_);

    // Only wait for the publisher once, after that just ignore the lack of connection
    pub_rviz_markers_waited_ = true;
  }

  // Check if any actual markers exist to publish
  if (markers.markers.empty())
  {
    return false;
  }

  // Change all markers to be inverted in color
  if (psychedelic_mode_)
  {
    for (auto& marker : markers.markers)
    {
      marker.color.r = 1.0 - marker.color.r;
      marker.color.g = 1.0 - marker.color.g;
      marker.color.b = 1.0 - marker.color.b;
      for (auto& color : marker.colors)
      {
        color.r = 1.0 - color.r;
        color.g = 1.0 - color.g;
        color.b = 1.0 - color.b;
      }
    }
  }

  for (auto& marker : markers.markers)
  {
    double norm = 0;
    norm += marker.pose.orientation.w * marker.pose.orientation.w;
    norm += marker.pose.orientation.x * marker.pose.orientation.x;
    norm += marker.pose.orientation.y * marker.pose.orientation.y;
    norm += marker.pose.orientation.z * marker.pose.orientation.z;
    norm = std::sqrt(norm);
    if (norm < std::numeric_limits<double>::epsilon())
    {
      marker.pose.orientation.w = 1;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
    }
    else
    {
      marker.pose.orientation.w = marker.pose.orientation.w / norm;
      marker.pose.orientation.x = marker.pose.orientation.x / norm;
      marker.pose.orientation.y = marker.pose.orientation.y / norm;
      marker.pose.orientation.z = marker.pose.orientation.z / norm;
    }
  }
  // Publish
  pub_rviz_markers_.publish(markers);
  return true;
}

bool RvizVisualTools::publishCone(const Eigen::Isometry3d& pose, double angle, colors color, double scale)
{
  return publishCone(convertPose(pose), angle, color, scale);
}

bool RvizVisualTools::publishCone(const geometry_msgs::Pose& pose, double angle, colors color, double scale)
{
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = pose;

  geometry_msgs::Point p[3];
  static const double DELTA_THETA = M_PI / 16.0;
  double theta = 0;

  triangle_marker_.points.clear();
  for (std::size_t i = 0; i < 32; i++)
  {
    p[0].x = 0;
    p[0].y = 0;
    p[0].z = 0;

    p[1].x = scale;
    p[1].y = scale * cos(theta) / angle;
    p[1].z = scale * sin(theta) / angle;

    p[2].x = scale;
    p[2].y = scale * cos(theta + DELTA_THETA) / angle;
    p[2].z = scale * sin(theta + DELTA_THETA) / angle;

    triangle_marker_.points.push_back(p[0]);
    triangle_marker_.points.push_back(p[1]);
    triangle_marker_.points.push_back(p[2]);

    theta += DELTA_THETA;
  }

  triangle_marker_.scale.x = 1.0;
  triangle_marker_.scale.y = 1.0;
  triangle_marker_.scale.z = 1.0;

  return publishMarker(triangle_marker_);
}

bool RvizVisualTools::publishABCDPlane(const double A, const double B, const double C, const double D, colors color,
                                       double x_width, double y_width)
{
  // The coefficients A,B,C give the normal to the plane.
  Eigen::Vector3d n(A, B, C);

  // Graphic is centered at this point
  double distance = D / n.norm();
  Eigen::Vector3d center = -distance * n.normalized();

  Eigen::Isometry3d pose;
  pose.translation() = center;

  // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
  Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
  pose.linear() = q.toRotationMatrix();

  double height = 0.001;  // very thin
  publishCuboid(pose, x_width, y_width, height, color);

  return true;
}

bool RvizVisualTools::publishXYPlane(const Eigen::Isometry3d& pose, colors color, double scale)
{
  return publishXYPlane(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishXYPlane(const geometry_msgs::Pose& pose, colors color, double scale)
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

bool RvizVisualTools::publishXZPlane(const Eigen::Isometry3d& pose, colors color, double scale)
{
  return publishXZPlane(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishXZPlane(const geometry_msgs::Pose& pose, colors color, double scale)
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

bool RvizVisualTools::publishYZPlane(const Eigen::Isometry3d& pose, colors color, double scale)
{
  return publishYZPlane(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishYZPlane(const geometry_msgs::Pose& pose, colors color, double scale)
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

bool RvizVisualTools::publishSphere(const Eigen::Isometry3d& pose, colors color, scales scale, const std::string& ns,
                                    std::size_t id)
{
  return publishSphere(convertPose(pose), color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d& point, colors color, scales scale, const std::string& ns,
                                    std::size_t id)
{
  geometry_msgs::Pose pose_msg;
  tf2::convert(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d& point, colors color, double scale, const std::string& ns,
                                    std::size_t id)
{
  geometry_msgs::Pose pose_msg = getIdentityPose();
  tf2::convert(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Point& point, colors color, scales scale,
                                    const std::string& ns, std::size_t id)
{
  geometry_msgs::Pose pose_msg = getIdentityPose();
  pose_msg.position = point;
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose& pose, colors color, scales scale, const std::string& ns,
                                    std::size_t id)
{
  return publishSphere(pose, color, getScale(scale), ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose& pose, colors color, double scale, const std::string& ns,
                                    std::size_t id)
{
  geometry_msgs::Vector3 scale_msg;
  scale_msg.x = scale * global_scale_;
  scale_msg.y = scale * global_scale_;
  scale_msg.z = scale * global_scale_;
  return publishSphere(pose, color, scale_msg, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose& pose, colors color, const geometry_msgs::Vector3 scale,
                                    const std::string& ns, std::size_t id)
{
  return publishSphere(pose, getColor(color), scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Isometry3d& pose, const std_msgs::ColorRGBA& color,
                                    const geometry_msgs::Vector3 scale, const std::string& ns, std::size_t id)
{
  return publishSphere(convertPose(pose), color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d& point, const std_msgs::ColorRGBA& color,
                                    const geometry_msgs::Vector3 scale, const std::string& ns, std::size_t id)
{
  geometry_msgs::Pose pose_msg = getIdentityPose();
  tf2::convert(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns, id);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose& pose, const std_msgs::ColorRGBA& color,
                                    const geometry_msgs::Vector3 scale, const std::string& ns, std::size_t id)
{
  // Set the frame ID and timestamp
  sphere_marker_.header.stamp = ros::Time::now();

  // Overwrite ID or increment?
  if (id == 0)
  {
    sphere_marker_.id++;
  }
  else
  {
    sphere_marker_.id = id;
  }

  sphere_marker_.pose = pose;
  sphere_marker_.color = color;
  sphere_marker_.scale = scale;
  sphere_marker_.ns = ns;

  // Helper for publishing rviz markers
  return publishMarker(sphere_marker_);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::PoseStamped& pose, colors color,
                                    const geometry_msgs::Vector3 scale, const std::string& ns, std::size_t id)
{
  // Set the frame ID and timestamp
  sphere_marker_.header = pose.header;

  if (id == 0)
  {
    sphere_marker_.id++;
  }
  else
  {
    sphere_marker_.id = id;
  }

  sphere_marker_.pose = pose.pose;
  sphere_marker_.color = getColor(color);
  sphere_marker_.scale = scale;
  sphere_marker_.ns = ns;

  // Helper for publishing rviz markers
  publishMarker(sphere_marker_);

  sphere_marker_.header.frame_id = base_frame_;  // restore default frame
  return true;
}

bool RvizVisualTools::publishXArrow(const Eigen::Isometry3d& pose, colors color, scales scale, double length)
{
  return publishArrow(convertPose(pose), color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::Pose& pose, colors color, scales scale, double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::PoseStamped& pose, colors color, scales scale, double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishYArrow(const Eigen::Isometry3d& pose, colors color, scales scale, double length)
{
  Eigen::Isometry3d arrow_pose = pose * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::Pose& pose, colors color, scales scale, double length)
{
  Eigen::Isometry3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::PoseStamped& pose, colors color, scales scale, double length)
{
  Eigen::Isometry3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishZArrow(const Eigen::Isometry3d& pose, colors color, scales scale, double length,
                                    std::size_t id)
{
  Eigen::Isometry3d arrow_pose = pose * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length, id);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::Pose& pose, colors color, scales scale, double length)
{
  Eigen::Isometry3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::PoseStamped& pose, colors color, scales scale, double length)
{
  Eigen::Isometry3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::PoseStamped& pose, colors color, scales scale, double length,
                                    std::size_t id)
{
  Eigen::Isometry3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length, id);
}

bool RvizVisualTools::publishArrow(const Eigen::Isometry3d& pose, colors color, scales scale, double length,
                                   std::size_t id)
{
  return publishArrow(convertPose(pose), color, scale, length, id);
}

bool RvizVisualTools::publishArrow(const geometry_msgs::Pose& pose, colors color, scales scale, double length,
                                   std::size_t id)
{
  // Set the frame ID and timestamp.
  arrow_marker_.header.stamp = ros::Time::now();
  arrow_marker_.header.frame_id = base_frame_;

  if (id == 0)
  {
    arrow_marker_.id++;
  }
  else
  {
    arrow_marker_.id = id;
  }

  arrow_marker_.pose = pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale);

  // override previous x scale specified
  if (length == 0)
  {  // auto set the scale
    arrow_marker_.scale.x *= 10.0;
  }
  else
  {
    arrow_marker_.scale.x = length;
  }

  // Helper for publishing rviz markers
  return publishMarker(arrow_marker_);
}

bool RvizVisualTools::publishArrow(const geometry_msgs::PoseStamped& pose, colors color, scales scale, double length,
                                   std::size_t id)
{
  // Set the frame ID and timestamp.
  arrow_marker_.header = pose.header;

  if (id == 0)
  {
    arrow_marker_.id++;
  }
  else
  {
    arrow_marker_.id = id;
  }

  arrow_marker_.pose = pose.pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale);

  // override previous x scale specified
  if (length == 0)
  {  // auto set the scale
    arrow_marker_.scale.x *= 10.0;
  }
  else
  {
    arrow_marker_.scale.x = length;
  }

  // Helper for publishing rviz markers
  publishMarker(arrow_marker_);

  arrow_marker_.header.frame_id = base_frame_;  // restore default frame
  return true;
}

bool RvizVisualTools::publishArrow(const geometry_msgs::Point& start, const geometry_msgs::Point& end, colors color,
                                   scales scale, std::size_t id)
{
  // Set the frame ID and timestamp.
  arrow_marker_.header.stamp = ros::Time::now();
  arrow_marker_.header.frame_id = base_frame_;

  if (id == 0)
  {
    arrow_marker_.id++;
  }
  else
  {
    arrow_marker_.id = id;
  }

  arrow_marker_.points.clear();
  arrow_marker_.points.push_back(start);
  arrow_marker_.points.push_back(end);
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale);

  // override previous y & z scale specified
  // scale.x is the shaft diameter
  // scale.y is the head diameter
  // scale.z it specifies the head length.
  arrow_marker_.scale.y *= 2.0;
  arrow_marker_.scale.z *= 3.0;

  // Helper for publishing rviz markers
  return publishMarker(arrow_marker_);
}

bool RvizVisualTools::publishAxisLabeled(const Eigen::Isometry3d& pose, const std::string& label, scales scale,
                                         colors color)
{
  return publishAxisLabeled(convertPose(pose), label, scale, color);
}

bool RvizVisualTools::publishAxisLabeled(const geometry_msgs::Pose& pose, const std::string& label, scales scale,
                                         colors color)
{
  publishAxis(pose, scale, label);

  // For avoiding overriden Axis and Text
  geometry_msgs::Pose pose_shifted = pose;
  pose_shifted.position.x -= 0.05;
  pose_shifted.position.y -= 0.05;
  pose_shifted.position.z -= 0.05;
  publishText(pose_shifted, label, color, static_cast<scales>(static_cast<int>(scale) + 1), false);
  return true;
}

bool RvizVisualTools::publishAxis(const geometry_msgs::Pose& pose, scales scale, const std::string& ns)
{
  double radius = getScale(scale).x;
  return publishAxis(pose, radius * 10.0, radius, ns);
}

bool RvizVisualTools::publishAxis(const Eigen::Isometry3d& pose, scales scale, const std::string& ns)
{
  double radius = getScale(scale).x;
  return publishAxis(pose, radius * 10.0, radius, ns);
}

bool RvizVisualTools::publishAxis(const geometry_msgs::Pose& pose, double length, double radius, const std::string& ns)
{
  return publishAxis(convertPose(pose), length, radius, ns);
}

bool RvizVisualTools::publishAxis(const Eigen::Isometry3d& pose, double length, double radius, const std::string& ns)
{
  // Use an internal function that will not actually publish anything, so that other makers can combine with an axis
  // without publishing
  publishAxisInternal(pose, length, radius, ns);

  return true;
}

bool RvizVisualTools::publishAxisInternal(const Eigen::Isometry3d& pose, double length, double radius,
                                          const std::string& ns)
{
  // Publish x axis
  Eigen::Isometry3d x_pose =
      Eigen::Translation3d(length / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  x_pose = pose * x_pose;
  publishCylinder(x_pose, RED, length, radius, ns);

  // Publish y axis
  Eigen::Isometry3d y_pose =
      Eigen::Translation3d(0, length / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  y_pose = pose * y_pose;
  publishCylinder(y_pose, GREEN, length, radius, ns);

  // Publish z axis
  Eigen::Isometry3d z_pose = Eigen::Translation3d(0, 0, length / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  z_pose = pose * z_pose;
  publishCylinder(z_pose, BLUE, length, radius, ns);

  return true;
}

bool RvizVisualTools::publishAxisPath(const EigenSTL::vector_Isometry3d& path, scales scale, const std::string& ns)
{
  // Create the cylinders
  for (const auto& i : path)
  {
    double radius = getScale(scale).x;
    publishAxisInternal(i, radius * 10.0, radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishAxisPath(const EigenSTL::vector_Isometry3d& path, double length, double radius,
                                      const std::string& ns)
{
  // Create the cylinders
  for (const auto& i : path)
  {
    publishAxisInternal(i, length, radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, colors color,
                                      scales scale, const std::string& ns)
{
  double radius = getScale(scale).x;
  return publishCylinder(point1, point2, getColor(color), radius, ns);
}

bool RvizVisualTools::publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, colors color,
                                      double radius, const std::string& ns)
{
  return publishCylinder(point1, point2, getColor(color), radius, ns);
}

bool RvizVisualTools::publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                                      const std_msgs::ColorRGBA& color, double radius, const std::string& ns)
{
  // Distance between two points
  double height = (point1 - point2).lpNorm<2>();

  // Find center point
  Eigen::Vector3d pt_center = getCenterPoint(point1, point2);

  // Create vector
  Eigen::Isometry3d pose;
  pose = getVectorBetweenPoints(pt_center, point2);

  // Convert pose to be normal to cylindar axis
  Eigen::Isometry3d rotation;
  rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;

  // Turn into msg
  return publishCylinder(convertPose(pose), color, height, radius, ns);
}

bool RvizVisualTools::publishCylinder(const Eigen::Isometry3d& pose, colors color, double height, double radius,
                                      const std::string& ns)
{
  return publishCylinder(convertPose(pose), color, height, radius, ns);
}

bool RvizVisualTools::publishCylinder(const geometry_msgs::Pose& pose, colors color, double height, double radius,
                                      const std::string& ns)
{
  return publishCylinder(pose, getColor(color), height, radius, ns);
}

bool RvizVisualTools::publishCylinder(const geometry_msgs::Pose& pose, const std_msgs::ColorRGBA& color, double height,
                                      double radius, const std::string& ns)
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

bool RvizVisualTools::publishMesh(const Eigen::Isometry3d& pose, const std::string& file_name, colors color,
                                  double scale, const std::string& ns, std::size_t id)
{
  return publishMesh(convertPose(pose), file_name, color, scale, ns, id);
}

bool RvizVisualTools::publishMesh(const geometry_msgs::Pose& pose, const std::string& file_name, colors color,
                                  double scale, const std::string& ns, std::size_t id)
{
  // Set the timestamp
  mesh_marker_.header.stamp = ros::Time::now();

  if (id == 0)
  {
    mesh_marker_.id++;
  }
  else
  {
    mesh_marker_.id = id;
  }

  // Set the mesh
  mesh_marker_.mesh_resource = file_name;
  mesh_marker_.mesh_use_embedded_materials = 1u;

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

bool RvizVisualTools::publishMesh(const Eigen::Isometry3d& pose, const shape_msgs::Mesh& mesh, colors color,
                                  double scale, const std::string& ns, std::size_t id)
{
  return publishMesh(convertPose(pose), mesh, color, scale, ns, id);
}

bool RvizVisualTools::publishMesh(const geometry_msgs::Pose& pose, const shape_msgs::Mesh& mesh, colors color,
                                  double scale, const std::string& ns, std::size_t id)
{
  triangle_marker_.header.stamp = ros::Time::now();

  if (id == 0)
  {
    triangle_marker_.id++;
  }
  else
  {
    triangle_marker_.id = id;
  }

  // Set the mesh
  triangle_marker_.points.clear();
  for (const shape_msgs::MeshTriangle& triangle : mesh.triangles)
    for (const uint32_t& index : triangle.vertex_indices)
      triangle_marker_.points.push_back(mesh.vertices[index]);

  // Set the pose
  triangle_marker_.pose = pose;

  // Set marker size
  triangle_marker_.scale.x = scale;
  triangle_marker_.scale.y = scale;
  triangle_marker_.scale.z = scale;

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  triangle_marker_.ns = ns;

  // Set marker color
  triangle_marker_.color = getColor(color);

  // Helper for publishing rviz markers
  return publishMarker(triangle_marker_);
}

bool RvizVisualTools::publishGraph(const graph_msgs::GeometryGraph& graph, colors color, double radius)
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
      if (!return_value.second)
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

bool RvizVisualTools::publishCuboid(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, colors color)
{
  return publishCuboid(convertPoint(point1), convertPoint(point2), color);
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                    colors color, const std::string& ns, std::size_t id)
{
  // Set the timestamp
  cuboid_marker_.header.stamp = ros::Time::now();
  cuboid_marker_.ns = ns;

  if (id == 0)
  {  // Provide a new id every call to this function
    cuboid_marker_.id++;
  }
  else
  {  // allow marker to be overwritten
    cuboid_marker_.id = id;
  }

  cuboid_marker_.color = getColor(color);

  // Calculate center pose
  geometry_msgs::Pose pose = getIdentityPose();
  pose.position.x = (point1.x - point2.x) / 2.0 + point2.x;
  pose.position.y = (point1.y - point2.y) / 2.0 + point2.y;
  pose.position.z = (point1.z - point2.z) / 2.0 + point2.z;
  cuboid_marker_.pose = pose;

  // Calculate scale
  cuboid_marker_.scale.x = fabs(point1.x - point2.x);
  cuboid_marker_.scale.y = fabs(point1.y - point2.y);
  cuboid_marker_.scale.z = fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (cuboid_marker_.scale.x == 0.0)
  {
    cuboid_marker_.scale.x = SMALL_SCALE;
  }
  if (cuboid_marker_.scale.y == 0.0)
  {
    cuboid_marker_.scale.y = SMALL_SCALE;
  }
  if (cuboid_marker_.scale.z == 0.0)
  {
    cuboid_marker_.scale.z = SMALL_SCALE;
  }

  // Helper for publishing rviz markers
  return publishMarker(cuboid_marker_);
}

bool RvizVisualTools::publishCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                                    colors color)
{
  return publishCuboid(convertPose(pose), depth, width, height, color);
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Pose& pose, double depth, double width, double height,
                                    colors color)
{
  cuboid_marker_.header.stamp = ros::Time::now();

  cuboid_marker_.id++;
  cuboid_marker_.color = getColor(color);

  cuboid_marker_.pose = pose;

  // Prevent scale from being zero
  if (depth <= 0)
  {
    cuboid_marker_.scale.x = SMALL_SCALE;
  }
  else
  {
    cuboid_marker_.scale.x = depth;
  }

  if (width <= 0)
  {
    cuboid_marker_.scale.y = SMALL_SCALE;
  }
  else
  {
    cuboid_marker_.scale.y = width;
  }

  if (height <= 0)
  {
    cuboid_marker_.scale.z = SMALL_SCALE;
  }
  else
  {
    cuboid_marker_.scale.z = height;
  }

  return publishMarker(cuboid_marker_);
}

bool RvizVisualTools::publishCuboid(const Eigen::Isometry3d& pose, const Eigen::Vector3d& size, colors color)
{
  return publishCuboid(pose, size.x(), size.y(), size.z(), color);
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size, colors color)
{
  return publishCuboid(pose, size.x, size.y, size.z, color);
}

bool RvizVisualTools::publishLine(const Eigen::Isometry3d& point1, const Eigen::Isometry3d& point2, colors color,
                                  scales scale)
{
  return publishLine(convertPoseToPoint(point1), convertPoseToPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, colors color,
                                  scales scale)
{
  return publishLine(convertPoint(point1), convertPoint(point2), color, scale);
}
bool RvizVisualTools::publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, colors color,
                                  double radius)
{
  geometry_msgs::Vector3 scale;
  scale.x = radius * global_scale_;
  scale.y = radius * global_scale_;
  scale.z = radius * global_scale_;
  return publishLine(convertPoint(point1), convertPoint(point2), getColor(color), scale);
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                                  const std_msgs::ColorRGBA& color, scales scale)
{
  return publishLine(convertPoint(point1), convertPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                                  const std_msgs::ColorRGBA& color, double radius)
{
  geometry_msgs::Vector3 scale;
  scale.x = radius * global_scale_;
  scale.y = radius * global_scale_;
  scale.z = radius * global_scale_;
  return publishLine(convertPoint(point1), convertPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, colors color,
                                  scales scale)
{
  return publishLine(point1, point2, getColor(color), scale);
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                  const std_msgs::ColorRGBA& color, scales scale)
{
  return publishLine(point1, point2, color, getScale(scale));
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                  const std_msgs::ColorRGBA& color, const geometry_msgs::Vector3& scale)
{
  // Set the timestamp
  line_strip_marker_.header.stamp = ros::Time::now();

  line_strip_marker_.id++;
  line_strip_marker_.color = color;
  line_strip_marker_.scale = scale;
  line_strip_marker_.scale.y = 0;
  line_strip_marker_.scale.z = 0;

  line_strip_marker_.points.clear();
  line_strip_marker_.points.push_back(point1);
  line_strip_marker_.points.push_back(point2);

  // Helper for publishing rviz markers
  return publishMarker(line_strip_marker_);
}

bool RvizVisualTools::publishLines(const EigenSTL::vector_Vector3d& aPoints, const EigenSTL::vector_Vector3d& bPoints,
                                   const std::vector<colors>& colors, scales scale)
{
  BOOST_ASSERT_MSG(aPoints.size() == bPoints.size() && bPoints.size() == colors.size(), "Mismatching vector sizes: "
                                                                                        "aPoints, bPoints, and colors");

  std::vector<geometry_msgs::Point> a_points_msg;
  std::vector<geometry_msgs::Point> b_points_msg;
  std::vector<std_msgs::ColorRGBA> colors_msg;

  for (std::size_t i = 0; i < aPoints.size(); ++i)
  {
    // Convert Eigen to ROS Msg
    a_points_msg.push_back(convertPoint(aPoints[i]));
    b_points_msg.push_back(convertPoint(bPoints[i]));
    // Convert color to ROS Msg
    colors_msg.push_back(getColor(colors[i]));
  }
  BOOST_ASSERT_MSG(a_points_msg.size() == b_points_msg.size() && b_points_msg.size() == colors_msg.size(),
                   "Mismatched "
                   "vector sizes");

  return publishLines(a_points_msg, b_points_msg, colors_msg, getScale(scale));
}

bool RvizVisualTools::publishLines(const std::vector<geometry_msgs::Point>& aPoints,
                                   const std::vector<geometry_msgs::Point>& bPoints,
                                   const std::vector<std_msgs::ColorRGBA>& colors, const geometry_msgs::Vector3& scale)
{
  // Setup marker
  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = "Line Array";

  // Provide a new id every call to this function
  line_list_marker_.id++;

  line_list_marker_.scale = scale;
  line_list_marker_.scale.z = 0;
  line_list_marker_.scale.y = 0;

  // line_list_marker_.color = getColor(BLUE); // This var is not used

  // Add each point pair to the line message
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();
  for (std::size_t i = 0; i < aPoints.size(); ++i)
  {
    line_list_marker_.points.push_back(aPoints[i]);
    line_list_marker_.points.push_back(bPoints[i]);
    line_list_marker_.colors.push_back(colors[i]);
    line_list_marker_.colors.push_back(colors[i]);
  }

  // Testing
  BOOST_ASSERT_MSG(line_list_marker_.colors.size() == line_list_marker_.points.size(), "Arrays mismatch in size");
  BOOST_ASSERT_MSG(line_list_marker_.colors.size() == aPoints.size() * 2, "Colors arrays mismatch in size");

  // Helper for publishing rviz markers
  return publishMarker(line_list_marker_);
}

bool RvizVisualTools::publishLineStrip(const std::vector<geometry_msgs::Point>& path, colors color, scales scale,
                                       const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Skipping path because " << path.size() << " points passed in.");
    return true;
  }

  line_strip_marker_.header.stamp = ros::Time();
  line_strip_marker_.ns = ns;

  // Provide a new id every call to this function
  line_strip_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_strip_marker_.scale = getScale(scale);
  line_strip_marker_.scale.z = 0;
  line_strip_marker_.scale.y = 0;
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

bool RvizVisualTools::publishPath(const std::vector<geometry_msgs::Pose>& path, colors color, scales scale,
                                  const std::string& ns)
{
  std::vector<geometry_msgs::Point> point_path(path.size());
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    point_path[i] = path[i].position;
  }

  return publishPath(point_path, color, getScale(scale).x, ns);
}

bool RvizVisualTools::publishPath(const std::vector<geometry_msgs::Point>& path, colors color, scales scale,
                                  const std::string& ns)
{
  return publishPath(path, color, getScale(scale).x, ns);
}

bool RvizVisualTools::publishPath(const EigenSTL::vector_Isometry3d& path, colors color, scales scale,
                                  const std::string& ns)
{
  return publishPath(path, color, getScale(scale).x, ns);
}

bool RvizVisualTools::publishPath(const EigenSTL::vector_Vector3d& path, colors color, scales scale,
                                  const std::string& ns)
{
  return publishPath(path, color, getScale(scale).x, ns);
}

bool RvizVisualTools::publishPath(const std::vector<geometry_msgs::Point>& path, colors color, double radius,
                                  const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Skipping path because " << path.size() << " points passed in.");
    return false;
  }

  // Create the cylinders
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    publishCylinder(convertPoint(path[i - 1]), convertPoint(path[i]), color, radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishPath(const EigenSTL::vector_Vector3d& path, colors color, double radius,
                                  const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Skipping path because " << path.size() << " points passed in.");
    return false;
  }

  // Create the cylinders
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    publishCylinder(path[i - 1], path[i], color, radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishPath(const EigenSTL::vector_Isometry3d& path, colors color, double radius,
                                  const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Skipping path because " << path.size() << " points passed in.");
    return false;
  }

  // Create the cylinders
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    publishCylinder(path[i - 1].translation(), path[i].translation(), color, radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishPath(const EigenSTL::vector_Vector3d& path, const std::vector<colors>& colors,
                                  double radius, const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Skipping path because " << path.size() << " points passed in.");
    return false;
  }

  if (path.size() != colors.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Skipping path because " << path.size() << " different from " << colors.size() << ".");
    return false;
  }

  // Create the cylinders
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    publishCylinder(path[i - 1], path[i], colors[i], radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishPath(const EigenSTL::vector_Vector3d& path, const std::vector<std_msgs::ColorRGBA>& colors,
                                  double radius, const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Skipping path because " << path.size() << " points passed in.");
    return false;
  }

  if (path.size() != colors.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Skipping path because " << path.size() << " different from " << colors.size() << ".");
    return false;
  }

  // Create the cylinders
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    publishCylinder(path[i - 1], path[i], colors[i], radius, ns);
  }

  return true;
}

bool RvizVisualTools::publishPolygon(const geometry_msgs::Polygon& polygon, colors color, scales scale,
                                     const std::string& ns)
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
    {
      first = temp;
    }
    points.push_back(temp);
  }
  points.push_back(first);  // connect first and last points for last line

  return publishPath(points, color, scale, ns);
}

bool RvizVisualTools::publishWireframeCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                                             colors color, const std::string& ns, std::size_t id)
{
  Eigen::Vector3d min_point, max_point;
  min_point << -depth / 2, -width / 2, -height / 2;
  max_point << depth / 2, width / 2, height / 2;
  return publishWireframeCuboid(pose, min_point, max_point, color, ns, id);
}

bool RvizVisualTools::publishWireframeCuboid(const Eigen::Isometry3d& pose, const Eigen::Vector3d& min_point,
                                             const Eigen::Vector3d& max_point, colors color, const std::string& ns,
                                             std::size_t id)
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

  if (id == 0)
  {  // Provide a new id every call to this function
    line_list_marker_.id++;
  }
  else
  {  // allow marker to be overwritten
    line_list_marker_.id = id;
  }

  std_msgs::ColorRGBA this_color = getColor(color);
  line_list_marker_.scale = getScale(XXSMALL);
  line_list_marker_.scale.y = 0;
  line_list_marker_.scale.z = 0;
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

bool RvizVisualTools::publishWireframeRectangle(const Eigen::Isometry3d& pose, double height, double width,
                                                colors color, scales scale, std::size_t id)
{
  if (id == 0)
  {  // Provide a new id every call to this function
    line_list_marker_.id++;
  }
  else
  {  // allow marker to be overwritten
    line_list_marker_.id = id;
  }

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

  std_msgs::ColorRGBA this_color = getColor(color);
  line_list_marker_.scale = getScale(scale, 0.25);
  line_list_marker_.scale.y = 0;
  line_list_marker_.scale.z = 0;

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

bool RvizVisualTools::publishWireframeRectangle(const Eigen::Isometry3d& pose, const Eigen::Vector3d& p1_in,
                                                const Eigen::Vector3d& p2_in, const Eigen::Vector3d& p3_in,
                                                const Eigen::Vector3d& p4_in, colors color, scales scale)
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
  line_list_marker_.scale = getScale(scale, 0.25);
  line_list_marker_.scale.y = 0;
  line_list_marker_.scale.z = 0;
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

bool RvizVisualTools::publishSpheres(const EigenSTL::vector_Vector3d& points, colors color, scales scale,
                                     const std::string& ns)
{
  std::vector<geometry_msgs::Point> points_msg;

  for (const auto& point : points)
  {
    points_msg.push_back(convertPoint(point));
  }

  return publishSpheres(points_msg, color, scale, ns);
}

bool RvizVisualTools::publishSpheres(const EigenSTL::vector_Vector3d& points, colors color, double scale,
                                     const std::string& ns)
{
  std::vector<geometry_msgs::Point> points_msg;

  for (const auto& point : points)
  {
    points_msg.push_back(convertPoint(point));
  }

  return publishSpheres(points_msg, color, scale, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point>& points, colors color, double scale,
                                     const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale * global_scale_;
  scale_vector.y = scale * global_scale_;
  scale_vector.z = scale * global_scale_;
  return publishSpheres(points, color, scale_vector, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point>& points, colors color, scales scale,
                                     const std::string& ns)
{
  return publishSpheres(points, color, getScale(scale), ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point>& points, colors color,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  spheres_marker_.header.stamp = ros::Time();
  spheres_marker_.ns = ns;

  // Provide a new id every call to this function
  spheres_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  spheres_marker_.scale = scale;
  spheres_marker_.color = this_color;
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

bool RvizVisualTools::publishSpheres(const EigenSTL::vector_Vector3d& points, const std::vector<colors>& colors,
                                     scales scale, const std::string& ns)
{
  BOOST_ASSERT_MSG(points.size() == colors.size(), "Mismatching vector sizes: points and colors");

  std::vector<geometry_msgs::Point> points_msg;
  std::vector<std_msgs::ColorRGBA> colors_msg;

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    // Convert Eigen to ROS Msg
    points_msg.push_back(convertPoint(points[i]));
    // Convert color to ROS Msg
    colors_msg.push_back(getColor(colors[i]));
  }

  return publishSpheres(points_msg, colors_msg, getScale(scale), ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point>& points,
                                     const std::vector<std_msgs::ColorRGBA>& colors,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  spheres_marker_.header.stamp = ros::Time();
  spheres_marker_.ns = ns;

  // Provide a new id every call to this function
  spheres_marker_.id++;

  spheres_marker_.scale = scale;
  spheres_marker_.points = points;  // straight copy
  spheres_marker_.colors = colors;  // straight copy

  // Helper for publishing rviz markers
  return publishMarker(spheres_marker_);
}

bool RvizVisualTools::publishText(const Eigen::Isometry3d& pose, const std::string& text, colors color, scales scale,
                                  bool static_id)
{
  return publishText(convertPose(pose), text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const Eigen::Isometry3d& pose, const std::string& text, colors color,
                                  const geometry_msgs::Vector3 scale, bool static_id)
{
  return publishText(convertPose(pose), text, color, scale, static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose& pose, const std::string& text, colors color, scales scale,
                                  bool static_id)
{
  return publishText(pose, text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose& pose, const std::string& text, colors color,
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
  text_marker_.scale.x = 0;
  text_marker_.scale.y = 0;
  // Helper for publishing rviz markers
  publishMarker(text_marker_);

  // Restore the ID count if needed
  if (static_id)
  {
    text_marker_.id = temp_id;
  }

  return true;
}

geometry_msgs::Pose RvizVisualTools::convertPose(const Eigen::Isometry3d& pose)
{
  geometry_msgs::Pose pose_msg;
  tf2::convert(pose, pose_msg);
  return pose_msg;
}

void RvizVisualTools::convertPoseSafe(const Eigen::Isometry3d& pose, geometry_msgs::Pose& pose_msg)
{
  tf2::convert(pose, pose_msg);
}

Eigen::Isometry3d RvizVisualTools::convertPose(const geometry_msgs::Pose& pose)
{
  Eigen::Isometry3d shared_pose_eigen;
  tf2::convert(pose, shared_pose_eigen);
  return shared_pose_eigen;
}

void RvizVisualTools::convertPoseSafe(const geometry_msgs::Pose& pose_msg, Eigen::Isometry3d& pose)
{
  tf2::convert(pose_msg, pose);
}

Eigen::Isometry3d RvizVisualTools::convertPoint32ToPose(const geometry_msgs::Point32& point)
{
  Eigen::Isometry3d shared_pose_eigen;
  shared_pose_eigen = Eigen::Isometry3d::Identity();
  shared_pose_eigen.translation().x() = point.x;
  shared_pose_eigen.translation().y() = point.y;
  shared_pose_eigen.translation().z() = point.z;
  return shared_pose_eigen;
}

geometry_msgs::Pose RvizVisualTools::convertPointToPose(const geometry_msgs::Point& point)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.orientation.x = 0.0;
  pose_msg.orientation.y = 0.0;
  pose_msg.orientation.z = 0.0;
  pose_msg.orientation.w = 1.0;
  pose_msg.position = point;
  return pose_msg;
}

Eigen::Isometry3d RvizVisualTools::convertPointToPose(const Eigen::Vector3d& point)
{
  Eigen::Isometry3d shared_pose_eigen;
  shared_pose_eigen = Eigen::Isometry3d::Identity();
  shared_pose_eigen.translation() = point;
  return shared_pose_eigen;
}

geometry_msgs::Point RvizVisualTools::convertPoseToPoint(const Eigen::Isometry3d& pose)
{
  geometry_msgs::Pose pose_msg;
  tf2::convert(pose, pose_msg);
  return pose_msg.position;
}

Eigen::Vector3d RvizVisualTools::convertPoint(const geometry_msgs::Point& point)
{
  Eigen::Vector3d point_eigen;
  point_eigen[0] = point.x;
  point_eigen[1] = point.y;
  point_eigen[2] = point.z;
  return point_eigen;
}

Eigen::Vector3d RvizVisualTools::convertPoint32(const geometry_msgs::Point32& point)
{
  Eigen::Vector3d point_eigen;
  point_eigen[0] = point.x;
  point_eigen[1] = point.y;
  point_eigen[2] = point.z;
  return point_eigen;
}

geometry_msgs::Point32 RvizVisualTools::convertPoint32(const Eigen::Vector3d& point)
{
  geometry_msgs::Point32 point32_msg;
  point32_msg.x = point[0];
  point32_msg.y = point[1];
  point32_msg.z = point[2];
  return point32_msg;
}

geometry_msgs::Point RvizVisualTools::convertPoint(const geometry_msgs::Vector3& point)
{
  geometry_msgs::Point point_msg;
  point_msg.x = point.x;
  point_msg.y = point.y;
  point_msg.z = point.z;
  return point_msg;
}

geometry_msgs::Point RvizVisualTools::convertPoint(const Eigen::Vector3d& point)
{
  geometry_msgs::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  return point_msg;
}

Eigen::Isometry3d RvizVisualTools::convertFromXYZRPY(double tx, double ty, double tz, double rx, double ry, double rz,
                                                     EulerConvention convention)
{
  Eigen::Isometry3d result;

  switch (convention)
  {
    case XYZ:
      result = Eigen::Translation3d(tx, ty, tz) * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
      break;

    case ZYX:
      result = Eigen::Translation3d(tx, ty, tz) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
      break;

    case ZXZ:
      result = Eigen::Translation3d(tx, ty, tz) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
      break;

    default:
      ROS_ERROR_STREAM("Invalid euler convention entry " << convention);
      break;
  }

  return result;
}

Eigen::Isometry3d RvizVisualTools::convertFromXYZRPY(const std::vector<double>& transform6, EulerConvention convention)
{
  if (transform6.size() != 6)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Incorrect number of variables passed for 6-size transform");
    throw;
  }

  return convertFromXYZRPY(transform6[0], transform6[1], transform6[2], transform6[3], transform6[4], transform6[5],
                           convention);
}

void RvizVisualTools::convertToXYZRPY(const Eigen::Isometry3d& pose, std::vector<double>& xyzrpy)
{
  xyzrpy.resize(6);
  convertToXYZRPY(pose, xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
}

void RvizVisualTools::convertToXYZRPY(const Eigen::Isometry3d& pose, double& x, double& y, double& z, double& roll,
                                      double& pitch, double& yaw)
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

void RvizVisualTools::generateRandomPose(geometry_msgs::Pose& pose, RandomPoseBounds pose_bounds)
{
  Eigen::Isometry3d pose_eigen;
  generateRandomPose(pose_eigen, pose_bounds);
  pose = convertPose(pose_eigen);
}

void RvizVisualTools::generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& depth, double& width,
                                           double& height, RandomPoseBounds pose_bounds,
                                           RandomCuboidBounds cuboid_bounds)
{
  // Size
  depth = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);
  width = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);
  height = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);

  // Orientation
  generateRandomPose(cuboid_pose, pose_bounds);
}

void RvizVisualTools::generateRandomPose(Eigen::Isometry3d& pose, RandomPoseBounds pose_bounds)
{
  // Error check elevation & azimuth angles
  // 0 <= elevation <= pi
  // 0 <= azimuth   <= 2 * pi
  if (pose_bounds.elevation_min_ < 0)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "min elevation bound < 0, setting equal to 0");
    pose_bounds.elevation_min_ = 0;
  }

  if (pose_bounds.elevation_max_ > M_PI)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "max elevation bound > pi, setting equal to pi ");
    pose_bounds.elevation_max_ = M_PI;
  }

  if (pose_bounds.azimuth_min_ < 0)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "min azimuth bound < 0, setting equal to 0");
    pose_bounds.azimuth_min_ = 0;
  }

  if (pose_bounds.azimuth_max_ > 2 * M_PI)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "max azimuth bound > 2 pi, setting equal to 2 pi ");
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

geometry_msgs::Pose RvizVisualTools::getIdentityPose()
{
  geometry_msgs::Pose pose;
  // Position
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

bool RvizVisualTools::posesEqual(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2, double threshold)
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

double RvizVisualTools::dRand(double min, double max)
{
  return std::uniform_real_distribution<double>(min, max)(mt_random_engine_);
}

float RvizVisualTools::fRand(float min, float max)
{
  return std::uniform_real_distribution<float>(min, max)(mt_random_engine_);
}

int RvizVisualTools::iRand(int min, int max)
{
  return std::uniform_int_distribution<int>(min, max)(mt_random_engine_);
}

void RvizVisualTools::setRandomSeed(unsigned int seed)
{
  mt_random_engine_.seed(seed);
}

void RvizVisualTools::printTranslation(const Eigen::Vector3d& translation)
{
  std::cout << "T.xyz = [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]" << std::endl;
}

void RvizVisualTools::printTransform(const Eigen::Isometry3d& transform)
{
  Eigen::Quaterniond q(transform.rotation());
  std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
            << transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", "
            << q.w() << "]" << std::endl;
}

void RvizVisualTools::printTransformRPY(const Eigen::Isometry3d& transform)
{
  // R-P-Y / X-Y-Z / 0-1-2 Euler Angle Standard
  Eigen::Vector3d vec = transform.rotation().eulerAngles(0, 1, 2);
  std::cout << "transform: [" << transform.translation().x() << ", " << transform.translation().y() << ", "
            << transform.translation().z() << ", " << vec[0] << ", " << vec[1] << ", " << vec[2] << "]\n";
}

void RvizVisualTools::printTransformFull(const Eigen::Isometry3d& transform)
{
  std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
            << transform.translation().z() << "], R = \n"
            << transform.rotation() << "\n";
}

void RvizVisualTools::prompt(const std::string& msg)
{
  getRemoteControl()->waitForNextStep(msg);
}

RemoteControlPtr& RvizVisualTools::getRemoteControl()
{
  if (!remote_control_)
  {
    loadRemoteControl();
  }
  return remote_control_;
}

void RvizVisualTools::loadRemoteControl()
{
  // Load remote
  if (!remote_control_)
  {
    remote_control_ = std::make_shared<RemoteControl>(nh_);
    ros::spinOnce();
  }
}

}  // namespace rviz_visual_tools
