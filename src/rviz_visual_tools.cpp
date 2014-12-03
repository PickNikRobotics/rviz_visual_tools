/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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

// Author: Dave Coleman

#include <rviz_visual_tools/rviz_visual_tools.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace rviz_visual_tools
{

RvizVisualTools::RvizVisualTools(const std::string& base_frame,
                         const std::string& marker_topic)
  :  nh_("~"),
     marker_topic_(marker_topic),
     base_frame_(base_frame)
{
  initialize();
}

void RvizVisualTools::initialize()
{
  floor_to_base_height_ = 0;
  marker_lifetime_ = ros::Duration(0.0); // 0 - unlimited
  muted_ = false;
  alpha_ = 0.8;
  global_scale_ = 1.0;
  // Cache the reusable markers
  loadRvizMarkers();
}

void RvizVisualTools::deleteAllMarkers() // TODO ROS-J change to bool
{
  // Helper for publishing rviz markers
  publishMarker( reset_marker_ );
}

void RvizVisualTools::resetMarkerCounts()
{
  arrow_marker_.id++;
  sphere_marker_.id++;
  block_marker_.id++;
  cylinder_marker_.id++;
  text_marker_.id++;
  rectangle_marker_.id++;
  line_marker_.id++;
  path_marker_.id++;
  spheres_marker_.id++;
}

bool RvizVisualTools::loadRvizMarkers()
{
  // Load reset marker -------------------------------------------------
  reset_marker_.header.frame_id = base_frame_;
  reset_marker_.header.stamp = ros::Time();
  reset_marker_.action = 3; // In ROS-J: visualization_msgs::Marker::DELETEALL;

  // Load arrow ----------------------------------------------------

  arrow_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  arrow_marker_.ns = "Arrow";
  // Set the marker type.
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  // Set the marker action.  Options are ADD and DELETE
  arrow_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  arrow_marker_.lifetime = marker_lifetime_;

  // Load rectangle ----------------------------------------------------

  rectangle_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  rectangle_marker_.ns = "Rectangle";
  // Set the marker type.
  rectangle_marker_.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  rectangle_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  rectangle_marker_.lifetime = marker_lifetime_;

  // Load line ----------------------------------------------------

  line_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  line_marker_.ns = "Line";
  // Set the marker type.
  line_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  // Set the marker action.  Options are ADD and DELETE
  line_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_marker_.lifetime = marker_lifetime_;

  // Load path ----------------------------------------------------

  path_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  path_marker_.ns = "Path";
  // Set the marker type.
  path_marker_.type = visualization_msgs::Marker::LINE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  path_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  path_marker_.lifetime = marker_lifetime_;
  // Constants
  path_marker_.pose.position.x = 0.0;
  path_marker_.pose.position.y = 0.0;
  path_marker_.pose.position.z = 0.0;

  path_marker_.pose.orientation.x = 0.0;
  path_marker_.pose.orientation.y = 0.0;
  path_marker_.pose.orientation.z = 0.0;
  path_marker_.pose.orientation.w = 1.0;

  // Load sphers ----------------------------------------------------

  spheres_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
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
  // Set the namespace and id for this marker.  This serves to create a unique ID
  block_marker_.ns = "Block";
  // Set the marker action.  Options are ADD and DELETE
  block_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  block_marker_.type = visualization_msgs::Marker::CUBE;
  // Lifetime
  block_marker_.lifetime = marker_lifetime_;

  // Load Cylinder ----------------------------------------------------
  cylinder_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  cylinder_marker_.ns = "Cylinder";
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  // Lifetime
  cylinder_marker_.lifetime = marker_lifetime_;

  // Load Sphere -------------------------------------------------
  sphere_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
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
  sphere_marker_.points.push_back( point_a );
  sphere_marker_.colors.push_back( getColor( BLUE ) );
  // Lifetime
  sphere_marker_.lifetime = marker_lifetime_;

  // Load Text ----------------------------------------------------
  // Set the namespace and id for this marker.  This serves to create a unique ID
  text_marker_.ns = "Text";
  // Set the marker action.  Options are ADD and DELETE
  text_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // Lifetime
  text_marker_.lifetime = marker_lifetime_;

  return true;
}

void RvizVisualTools::loadMarkerPub()
{
  if (pub_rviz_marker_)
    return;

  // Rviz marker publisher
  pub_rviz_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing Rviz markers on topic " << pub_rviz_marker_.getTopic());

  waitForSubscriber(pub_rviz_marker_); 
}

bool RvizVisualTools::waitForSubscriber(const ros::Publisher &pub, const double &wait_time)
{
  // Benchmark runtime
  ros::Time start_time;
  start_time = ros::Time::now();

  // Will wait at most 1000 ms (1 sec)
  ros::Time maxTime(ros::Time::now() + ros::Duration(wait_time));

  // This is wrong. It returns only the number of subscribers that have already established their direct connections to this publisher
  int num_existing_subscribers = pub.getNumSubscribers();

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  // Wait for subsriber
  while(num_existing_subscribers == 0)
  {
    // Check if timed out
    if (ros::Time::now() > maxTime)
    {
      ROS_WARN_STREAM_NAMED("visual_tools", "Topic '" << pub.getTopic() << "' unable to connect to any subscribers within " 
                            << wait_time << " seconds. It is possible initially published visual messages will be lost.");
      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    // Check again
    num_existing_subscribers = pub.getNumSubscribers();
    //std::cout << "num_existing_subscribers " << num_existing_subscribers << std::endl;
  }

  // Benchmark runtime
  if (false)
  {
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_DEBUG_STREAM_NAMED("visual_tools", "Topic '" << pub.getTopic() << "' took " << duration 
                           << " seconds to connect to a subscriber. Connected to " << num_existing_subscribers 
                           << " total subsribers");
  }
  return true;
}

void RvizVisualTools::setFloorToBaseHeight(double floor_to_base_height)
{
  floor_to_base_height_ = floor_to_base_height;
}

void RvizVisualTools::setLifetime(double lifetime)
{
  marker_lifetime_ = ros::Duration(lifetime);

  // Update cached markers
  arrow_marker_.lifetime = marker_lifetime_;
  rectangle_marker_.lifetime = marker_lifetime_;
  line_marker_.lifetime = marker_lifetime_;
  sphere_marker_.lifetime = marker_lifetime_;
  block_marker_.lifetime = marker_lifetime_;
  cylinder_marker_.lifetime = marker_lifetime_;
  text_marker_.lifetime = marker_lifetime_;
}

const rviz_visual_tools::colors RvizVisualTools::getRandColor()
{
  std::vector<rviz_visual_tools::colors> all_colors;
  
  all_colors.push_back(RED);
  all_colors.push_back(GREEN);
  all_colors.push_back(BLUE);
  all_colors.push_back(GREY);
  all_colors.push_back(WHITE);
  all_colors.push_back(ORANGE);
  //all_colors.push_back(BLACK);
  all_colors.push_back(YELLOW);
  all_colors.push_back(PURPLE);
  
  int rand_num = iRand(0, all_colors.size() - 1);
  return all_colors[ rand_num ];
}

std_msgs::ColorRGBA RvizVisualTools::getColor(const rviz_visual_tools::colors &color)
{
  std_msgs::ColorRGBA result;
  result.a = alpha_;
  switch(color)
  {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      break;
    case GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      break;
    case WHITE:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      break;
    case ORANGE:
      result.r = 1.0;
      result.g = 0.5;
      result.b = 0.0;
      break;
    case TRANSLUCENT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
      result.a = 0.3;
      break;
    case TRANSLUCENT2:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.1;
      break;
    case BLACK:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      break;
    case YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      break;
    case PURPLE:
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      break;
    case RAND:
      // Make sure color is not *too* light
      do
      {
        result.r = fRand(0.0,1.0);
        result.g = fRand(0.0,1.0);
        result.b = fRand(0.0,1.0);
      } while (result.r + result.g + result.b < 1.5); // 3 would be white
      break;
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
  }

  return result;
}

geometry_msgs::Vector3 RvizVisualTools::getScale(const rviz_visual_tools::scales &scale, bool arrow_scale, double marker_scale)
{
  geometry_msgs::Vector3 result;
  double val(0.0);
  switch(scale)
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
      ROS_ERROR_STREAM_NAMED("visualization_tools","Not implemented yet");
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
  // from http://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/

  // Goal pose:
  Eigen::Quaterniond q;

  Eigen::Vector3d axis_vector = b - a;
  axis_vector.normalize();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
  right_axis_vector.normalized();
  double theta = axis_vector.dot(up_vector);
  double angle_rotation = -1.0*acos(theta);

  //-------------------------------------------
  // Method 1 - TF - works
  //Convert to TF
  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);

  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);
  //-------------------------------------------
  //std::cout << q.toRotationMatrix() << std::endl;

  //-------------------------------------------
  // Method 2 - Eigen - broken TODO
  //q = Eigen::AngleAxis<double>(angle_rotation, right_axis_vector);
  //-------------------------------------------
  //std::cout << q.toRotationMatrix() << std::endl;

  // Rotate so that vector points along line
  Eigen::Affine3d pose;
  q.normalize();
  pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
  pose.translation() = a;

  return pose;
}

bool RvizVisualTools::publishSphere(const Eigen::Affine3d &pose, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  return publishSphere(convertPose(pose), color, scale, ns);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d &point, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d &point, const rviz_visual_tools::colors color, const double scale, const std::string& ns)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Point &point, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position = point;
  return publishSphere(pose_msg, color, scale, ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  return publishSphere(pose, color, getScale(scale, false, 0.1), ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, double scale, const std::string& ns)
{
  geometry_msgs::Vector3 scale_msg;
  scale_msg.x = scale;
  scale_msg.y = scale;
  scale_msg.z = scale;
  return publishSphere(pose, color, scale_msg, ns);
}
bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, const geometry_msgs::Vector3 scale, const std::string& ns)
{
  if(muted_)
    return true; // this function will only work if we have loaded the publishers

  // Set the frame ID and timestamp
  sphere_marker_.header.stamp = ros::Time::now();

  sphere_marker_.id++;
  sphere_marker_.color = getColor(color);
  sphere_marker_.scale = scale;
  sphere_marker_.ns = ns;

  // Update the single point with new pose
  sphere_marker_.points[0] = pose.position;
  sphere_marker_.colors[0] = getColor(color);

  // Helper for publishing rviz markers
  return publishMarker( sphere_marker_ );
}

bool RvizVisualTools::publishArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale)
{
  return publishArrow(convertPose(pose), color, scale);
}

bool RvizVisualTools::publishArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale)
{
  if(muted_)
    return true;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  arrow_marker_.header.stamp = ros::Time::now();

  arrow_marker_.id++;
  arrow_marker_.pose = pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale, true);

  // Helper for publishing rviz markers
  return publishMarker( arrow_marker_ );
}

bool RvizVisualTools::publishBlock(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, const double &block_size)
{
  if(muted_)
    return true;

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
  block_marker_.color = getColor( color );

  // Helper for publishing rviz markers
  return publishMarker( block_marker_ );
}

bool RvizVisualTools::publishCylinder(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, double height, double radius)
{
  if(muted_)
    return true;

  // Set the timestamp
  cylinder_marker_.header.stamp = ros::Time::now();

  cylinder_marker_.id++;

  // Set the pose
  cylinder_marker_.pose = pose;

  // Set marker size
  cylinder_marker_.scale.x = radius;
  cylinder_marker_.scale.y = radius;
  cylinder_marker_.scale.z = height;

  // Set marker color
  cylinder_marker_.color = getColor( color );

  // Helper for publishing rviz markers
  return publishMarker( cylinder_marker_ );
}

bool RvizVisualTools::publishGraph(const graph_msgs::GeometryGraph &graph, const rviz_visual_tools::colors color, double radius)
{
  if(muted_)
    return true;

  // Track which pairs of nodes we've already connected since graph is bi-directional
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator,bool> return_value;

  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert( node_ids(i,j) );
      if (return_value.second == false)
      {
        // Element already existed in set, so don't add a new collision object
      }
      else
      {
        // Create a cylinder from two points
        a = convertPoint(graph.nodes[i]);
        b = convertPoint(graph.nodes[graph.edges[i].node_ids[j]]);

        // add other direction of edge
        added_edges.insert( node_ids(j,i) );

        // Distance between two points
        double height = (a - b).lpNorm<2>();

        // Find center point
        Eigen::Vector3d pt_center = getCenterPoint(a, b);

        // Create vector
        Eigen::Affine3d pose;
        pose = getVectorBetweenPoints(pt_center, b);

        // Convert pose to be normal to cylindar axis
        Eigen::Affine3d rotation;
        rotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY());
        pose = pose * rotation;

        // Publish individually
        publishCylinder(convertPose(pose), color, height, radius);
      }
    }
  }

  return true;
}

bool RvizVisualTools::publishRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, 
                                       const rviz_visual_tools::colors color)
{
  if(muted_)
    return true;

  // Set the timestamp
  rectangle_marker_.header.stamp = ros::Time::now();

  rectangle_marker_.id++;
  rectangle_marker_.color = getColor(color);

  // Calculate center pose
  geometry_msgs::Pose pose;
  pose.position.x = (point1.x - point2.x) / 2.0 + point2.x;
  pose.position.y = (point1.y - point2.y) / 2.0 + point2.y;
  pose.position.z = (point1.z - point2.z) / 2.0 + point2.z;
  rectangle_marker_.pose = pose;

  // Calculate scale
  rectangle_marker_.scale.x = fabs(point1.x - point2.x);
  rectangle_marker_.scale.y = fabs(point1.y - point2.y);
  rectangle_marker_.scale.z = fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (!rectangle_marker_.scale.x) rectangle_marker_.scale.x = SMALL_SCALE;
  if (!rectangle_marker_.scale.y) rectangle_marker_.scale.y = SMALL_SCALE;
  if (!rectangle_marker_.scale.z) rectangle_marker_.scale.z = SMALL_SCALE;

  // Helper for publishing rviz markers
  return publishMarker( rectangle_marker_ );
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                              const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale)
{
  if(muted_)
    return true;

  // Set the timestamp
  line_marker_.header.stamp = ros::Time::now();

  line_marker_.id++;
  line_marker_.color = getColor(color);
  line_marker_.scale = getScale( scale, false, 0.1 );

  line_marker_.points.clear();
  line_marker_.points.push_back(point1);
  line_marker_.points.push_back(point2);

  // Helper for publishing rviz markers
  return publishMarker( line_marker_ );
}

bool RvizVisualTools::publishPath(const std::vector<geometry_msgs::Point> &path, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  if(muted_)
    return true;

  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED("publishPath","Skipping path because " << path.size() << " points passed in.");
    return true;
  }

  path_marker_.header.stamp = ros::Time();
  path_marker_.ns = ns;

  // Provide a new id every call to this function
  path_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor( color );
  path_marker_.scale = getScale(scale, false, 0.25);
  path_marker_.color = this_color;
  path_marker_.points.clear();
  path_marker_.colors.clear();

  // Convert path coordinates
  for( std::size_t i = 1; i < path.size(); ++i )
  {
    // Add the point pair to the line message
    path_marker_.points.push_back( path[i-1] );
    path_marker_.points.push_back( path[i] );
    path_marker_.colors.push_back( this_color );
    path_marker_.colors.push_back( this_color );
  }

  // Helper for publishing rviz markers
  return publishMarker( path_marker_ );
}

bool RvizVisualTools::publishPolygon(const geometry_msgs::Polygon &polygon, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point temp;
  geometry_msgs::Point first; // remember first point because we will connect first and last points for last line
  for (std::size_t i = 0; i < polygon.points.size(); ++i)
  {
    temp.x = polygon.points[i].x;
    temp.y = polygon.points[i].y;
    temp.z = polygon.points[i].z;
    if (i == 0)
      first = temp;
    points.push_back(temp);
  }
  points.push_back(first); // connect first and last points for last line

  publishPath(points, color, scale, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<Eigen::Vector3d> &points, const rviz_visual_tools::colors color, const double scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points_msg;
  geometry_msgs::Point temp;

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    tf::pointEigenToMsg(points[i], temp);
    points_msg.push_back(temp);
  }


  return publishSpheres(points_msg, color, scale, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors color, const double scale, const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  publishSpheres( points, color, scale_vector, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors color, const rviz_visual_tools::scales scale, const std::string& ns)
{
  publishSpheres( points, color, getScale(scale, false, 0.25), ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors color, const geometry_msgs::Vector3 &scale, const std::string& ns)
{
  if(muted_)
    return true;

  spheres_marker_.header.stamp = ros::Time();
  spheres_marker_.ns = ns;

  // Provide a new id every call to this function
  spheres_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor( color );
  spheres_marker_.scale = scale;
  spheres_marker_.color = this_color;
  //spheres_marker_.points.clear();
  spheres_marker_.colors.clear();

  spheres_marker_.points = points; // straight copy

  // Convert path coordinates
  for( std::size_t i = 0; i < points.size(); ++i )
  {
    spheres_marker_.colors.push_back( this_color );
  }

  // Helper for publishing rviz markers
  return publishMarker( spheres_marker_ );
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales scale, bool static_id)
{
  publishText(pose, text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const rviz_visual_tools::colors &color, const geometry_msgs::Vector3 scale, bool static_id)
{
  if(muted_)
    return true;

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
  text_marker_.color = getColor( color );
  text_marker_.scale = scale;

  // Helper for publishing rviz markers
  publishMarker( text_marker_ );

  // Restore the ID count if needed
  if (static_id)
    text_marker_.id = temp_id;

  return true;
}

bool RvizVisualTools::publishMarker(const visualization_msgs::Marker &marker)
{
  if(muted_)
    return true;

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( marker );
  ros::spinOnce();

  return true;
}

bool RvizVisualTools::publishTest()
{
  // Create pose
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;

  // Test all shapes ----------

  ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
  generateRandomPose(pose1);
  publishArrow(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Sphere");
  generateRandomPose(pose1);
  publishSphere(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Rectangle");
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishRectangle(pose1.position, pose2.position, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Line");
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishLine(pose1.position, pose2.position, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Block");
  generateRandomPose(pose1);
  publishBlock(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Cylinder");
  generateRandomPose(pose1);
  publishCylinder(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Text");
  generateRandomPose(pose1);
  publishText(pose1, "Test", rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  return true;
}

geometry_msgs::Pose RvizVisualTools::convertPose(const Eigen::Affine3d &pose)
{
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose, pose_msg);
  return pose_msg;
}

Eigen::Affine3d RvizVisualTools::convertPose(const geometry_msgs::Pose &pose)
{
  Eigen::Affine3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  return pose_eigen;
}

Eigen::Affine3d RvizVisualTools::convertPoint32ToPose(const geometry_msgs::Point32 &point)
{
  Eigen::Affine3d pose_eigen = Eigen::Affine3d::Identity();
  pose_eigen.translation().x() = point.x;
  pose_eigen.translation().y() = point.y;
  pose_eigen.translation().z() = point.z;
  return pose_eigen;
}

geometry_msgs::Pose RvizVisualTools::convertPointToPose(const geometry_msgs::Point &point)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position = point;
  return pose_msg;
}

geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose)
{
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose, pose_msg);
  return pose_msg.position;
}

Eigen::Vector3d RvizVisualTools::convertPoint(const geometry_msgs::Point &point)
{
  Eigen::Vector3d point_eigen;
  point_eigen[0] = point.x;
  point_eigen[1] = point.y;
  point_eigen[2] = point.z;
  return point_eigen;
}

Eigen::Vector3d RvizVisualTools::convertPoint32(const geometry_msgs::Point32 &point)
{
  Eigen::Vector3d point_eigen;
  point_eigen[0] = point.x;
  point_eigen[1] = point.y;
  point_eigen[2] = point.z;
  return point_eigen;
}

geometry_msgs::Point32 RvizVisualTools::convertPoint32(const Eigen::Vector3d &point)
{
  geometry_msgs::Point32 point_msg;
  point_msg.x = point[0];
  point_msg.y = point[1];
  point_msg.z = point[2];
  return point_msg;
}

void RvizVisualTools::generateRandomPose(geometry_msgs::Pose& pose)
{
  // Position
  pose.position.x = dRand(0, 1);
  pose.position.y = dRand(0, 1);
  pose.position.z = dRand(0, 1);

  // Orientation on place
  double angle = M_PI * dRand(0.1,1.0);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

void RvizVisualTools::generateEmptyPose(geometry_msgs::Pose& pose)
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

double RvizVisualTools::dRand(double dMin, double dMax)
{
  double d = (double)rand() / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

float RvizVisualTools::fRand(float dMin, float dMax)
{
  float d = (float)rand() / RAND_MAX;
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
  }
  while (x >= RAND_MAX - remainder);
  return min + x % n;
}

void RvizVisualTools::print()
{
  ROS_WARN_STREAM_NAMED("visual_tools","Debug Visual Tools variable values:");
  std::cout << "marker_topic_: " << marker_topic_ << std::endl;
  std::cout << "base_frame_: " << base_frame_ << std::endl;
  std::cout << "floor_to_base_height_: " << floor_to_base_height_ << std::endl;
  std::cout << "marker_lifetime_: " << marker_lifetime_.toSec() << std::endl;
  std::cout << "muted_: " << muted_ << std::endl;
  std::cout << "alpha_: " << alpha_ << std::endl;
}

} // namespace


