/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

/* \author  Dave Coleman
 * \desc    Helper functions for displaying and debugging data in Rviz via published markers
 *
 *          See README.md for developers notes.
 *
 */

#ifndef RVIZ_VISUAL_TOOLS__VISUAL_TOOLS_H_
#define RVIZ_VISUAL_TOOLS__VISUAL_TOOLS_H_

#include <ros/ros.h>

// Eigen
#include <Eigen/Geometry>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Boost
#include <boost/shared_ptr.hpp>

// Messages
#include <std_msgs/ColorRGBA.h>
#include <graph_msgs/GeometryGraph.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace rviz_visual_tools
{

// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
static const double SMALL_SCALE = 0.001;
static const double LARGE_SCALE = 100;

// Note: when adding new colors to colors, also add them to getRandColor() function
enum colors { RED,
              GREEN,
              BLUE,
              GREY,
              WHITE,
              ORANGE,
              BLACK,
              YELLOW,
              PURPLE,
              TRANSLUCENT,
              TRANSLUCENT2,
              RAND };

enum scales { XXSMALL,
              XSMALL,
              SMALL,
              REGULAR,
              LARGE, xLARGE, xxLARGE, xxxLARGE,
              XLARGE,
              XXLARGE };

class RvizVisualTools
{
protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher pub_rviz_marker_; // for rviz visualization markers

  // Strings
  std::string marker_topic_; // topic to publish to rviz
  std::string base_frame_; // name of base link

  // TODO rename this
  double floor_to_base_height_; // allows an offset between base link and floor where objects are built

  // Duration to have Rviz markers persist, 0 for infinity
  ros::Duration marker_lifetime_;

  // End Effector Markers
  visualization_msgs::MarkerArray ee_marker_array_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
  std::vector<geometry_msgs::Pose> marker_poses_;

  // Library settings
  bool muted_; // Whether to actually publish to rviz or not
  double alpha_; // opacity of all markers
  double global_scale_; // allow all markers to be increased by a constanct factor

  // Cached Rviz markers
  visualization_msgs::Marker arrow_marker_;
  visualization_msgs::Marker sphere_marker_;
  visualization_msgs::Marker block_marker_;
  visualization_msgs::Marker cylinder_marker_;
  visualization_msgs::Marker text_marker_;
  visualization_msgs::Marker rectangle_marker_;
  visualization_msgs::Marker line_marker_;
  visualization_msgs::Marker path_marker_;
  visualization_msgs::Marker spheres_marker_;
  visualization_msgs::Marker reset_marker_;

private:
  /**
   * \brief Shared function for initilization by constructors
   */
  void initialize();

public:

  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   */
  RvizVisualTools(const std::string& base_frame,
              const std::string& marker_topic = RVIZ_MARKER_TOPIC);
  /**
   * \brief Deconstructor
   */
  ~RvizVisualTools() {};

  /**
   * \brief Tell Rviz to clear all markers on a particular display. Note: only works on ROS Indigo and newer
   */
  void deleteAllMarkers();

  /**
   * \brief Reset the id's of all published markers so that they overwrite themselves in the future
   *        NOTE you may prefer deleteAllMarkers()
   */
  void resetMarkerCounts();

  /**
   * \brief Pre-load rviz markers for better efficiency
   * \return converted pose   * \return true on sucess
   */
  bool loadRvizMarkers();

  /**
   * \brief Load publishers as needed
   */
  void loadMarkerPub();

  /**
   * \brief Wait until at least one subscriber connects to a publisher
   * \param pub - the publisher to check for subsribers
   * \param wait_time - time to wait for subscriber to be availnle before throwing warning
   * \return true on successful connection
   */
  bool waitForSubscriber(const ros::Publisher &pub, const double &wait_time = 1.0);

  /**
   * \brief Return if we are in verbose mode
   */
  bool isMuted()
  {
    return muted_;
  }

  /**
   * \brief Set this class to not actually publish anything to Rviz.
   * \param muted true if verbose
   */
  void setMuted(bool muted)
  {
    muted_ = muted;
  }

  /**
   * \brief Allows an offset between base link and floor where objects are built. Default is zero
   * \param floor_to_base_height - the offset
   */
  void setFloorToBaseHeight(double floor_to_base_height);

  /**
   * \brief Change the transparency of all markers published
   * \param alpha - value 0 - 1 where 0 is invisible
   */
  void setAlpha(double alpha)
  {
    alpha_ = alpha;
  }

  /**
   * \brief Set the lifetime of markers published to rviz
   * \param lifetime seconds of how long to show markers. 0 for inifinity
   */
  void setLifetime(double lifetime);

  /**
   * \brief Get a random color from the list of hardcoded enum color types
   * \return Random color from rviz_visual_tools::colors
   */
  const rviz_visual_tools::colors getRandColor();

  /**
   * \brief Get the RGB value of standard colors
   * \param color - an enum pre-defined name of a color
   * \return the RGB message equivalent
   */
  std_msgs::ColorRGBA getColor(const rviz_visual_tools::colors &color);

  /**
   * \brief Get the rviz marker scale of standard sizes
   * \param scale - an enum pre-defined name of a size
   * \param arrow_scale - they do not have an even scaling, compensate
   * \param marker_scale - amount to scale the scale for accounting for different types of markers
   * \return vector of 3 scales
   */
  geometry_msgs::Vector3 getScale(const rviz_visual_tools::scales &scale, bool arrow_scale = false, double marker_scale = 1.0);

  /**
   * \brief Create a vector that points from point a to point b
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \return vector from a to b
   */
  Eigen::Affine3d getVectorBetweenPoints(Eigen::Vector3d a, Eigen::Vector3d b);

  /**
   * \brief Find the center between to points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \return center point
   */
  Eigen::Vector3d getCenterPoint(Eigen::Vector3d a, Eigen::Vector3d b);

  /**
   * \brief Get the base frame
   * \return name of base frame
   */
  const std::string getBaseFrame()
  {
    return base_frame_;
  }

  /**
   * \brief Change the global base frame
   *        Note: this might reset all your current markers
   * \param name of frame
   */
  void setBaseFrame(const std::string& base_frame)
  {
    base_frame_ = base_frame;
    loadRvizMarkers();
  }

  /**
   * \brief Getter for the global scale used for changing size of all markers
   */
  double getGlobalScale()
  {
    return global_scale_;
  }

  /**
   * \brief Setter for the global scale used for changing size of all markers
   */
  void setGlobalScale(double global_scale)
  {
    global_scale_ = global_scale;
  }

  /**
   * \brief Publish a marker of a sphere to rviz
   * \param pose - the location to publish the sphere with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishSphere(const Eigen::Affine3d &pose, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const Eigen::Vector3d &point, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const Eigen::Vector3d &point, const rviz_visual_tools::colors color, const double scale, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Point &point, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, const double scale, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color, const geometry_msgs::Vector3 scale, const std::string& ns = "Sphere");

  /**
   * \brief Publish a marker of a series of spheres to rviz
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const std::vector<Eigen::Vector3d> &points, const rviz_visual_tools::colors color = BLUE, const double scale = 0.1, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors color = BLUE, const double scale = 0.1, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors color, const geometry_msgs::Vector3 &scale, const std::string& ns = "Spheres");



  /**
   * \brief Publish a marker of an arrow to rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR);
  bool publishArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR);

  /**
   * \brief Publish a marker of rectangle to rviz
   * \param point1 - x,y,z top corner location of box
   * \param point2 - x,y,z bottom opposite corner location of box
   * \param color - an enum pre-defined name of a color
   * \return true on success
   */
  bool publishRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, const rviz_visual_tools::colors color = BLUE);

  /**
   * \brief Publish a marker of line to rviz
   * \param point1 - x,y,z of start of line
   * \param point2 - x,y,z of end of line
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                   const rviz_visual_tools::colors color = BLUE, const rviz_visual_tools::scales scale = REGULAR);

  /**
   * \brief Publish a marker of a series of connected lines to rviz
   * \param path - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPath(const std::vector<geometry_msgs::Point> &path, const rviz_visual_tools::colors color = RED, const rviz_visual_tools::scales scale = REGULAR,
                   const std::string& ns = "Path");

  /**
   * \brief Publish a marker of a polygon to Rviz
   * \param polygon - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPolygon(const geometry_msgs::Polygon &polygon, const rviz_visual_tools::colors color = RED, const rviz_visual_tools::scales scale = REGULAR,
                      const std::string& ns = "Polygon");

  /**
   * \brief Publish a marker of a block to Rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param size - height=width=depth=size
   * \return true on success
   */
  bool publishBlock(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color = BLUE, const double &block_size = 0.1);

  /**
   * \brief Publish a marker of a cylinder to Rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param height - geometry of cylinder
   * \param radius - geometry of cylinder
   * \return true on success
   */
  bool publishCylinder(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors color = BLUE, double height = 0.1, double radius = 0.1);

  /**
   * \brief Publish a graph
   * \param graph of nodes and edges
   * \param color - an enum pre-defined name of a color
   * \param radius - width of cylinders
   * \return true on success
   */
  bool publishGraph(const graph_msgs::GeometryGraph &graph, const rviz_visual_tools::colors color, double radius);

  /**
   * \brief Publish a marker of a text to Rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param text - what to display
   * \param color - an enum pre-defined name of a colo
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishText(const geometry_msgs::Pose &pose, const std::string &text,
                   const rviz_visual_tools::colors &color = WHITE, const rviz_visual_tools::scales scale = REGULAR, bool static_id = true);

  bool publishText(const geometry_msgs::Pose &pose, const std::string &text,
                   const rviz_visual_tools::colors &color, const geometry_msgs::Vector3 scale, bool static_id = true);

  /**
   * \brief Publish a visualization_msgs Marker of a custom type. Allows reuse of the ros publisher
   * \param marker - a pre-made marker ready to be published
   * \return true on success
   */
  bool publishMarker(const visualization_msgs::Marker &marker);

  /**
   * \brief Run a simple test of all visual_tool's features
   * \return true on success
   */
  bool publishTest();

  /**
   * \brief Convert an Eigen pose to a geometry_msg pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted pose
   */
  static geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose);

  /**
   * \brief Convert a geometry_msg pose to an Eigen pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted pose
   */
  static Eigen::Affine3d convertPose(const geometry_msgs::Pose &pose);

  /**
   * \brief Convert a geometry_msg point (32bit) to an Eigen pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted point with default rotation matrix
   */
  static Eigen::Affine3d convertPoint32ToPose(const geometry_msgs::Point32 &point);

  /**
   * \brief
   * \param input - description
   * \param input - description
   * \return
   */
  static geometry_msgs::Pose convertPointToPose(const geometry_msgs::Point &point);

  /**
   * \brief Convert an Eigen pose to a geometry_msg point
   *        Note: NOT memory efficient
   * \param pose
   * \return converted point with orientation discarded
   */
  static geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  static Eigen::Vector3d convertPoint(const geometry_msgs::Point &point);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  static Eigen::Vector3d convertPoint32(const geometry_msgs::Point32 &point);

  /**
   * \brief Convert an Eigen point to a 32 bit geometry_msg point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  static geometry_msgs::Point32 convertPoint32(const Eigen::Vector3d &point);

  /**
   * \brief Create a random pose
   * \param Pose to fill in
   */
  void generateRandomPose(geometry_msgs::Pose& pose);

  /**
   * \brief Create a pose of position (0,0,0) and quaternion (0,0,0,1)
   * \param Pose to fill in
   */
  void generateEmptyPose(geometry_msgs::Pose& pose);

  /**
   * \brief Get random between min and max
   */
  static double dRand(double min, double max);
  static float fRand(float min, float max);
  static int iRand(int min, int max);

  /**
   * \brief Debug variables to console
   */
  void print();

}; // class

typedef boost::shared_ptr<RvizVisualTools> RvizVisualToolsPtr;
typedef boost::shared_ptr<const RvizVisualTools> RvizVisualToolsConstPtr;

} // namespace

#endif
