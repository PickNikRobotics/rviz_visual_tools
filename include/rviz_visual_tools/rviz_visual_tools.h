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

#ifndef RVIZ_VISUAL_TOOLS__VISUAL_TOOLS_H_
#define RVIZ_VISUAL_TOOLS__VISUAL_TOOLS_H_

#include <ros/ros.h>
#include <cmath> // for random poses

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <trajectory_msgs/JointTrajectory.h>

// Deprecation
#include <rviz_visual_tools/deprecation.h>

namespace rviz_visual_tools
{

// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
static const double SMALL_SCALE = 0.001;
static const double LARGE_SCALE = 100;

// Note: when adding new colors to colors, also add them to getRandColor() function
enum colors { BLACK,
              BLUE,
              BROWN,
              CYAN,
              DARK_GREY,
              GREEN,
              GREY,
              LIME_GREEN,
              MAGENTA,
              ORANGE,
              PINK,
              PURPLE,
              RED,
              WHITE,
              YELLOW,
              TRANSLUCENT_LIGHT,
              TRANSLUCENT,
              TRANSLUCENT_DARK,
              RAND,
	      CLEAR,
              DEFAULT // i.e. 'do not change default color'
};

enum scales { XXSMALL,
              XSMALL,
              SMALL,
              REGULAR,
              LARGE, xLARGE, xxLARGE, xxxLARGE,
              XLARGE,
              XXLARGE };

/**
 * \brief Bounds for generateRandomPose()
 */
struct RandomPoseBounds
{
  double x_min_,         x_max_;
  double y_min_,         y_max_;
  double z_min_,         z_max_;
  double elevation_min_, elevation_max_;
  double azimuth_min_,   azimuth_max_;
  double angle_min_,     angle_max_;

  RandomPoseBounds(double x_min = 0.0, double x_max = 1.0, double y_min = 0.0, double y_max = 1.0, double z_min = 0.0, double z_max = 1.0,
                   double elevation_min = 0.0, double elevation_max = M_PI, double azimuth_min = 0.0, double azimuth_max = 2 * M_PI,
                   double angle_min = 0.0, double angle_max = 2 * M_PI)
  {
    x_min_ = x_min;                 x_max_ = x_max;
    y_min_ = y_min;                 y_max_ = y_max;
    z_min_ = z_min;                 z_max_ = z_max;
    elevation_min_ = elevation_min; elevation_max_ = elevation_max;
    azimuth_min_ = azimuth_min;     azimuth_max_ = azimuth_max;
    angle_min_ = angle_min;         angle_max_ = angle_max;
  }
};

/**
 * \brief Bounds for generateRandomCuboid()
 */
struct RandomCuboidBounds
{
  double cuboid_size_min_, cuboid_size_max_;

  RandomCuboidBounds(double cuboid_size_min = 0.02, double cuboid_size_max = 0.15)
  {
    cuboid_size_min_ = cuboid_size_min;
    cuboid_size_max_ = cuboid_size_max;
  }
};


class RvizVisualTools
{

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
  bool deleteAllMarkers();

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
  colors getRandColor();

  /**
   * \brief Get the RGB value of standard colors
   * \param color - an enum pre-defined name of a color
   * \return the RGB message equivalent
   */
  std_msgs::ColorRGBA getColor(const colors &color);

  /**
   * \brief Get the rviz marker scale of standard sizes
   * \param scale - an enum pre-defined name of a size
   * \param arrow_scale - they do not have an even scaling, compensate
   * \param marker_scale - amount to scale the scale for accounting for different types of markers
   * \return vector of 3 scales
   */
  geometry_msgs::Vector3 getScale(const scales &scale, bool arrow_scale = false, double marker_scale = 1.0);

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
   * \brief Display a visualization_msgs Marker of a custom type. Allows reuse of the ros publisher
   * \param marker - a pre-made marker ready to be published
   * \return true on success
   */
  bool publishMarker(const visualization_msgs::Marker &marker);

  /**
   * \brief Enable batch publishing - useful for when many markers need to be published at once and the ROS topic can get
   *        overloaded. This collects all published markers into array and only publishes them with triggerBatchPublish() is called
   */
  void enableBatchPublishing(bool enable);

  /**
   * \brief Trigger the publish function to send out all collected markers
   * \return true on success
   */
  bool triggerBatchPublish();

  /**
   * \brief Trigger the publish function to send out all collected markers. Also then turns off the batch mode. This is safer
   *        incase programmer forgets
   * \return true on success
   */
  bool triggerBatchPublishAndDisable();

  /**
   * \brief Display an array of markers, allows reuse of the ROS publisher
   * \param markers
   * \return true on success
   */
  bool publishMarkers(const visualization_msgs::MarkerArray &markers);

  /**
   * \brief Display a cone of a given angle along the x-axis
   * \param pose - the location and orientation of the cone
   * \param color - color of the cone
   * \param scale - size of the cone
   * \return true on success
   */
  bool publishCone(const Eigen::Affine3d &pose, double angle, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale  = 1.0);
  bool publishCone(const geometry_msgs::Pose &pose, double angle, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);

  /**
   * \brief Display the XY plane of a given pose
   * \param pose - the position of the plane
   * \param color - the color of the plane
   * \param scale - the size of the vizualized plane
   * \return true on success
   */
  bool publishXYPlane(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);
  bool publishXYPlane(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);

  /**
   * \brief Display the XY plane of a given pose
   * \param pose - the position of the plane
   * \param color - the color of the plane
   * \param scale - the size of the vizualized plane
   * \return true on success
   */
  bool publishXZPlane(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);
  bool publishXZPlane(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);

  /**
   * \brief Display the XY plane of a given pose
   * \param pose - the position of the plane
   * \param color - the color of the plane
   * \param scale - the size of the vizualized plane
   * \return true on success
   */
  bool publishYZPlane(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);
  bool publishYZPlane(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color = TRANSLUCENT, double scale = 1.0);

  /**
   * \brief Display a marker of a sphere
   * \param pose - the location to publish the sphere with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishSphere(const Eigen::Affine3d &pose, const colors &color = BLUE, const scales &scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const Eigen::Vector3d &point, const colors &color = BLUE, const scales &scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const Eigen::Vector3d &point, const colors &color, const double scale, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Point &point, const colors &color = BLUE, const scales &scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const colors &color = BLUE, const scales &scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const colors &color, const double scale, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const colors &color, const geometry_msgs::Vector3 scale, const std::string& ns = "Sphere");

  /**
   * \brief Display a marker of a series of spheres
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const std::vector<Eigen::Vector3d> &points, const colors &color = BLUE, const double scale = 0.1, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const colors &color = BLUE, const double scale = 0.1, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const colors &color = BLUE, const scales &scale = REGULAR, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const colors &color, const geometry_msgs::Vector3 &scale, const std::string& ns = "Spheres");

  /**
   * \brief Display an arrow along the x-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - the length of the arrow tail
   * \return true on success
   */
  bool publishXArrow(const Eigen::Affine3d &pose, const colors &color = RED,
                     const scales &scale = REGULAR, double length = 0.1);
  bool publishXArrow(const  geometry_msgs::Pose &pose, const colors &color = RED,
                     const scales &scale = REGULAR, double length = 0.1);
  bool publishXArrow(const  geometry_msgs::PoseStamped &pose, const colors &color = RED,
                     const scales &scale = REGULAR, double length = 0.1);

  /**
   * \brief Display an arrow along the y-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - the length of the arrow tail
   * \return true on success
   */
  bool publishYArrow(const Eigen::Affine3d &pose, const colors &color = GREEN,
                     const scales &scale = REGULAR, double length = 0.1);
  bool publishYArrow(const  geometry_msgs::Pose &pose, const colors &color = GREEN,
                     const scales &scale = REGULAR, double length = 0.1);
  bool publishYArrow(const  geometry_msgs::PoseStamped &pose, const colors &color = GREEN,
                     const scales &scale = REGULAR, double length = 0.1);

  /**
   * \brief Display an arrow along the z-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - the length of the arrow tail
   * \return true on success
   */
  bool publishZArrow(const Eigen::Affine3d &pose, const colors &color = BLUE,
                     const scales &scale = REGULAR, double length = 0.1);
  bool publishZArrow(const  geometry_msgs::Pose &pose, const colors &color = BLUE,
                     const scales &scale = REGULAR, double length = 0.1);
  bool publishZArrow(const  geometry_msgs::PoseStamped &pose, const colors &color = BLUE,
                     const scales &scale = REGULAR, double length = 0.1);

  /**
   * \brief Display an arrow along the x-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - how long the arrow tail should be
   * \return true on success
   */
  bool publishArrow(const Eigen::Affine3d &pose, const colors &color = BLUE,
                    const scales &scale = REGULAR, double length = 0.1);
  bool publishArrow(const geometry_msgs::Pose &pose, const colors &color = BLUE,
                    const scales &scale = REGULAR, double length = 0.1);
  bool publishArrow(const geometry_msgs::PoseStamped &pose, const colors &color = BLUE,
                    const scales &scale = REGULAR, double length = 0.1);

  /**
   * \brief Display a rectangular cuboid
   * \param point1 - x,y,z top corner location of box
   * \param point2 - x,y,z bottom opposite corner location of box
   * \param color - an enum pre-defined name of a color
   * \return true on success
   */
  bool publishCuboid(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                     const colors &color = BLUE);
  bool publishCuboid(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                     const colors &color = BLUE);
  /**
   * \brief Display a rectangular cuboid
   * \param pose - pose of the box
   * \param depth - depth of the box
   * \param width - width of the box
   * \param height - height of the box
   * \param color - an enum pre-defined name of a color
   * \return true on success
   */
  bool publishCuboid(const geometry_msgs::Pose &pose, const double depth, const double width, const double height,
                     const colors &color = BLUE);
  bool publishCuboid(const Eigen::Affine3d  &pose, const double depth, const double width, const double height,
                     const colors &color = BLUE);

  /**
   * \brief Display a marker of line
   * \param point1 - x,y,z of start of line
   * \param point2 - x,y,z of end of line
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishLine(const Eigen::Affine3d &point1, const Eigen::Affine3d &point2,
                   const colors &color = BLUE, const scales &scale = REGULAR);
  bool publishLine(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                   const colors &color = BLUE, const scales &scale = REGULAR);
  bool publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                   const colors &color = BLUE, const scales &scale = REGULAR);

  /**
   * \brief Display a marker of a series of connected lines
   * \param path - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPath(const std::vector<geometry_msgs::Point> &path, const colors &color = RED,
                   const scales &scale = REGULAR,
                   const std::string& ns = "Path");

  /**
   * \brief Display a marker of a polygon
   * \param polygon - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPolygon(const geometry_msgs::Polygon &polygon, const colors &color = RED,
                      const scales &scale = REGULAR,
                      const std::string& ns = "Polygon");

  /**
   * \brief Display a marker of a block
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param size - height=width=depth=size
   * \return true on success
   */
  bool publishBlock(const geometry_msgs::Pose &pose, const colors &color = BLUE, const double &block_size = 0.1);
  bool publishBlock(const Eigen::Affine3d &pose, const colors &color = BLUE, const double &block_size = 0.1);

  /**
   * \brief Publish transformed wireframe cuboid. Useful eg to show an oriented bounding box.
   * \param pose - cuboid vertices are transformed according to it
   * \param depth - object depth
   * \param width - object width
   * \param height - object height
   * \param color - an enum pre-defined name of a color
   * \param ns - namespace
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults to incremental counter
   * \return true on success
   */
  bool publishWireframeCuboid(const Eigen::Affine3d &pose, double depth, double width,
                              double height, const rviz_visual_tools::colors &color = BLUE,
                              const std::string& ns = "Wireframe Cuboid", const std::size_t &id = 0);

  /**
   * \brief Publish transformed wireframe cuboid. Useful eg to show an oriented bounding box.
   * \param pose - cuboid vertices are transformed according to it
   * \param min_point - minimum x, y, z coordinates
   * \param max_point - maximum x, y, z coordinates
   * \param color - an enum pre-defined name of a color
   * \param ns - namespace
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults to incremental counter
   * \return true on success
   */
  bool publishWireframeCuboid(const Eigen::Affine3d &pose, const Eigen::Vector3d &min_point,
                              const Eigen::Vector3d &max_point, const rviz_visual_tools::colors &color = BLUE,
                              const std::string& ns = "Wireframe Cuboid", const std::size_t &id = 0);

  /**
   * \brief Publish outline of a rectangle
   * \param pose - cuboid vertices are transformed according to it
   * \param height
   * \param width
   * \param color - an enum pre-defined name of a color
   * \return true on success
   */
  bool publishWireframeRectangle(const Eigen::Affine3d &pose, const double& height, const double& width,
                                 const colors &color = BLUE, const scales &scale = REGULAR);
  bool publishWireframeRectangle(const Eigen::Affine3d &pose,
                                 const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                 const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                 const colors &color, const scales &scale);
  /**
   * \brief Display a marker of a axis with a text label describing it
   * \param pose - the location to publish the marker with respect to the base frame
   * \param label - name of axis/coordinate frame
   * \param scale - size of axis
   * \return true on success
   */
  bool publishAxisLabeled(const Eigen::Affine3d &pose, const std::string& label, const scales &scale = REGULAR);
  bool publishAxisLabeled(const geometry_msgs::Pose &pose, const std::string& label, const scales &scale = REGULAR);

  /**
   * \brief Display a marker of a axis
   * \param pose - the location to publish the marker with respect to the base frame
   * \param length - geometry of cylinder
   * \param radius - geometry of cylinder
   * \return true on success
   */
  bool publishAxis(const geometry_msgs::Pose &pose, double length = 0.1, double radius = 0.01, const std::string& ns = "Axis");
  bool publishAxis(const Eigen::Affine3d &pose, double length = 0.1, double radius = 0.01, const std::string& ns = "Axis");

  /**
   * \brief Display a marker of a cylinder
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param height - geometry of cylinder
   * \param radius - geometry of cylinder
   * \return true on success
   */
  bool publishCylinder(const Eigen::Affine3d &pose, const colors &color = BLUE, double height = 0.1,
                       double radius = 0.01, const std::string& ns = "Cylinder");
  bool publishCylinder(const geometry_msgs::Pose &pose, const colors &color = BLUE, double height = 0.1,
                       double radius = 0.01, const std::string& ns = "Cylinder");

  /**
   * \brief Display a mesh from file
   * \param pose - the location to publish the marker with respect to the base frame
   * \param file name of mesh, starting with "file://"
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults to incremental counter
   * \return true on success
   */
  bool publishMesh(const Eigen::Affine3d &pose, const std::string& file_name, const colors &color = CLEAR,
                   double scale = 1, const std::string &ns = "mesh", const std::size_t &id = 0);
  bool publishMesh(const geometry_msgs::Pose &pose, const std::string& file_name, const colors &color = CLEAR,
                   double scale = 1, const std::string &ns = "mesh", const std::size_t &id = 0);

  /**
   * \brief Display a graph
   * \param graph of nodes and edges
   * \param color - an enum pre-defined name of a color
   * \param radius - width of cylinders
   * \return true on success
   */
  bool publishGraph(const graph_msgs::GeometryGraph &graph, const colors &color, double radius);

  /**
   * \brief Display a marker of a text
   * \param pose - the location to publish the marker with respect to the base frame
   * \param text - what to display
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param static_id - if true, only one text can be published at a time
   * \return true on success
   */
  bool publishText(const Eigen::Affine3d &pose, const std::string &text,
                   const colors &color = WHITE, const scales &scale = REGULAR, bool static_id = true);

  bool publishText(const geometry_msgs::Pose &pose, const std::string &text,
                   const colors &color = WHITE, const scales &scale = REGULAR, bool static_id = true);

  bool publishText(const geometry_msgs::Pose &pose, const std::string &text,
                   const colors &color, const geometry_msgs::Vector3 scale, bool static_id = true);

  /**
   * \brief Run a simple test of all visual_tool's features
   * \return true on success
   */
  bool publishTests();

  /**
   * \brief Convert an Eigen pose to a geometry_msg pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted pose
   */
  geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose);

  /**
   * \brief Convert a geometry_msg pose to an Eigen pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted pose
   */
  Eigen::Affine3d convertPose(const geometry_msgs::Pose &pose);

  /**
   * \brief Convert a geometry_msg point (32bit) to an Eigen pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted point with default rotation matrix
   */
  Eigen::Affine3d convertPoint32ToPose(const geometry_msgs::Point32 &point);

  /**
   * \brief
   * \param input - description
   * \param input - description
   * \return
   */
  geometry_msgs::Pose convertPointToPose(const geometry_msgs::Point &point);

  /**
   * \brief Convert an Eigen pose to a geometry_msg point
   *        Note: NOT memory efficient
   * \param pose
   * \return converted point with orientation discarded
   */
  geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  Eigen::Vector3d convertPoint(const geometry_msgs::Point &point);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  Eigen::Vector3d convertPoint32(const geometry_msgs::Point32 &point);

  /**
   * \brief Convert an Eigen point to a 32 bit geometry_msg point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  geometry_msgs::Point32 convertPoint32(const Eigen::Vector3d &point);

  /**
   * \brief Convert a Vector3 to a geometry_msg Point
   *        Note: NOT memory efficient
   * \param point
   * \return converted point
   */
  geometry_msgs::Point convertPoint(const geometry_msgs::Vector3 &point);

  /**
   * \brief Convert a Eigen point to a geometry_msg Point
   *        Note: NOT memory efficient
   * \param point
   * \return converted point
   */
  geometry_msgs::Point convertPoint(const Eigen::Vector3d &point);

  /**
   * \brief Convert a 6-vector of x,y,z, roll,pitch,yall to an Affine3d with quaternion using Euler ZXY convention
   * \return 4x4 matrix in form of affine3d
   */
  static Eigen::Affine3d convertXYZRPY(const double& x, const double& y, const double& z,
                                       const double& roll, const double& pitch, const double& yaw);
  static Eigen::Affine3d convertXYZRPY(std::vector<double> transform6);

  /**
   * \brief Convert an affine3d to xyz rpy components
   */
  static void convertToXYZRPY(const Eigen::Affine3d& pose, double& x, double& y, double& z, 
                              double& roll, double& pitch, double& yaw);
  /**
   * \brief Create a random pose within bounds of random_pose_bounds_
   * \param Pose to fill in
   * \parma options bounds on the pose to generate
   */
  void generateRandomPose(geometry_msgs::Pose& pose, RandomPoseBounds pose_bounds = RandomPoseBounds());
  void generateRandomPose(Eigen::Affine3d& pose, RandomPoseBounds pose_bounds = RandomPoseBounds());

  /**
   * \brief Create a random rectangular cuboid of some shape
   */
  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& depth, double& width, double& height,
                            RandomPoseBounds pose_bounds = RandomPoseBounds(), RandomCuboidBounds cuboid_bounds = RandomCuboidBounds());

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

protected:

  /**
   * \brief Allows certain marker functions to batch publish without breaking external functinality
   */
  void enableInternalBatchPublishing(bool enable);

  /**
   * \brief Trigger the publish function to send out all collected markers. Also then turns off the batch mode. This is safer
   *        incase programmer forgets. This is the internal version
   * \return true on success
   */
  bool triggerInternalBatchPublishAndDisable();

  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher pub_rviz_markers_; // for rviz visualization markers

  // Strings
  std::string marker_topic_; // topic to publish to rviz
  std::string base_frame_; // name of base link

  // TODO rename this
  double floor_to_base_height_; // allows an offset between base link and floor where objects are built

  // Duration to have Rviz markers persist, 0 for infinity
  ros::Duration marker_lifetime_; // TODO remove this  

  // Settings
  bool batch_publishing_enabled_;
  bool internal_batch_publishing_enabled_; // this allows certain marker functions to batch publish without breaking external functinality
  double alpha_; // opacity of all markers
  double global_scale_; // allow all markers to be increased by a constanct factor

  // Cached Rviz Marker Array
  visualization_msgs::MarkerArray markers_;

  // Cached Rviz markers
  visualization_msgs::Marker arrow_marker_;
  visualization_msgs::Marker sphere_marker_;
  visualization_msgs::Marker block_marker_;
  visualization_msgs::Marker cylinder_marker_;
  visualization_msgs::Marker mesh_marker_;
  visualization_msgs::Marker text_marker_;
  visualization_msgs::Marker cuboid_marker_;
  visualization_msgs::Marker line_marker_;
  visualization_msgs::Marker line_list_marker_;
  visualization_msgs::Marker spheres_marker_;
  visualization_msgs::Marker reset_marker_;
  visualization_msgs::Marker triangle_marker_;

  // Cached geometry variables used for conversion
  geometry_msgs::Pose shared_pose_msg_;
  geometry_msgs::Point shared_point_msg_;
  geometry_msgs::Point32 shared_point32_msg_;
  Eigen::Affine3d shared_pose_eigen_;
  Eigen::Vector3d shared_point_eigen_;
  
}; // class

typedef boost::shared_ptr<RvizVisualTools> RvizVisualToolsPtr;
typedef boost::shared_ptr<const RvizVisualTools> RvizVisualToolsConstPtr;

} // namespace

#endif
