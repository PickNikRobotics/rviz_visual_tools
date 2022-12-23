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

/* Author: Dave Coleman <dave@picknik.ai>, Andy McEvoy
   Desc:   Helper functions for displaying basic shape markers in Rviz
*/

#pragma once

// rviz_visual_tools
#include <rviz_visual_tools/remote_control.hpp>

#include <rclcpp/rclcpp.hpp>

// C++
#include <string>
#include <vector>

// Eigen
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

// Rviz
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Messages
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/color_rgba.hpp>
// #include <graph_msgs/msg/geometry_graph.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS      // ros is being built around shared libraries
#ifdef rviz_visual_tools_EXPORTS  // we are building a shared lib/dll
#define RVIZ_VISUAL_TOOLS_DECL ROS_HELPER_EXPORT
#else  // we are using shared lib/dll
#define RVIZ_VISUAL_TOOLS_DECL ROS_HELPER_IMPORT
#endif
#else  // ros is being built around static libraries
#define RVIZ_VISUAL_TOOLS_DECL
#endif

namespace rviz_visual_tools
{
// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
static const double SMALL_SCALE = 0.001;
static const double LARGE_SCALE = 100;

// Note: when adding new colors to colors, also add them to getRandColor() function
enum Colors
{
  BLACK = 0,
  BROWN = 1,
  BLUE = 2,
  CYAN = 3,
  GREY = 4,
  DARK_GREY = 5,
  GREEN = 6,
  LIME_GREEN = 7,
  MAGENTA = 8,
  ORANGE = 9,
  PURPLE = 10,
  RED = 11,
  PINK = 12,
  WHITE = 13,
  YELLOW = 14,
  TRANSLUCENT = 15,
  TRANSLUCENT_LIGHT = 16,
  TRANSLUCENT_DARK = 17,
  RAND = 18,
  CLEAR = 19,
  DEFAULT = 20  // i.e. 'do not change default color'
};

enum Scales
{
  XXXXSMALL = 1,
  XXXSMALL = 2,
  XXSMALL = 3,
  XSMALL = 4,
  SMALL = 5,
  MEDIUM = 6,  // same as REGULAR
  LARGE = 7,
  XLARGE = 8,
  XXLARGE = 9,
  XXXLARGE = 10,
  XXXXLARGE = 11,
};

enum EulerConvention
{
  XYZ = 0,
  ZYX,  // This is the ROS standard: http://www.ros.org/reps/rep-0103.html
  ZXZ
};

/**
 * \brief Bounds for generateRandomPose()
 */
struct RandomPoseBounds
{
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;
  double elevation_min_, elevation_max_;
  double azimuth_min_, azimuth_max_;
  double angle_min_, angle_max_;

  RandomPoseBounds(double x_min = 0.0, double x_max = 1.0, double y_min = 0.0, double y_max = 1.0,
                   double z_min = 0.0, double z_max = 1.0, double elevation_min = 0.0,
                   double elevation_max = M_PI, double azimuth_min = 0.0,
                   double azimuth_max = 2 * M_PI, double angle_min = 0.0,
                   double angle_max = 2 * M_PI)
  {
    x_min_ = x_min;
    x_max_ = x_max;
    y_min_ = y_min;
    y_max_ = y_max;
    z_min_ = z_min;
    z_max_ = z_max;
    elevation_min_ = elevation_min;
    elevation_max_ = elevation_max;
    azimuth_min_ = azimuth_min;
    azimuth_max_ = azimuth_max;
    angle_min_ = angle_min;
    angle_max_ = angle_max;
  }
};

/**
 * \brief Bounds for generateRandomCuboid()
 */
struct RandomCuboidBounds
{
  double cuboid_size_min_, cuboid_size_max_;

  explicit RandomCuboidBounds(double cuboid_size_min = 0.02, double cuboid_size_max = 0.15)
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
   * \param node - a pointer to a ros node
   */
  template <typename NodePtr>
  explicit RvizVisualTools(const std::string& base_frame, const std::string& marker_topic,
                           NodePtr node, const RemoteControlPtr& remote_control = nullptr)
    : RvizVisualTools(base_frame, marker_topic, node->get_node_base_interface(),
                      node->get_node_topics_interface(), node->get_node_graph_interface(),
                      node->get_node_clock_interface(), node->get_node_logging_interface(),
                      remote_control)
  {
  }

  explicit RvizVisualTools(
      const std::string& base_frame, const std::string& marker_topic,
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr& node_base_interface,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
      const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr& graph_interface,
      const rclcpp::node_interfaces::NodeClockInterface::SharedPtr& clock_interface,
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface,
      const RemoteControlPtr& remote_control = nullptr);

  /**
   * \brief Deconstructor
   */
  ~RvizVisualTools()
  {
  }

  /**
   * \brief Tell Rviz to clear all markers on a particular display.
   * \param ns - the namespace of the marker to be deleted
   * \param id - the id of the marker to be deleted
   * \return true if we published a marker message
   */
  bool deleteMarker(const std::string& ns, std::size_t id);

  /**
   * \brief Tell Rviz to clear all markers on a particular display.
   * \return true if we published a marker message
   */
  bool deleteAllMarkers();

  /**
   * \brief Tell Rviz to clear all markers on a particular display in a particular namespace.
   * \return true if we published a marker message
   */
  bool deleteAllMarkers(const std::string& ns);

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

  /** \brief Set marker array topic */
  void setMarkerTopic(const std::string& topic)
  {
    marker_topic_ = topic;
  }

  /**
   * \brief Load publishers as needed
   * \param wait_for_subscriber - whether a sleep for loop should be used to check for connectivity
   * to an external subscriber before proceeding
   */
  void loadMarkerPub(bool wait_for_subscriber = true);

  /** \brief Optional blocking function to call *after* calling loadMarkerPub(). Allows you to do
   *         some intermediate processing before wasting cycles waiting for the marker pub to find a
   * subscriber
   * \param wait_time - Wait some amount of time for a subscriber before returning. Method will
   * early exit if a subscriber is found before wait_time. Set to 0 to disable waiting and simply
   * return if a subscriber exists.
   * \return - true if a subscriber is found
   */
  bool waitForMarkerSub(double wait_time = 5);
  [[deprecated("waitForMarkerPub deprecated. Use waitForMarkerSub instad")]] bool
  waitForMarkerPub(double wait_time = 5)
  {
    return waitForMarkerSub(wait_time);
  }

  /**
   * \brief Wait until at least one subscriber connects to a publisher
   * \param pub - the publisher to check for subscribers
   * \param wait_time - time to wait for subscriber to be available before throwing warning (sec)
   * \return true on successful connection
   */
  template <typename MessageT>
  bool waitForSubscriber(std::shared_ptr<rclcpp::Publisher<MessageT> >& pub, double wait_time = 5.0)
  {
    // if the user does not want to wait return no connection
    if (!wait_for_subscriber_)
    {
      return false;
    }
    // Will wait at most this amount of time
    rclcpp::Time max_time(clock_interface_->get_clock()->now() +
                          rclcpp::Duration::from_seconds(wait_time));
    // This is wrong. It returns only the number of subscribers that have already
    // established their direct connections to this publisher
    // How often to check for subscribers
    rclcpp::Duration loop_duration = rclcpp::Duration::from_seconds(1.0 / 200.0);
    if (!pub)
    {
      RCLCPP_ERROR(logger_,
                   "loadMarkerPub() has not been called yet, unable to wait for subscriber.");
    }
    std::string topic_name = pub->get_topic_name();
    int num_existing_subscribers = graph_interface_->count_subscribers(topic_name);
    if (wait_time > 0 && num_existing_subscribers == 0)
    {
      RCLCPP_INFO_STREAM(logger_, "Topic " << pub->get_topic_name() << " waiting " << wait_time
                                           << " seconds for subscriber.");
    }
    // Wait for subscriber
    while (wait_time > 0 && num_existing_subscribers == 0 && rclcpp::ok())
    {
      if (clock_interface_->get_clock()->now() > max_time)  // Check if timed out
      {
        RCLCPP_WARN_STREAM(
            logger_,
            "Topic " << pub->get_topic_name() << " unable to connect to any subscribers within "
                     << wait_time
                     << " sec. It is possible initially published visual messages will be lost.");
        pub_rviz_markers_connected_ = false;
        return pub_rviz_markers_connected_;
      }
      // Check again
      num_existing_subscribers = graph_interface_->count_subscribers(topic_name);
      // Sleep
      rclcpp::sleep_for(std::chrono::nanoseconds(loop_duration.nanoseconds()));
    }
    if (!rclcpp::ok())
    {
      pub_rviz_markers_connected_ = false;
      return false;
    }
    pub_rviz_markers_connected_ = (num_existing_subscribers != 0);
    return pub_rviz_markers_connected_;
  }

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
   * \return Random color from colors
   */
  static Colors getRandColor();

  /**
   * \brief Get the RGB value of standard colors
   * \param color - an enum pre-defined name of a color
   * \return the RGB message equivalent
   */
  std_msgs::msg::ColorRGBA getColor(Colors color) const;

  /** \brief Used by interfaces that do not directly depend on Rviz Visual Tools, such as OMPL */
  static Colors intToRvizColor(std::size_t color);

  /** \brief Used by interfaces that do not directly depend on Rviz Visual Tools, such as OMPL */
  static rviz_visual_tools::Scales intToRvizScale(std::size_t scale);

  /** \brief Convert an enum to its string name equivalent */
  static std::string scaleToString(Scales scale);

  /**
   * \brief Create a random color that is not too light
   * \return the RGB message of a random color
   */
  std_msgs::msg::ColorRGBA createRandColor() const;

  /**
   * \brief Interpolate from [start, end] with value of size steps with current value count
   * \return interpolated value
   */
  static double slerp(double start, double end, double range, double value);

  /**
   * \brief Convert a value from [0,1] to a color Green->Red
   * \return interpolated color
   */
  std_msgs::msg::ColorRGBA getColorScale(double value) const;

  /**
   * \brief Get the rviz marker scale of standard sizes
   * \param scale - an enum pre-defined name of a size
   * \param marker_scale - amount to scale the scale for accounting for different types of markers
   * \return vector of 3 scales
   */
  geometry_msgs::msg::Vector3 getScale(Scales scale, double marker_scale = 1.0) const;

  /**
   * \brief Create a vector that points from point a to point b
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \return vector from a to b
   */
  Eigen::Isometry3d getVectorBetweenPoints(const Eigen::Vector3d& a,
                                           const Eigen::Vector3d& b) const;

  /**
   * \brief Find the center between to points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \return center point
   */
  Eigen::Vector3d getCenterPoint(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

  /**
   * \brief Get the base frame
   * \return name of base frame
   */
  const std::string getBaseFrame() const
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
  double getGlobalScale() const
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
  bool publishMarker(visualization_msgs::msg::Marker& marker);

  /**
   * \brief Enable batch publishing - useful for when many markers need to be published at once and
   * the ROS topic can get overloaded. This collects all published markers into array and only
   * publishes
   * them with trigger() is called
   */
  void enableBatchPublishing(bool enable = true);

  /**
   * \brief Enable frame locking - useful for when the markers is attached to a moving TF, the
   * marker will be
   * re-transformed into its frame every time-step
   */
  void enableFrameLocking(bool enable = true);

  /**
   * \brief Trigger the publish function to send out all collected markers IF there are at least
   *        queueSize number of markers ready to be published.
   *        Warning: when using this in a loop be sure to call trigger() at end of loop
   *        in case there are any remainder markers in the queue
   * \return true on success
   */
  bool triggerEvery(std::size_t queueSize);

  /**
   * \brief Trigger the publish function to send out all collected markers
   * \return true on success
   */
  bool trigger();

  /**
   * \brief Display an array of markers, allows reuse of the ROS publisher
   * \param markers
   * \return true on success
   */
  bool publishMarkers(visualization_msgs::msg::MarkerArray& markers);

  /**
   * \brief Display a cone of a given angle along the x-axis
   * \param pose - the location and orientation of the cone
   * \param color - color of the cone
   * \param scale - size of the cone
   * \return true on success
   */
  bool publishCone(const Eigen::Isometry3d& pose, double angle, Colors color = TRANSLUCENT,
                   double scale = 1.0);
  bool publishCone(const geometry_msgs::msg::Pose& pose, double angle, Colors color = TRANSLUCENT,
                   double scale = 1.0);

  /**
   * \brief Display a plane. Vector (A, B, C) gives the normal to the plane.
   *        |D|/|(A,B,C)| gives the distance to plane along that unit normal.
   *        The plane equation used is Ax+By+Cz+D=0.
   * \param A - coefficient from Ax+By+Cz+D=0
   * \param B - coefficient from Ax+By+Cz+D=0
   * \param C - coefficient from Ax+By+Cz+D=0
   * \param D - coefficient from Ax+By+Cz+D=0
   * \param color - the color of the plane
   * \param x_width - X-size of the vizualized plane [meters]
   * \param y_width - Y-size of the visualized plane [meters]
   * \return true on success
   */
  bool publishABCDPlane(const double A, const double B, const double C, const double D,
                        Colors color = TRANSLUCENT, double x_width = 1.0, double y_width = 1.0);

  /**
   * \brief Display a plane given a vector normal to the plane and the distance to the plane along
   *        that normal. The plane normal does not need to be given as a unit vector.
   * \param normal - a vector representing the normal of the plane
   * \param d - the distance to the plane along the vector
   * \param color - the color of the plane
   * \param x_width - X-size of the vizualized plane [meters]
   * \param y_width - Y-size of the visualized plane [meters]
   * \return true on success
   */
  bool publishNormalAndDistancePlane(const Eigen::Vector3d& normal, const double d,
                                     const Colors color = TRANSLUCENT, const double x_width = 1.0,
                                     const double y_width = 1.0);

  /**
   * \brief Display the XY plane of a given pose
   * \param pose - the position of the plane
   * \param color - the color of the plane
   * \param scale - the size of the vizualized plane
   * \return true on success
   */
  bool publishXYPlane(const Eigen::Isometry3d& pose, Colors color = TRANSLUCENT,
                      double scale = 1.0);
  bool publishXYPlane(const geometry_msgs::msg::Pose& pose, Colors color = TRANSLUCENT,
                      double scale = 1.0);

  /**
   * \brief Display the XZ plane of a given pose
   * \param pose - the position of the plane
   * \param color - the color of the plane
   * \param scale - the size of the vizualized plane
   * \return true on success
   */
  bool publishXZPlane(const Eigen::Isometry3d& pose, Colors color = TRANSLUCENT,
                      double scale = 1.0);
  bool publishXZPlane(const geometry_msgs::msg::Pose& pose, Colors color = TRANSLUCENT,
                      double scale = 1.0);

  /**
   * \brief Display the YZ plane of a given pose
   * \param pose - the position of the plane
   * \param color - the color of the plane
   * \param scale - the size of the vizualized plane
   * \return true on success
   */
  bool publishYZPlane(const Eigen::Isometry3d& pose, Colors color = TRANSLUCENT,
                      double scale = 1.0);
  bool publishYZPlane(const geometry_msgs::msg::Pose& pose, Colors color = TRANSLUCENT,
                      double scale = 1.0);

  /**
   * \brief Display a marker of a sphere
   * \param pose - the location to publish the sphere with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults
   * to incremental counter
   * \return true on success
   */
  bool publishSphere(const Eigen::Isometry3d& pose, Colors color = BLUE, Scales scale = MEDIUM,
                     const std::string& ns = "Sphere", std::size_t id = 0);
  bool publishSphere(const Eigen::Vector3d& point, Colors color = BLUE, Scales scale = MEDIUM,
                     const std::string& ns = "Sphere", std::size_t id = 0);
  bool publishSphere(const Eigen::Vector3d& point, Colors color, double scale,
                     const std::string& ns = "Sphere", std::size_t id = 0);
  bool publishSphere(const geometry_msgs::msg::Point& point, Colors color = BLUE,
                     Scales scale = MEDIUM, const std::string& ns = "Sphere", std::size_t id = 0);
  bool publishSphere(const geometry_msgs::msg::Pose& pose, Colors color = BLUE,
                     Scales scale = MEDIUM, const std::string& ns = "Sphere", std::size_t id = 0);
  bool publishSphere(const geometry_msgs::msg::Pose& pose, Colors color, double scale,
                     const std::string& ns = "Sphere", std::size_t id = 0);
  bool publishSphere(const geometry_msgs::msg::Pose& pose, Colors color,
                     const geometry_msgs::msg::Vector3 scale, const std::string& ns = "Sphere",
                     std::size_t id = 0);
  bool publishSphere(const geometry_msgs::msg::Pose& pose, const std_msgs::msg::ColorRGBA& color,
                     const geometry_msgs::msg::Vector3 scale, const std::string& ns = "Sphere",
                     std::size_t id = 0);
  bool publishSphere(const Eigen::Isometry3d& pose, const std_msgs::msg::ColorRGBA& color,
                     const geometry_msgs::msg::Vector3 scale, const std::string& ns = "Sphere",
                     std::size_t id = 0);
  bool publishSphere(const Eigen::Vector3d& point, const std_msgs::msg::ColorRGBA& color,
                     const geometry_msgs::msg::Vector3 scale, const std::string& ns = "Sphere",
                     std::size_t id = 0);
  bool publishSphere(const geometry_msgs::msg::PoseStamped& pose, Colors color,
                     const geometry_msgs::msg::Vector3 scale, const std::string& ns = "Sphere",
                     std::size_t id = 0);

  /**
   * \brief Display a marker of a series of spheres
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const EigenSTL::vector_Vector3d& points, Colors color = BLUE,
                      Scales scale = MEDIUM, const std::string& ns = "Spheres");
  bool publishSpheres(const EigenSTL::vector_Vector3d& points, Colors color, double scale = 0.1,
                      const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::msg::Point>& points, Colors color = BLUE,
                      Scales scale = MEDIUM, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::msg::Point>& points, Colors color = BLUE,
                      double scale = 0.1, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::msg::Point>& points, Colors color,
                      const geometry_msgs::msg::Vector3& scale, const std::string& ns = "Spheres");

  /**
   * \brief Display a marker of a series of spheres, with the possibility of different colors
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const EigenSTL::vector_Vector3d& points, const std::vector<Colors>& colors,
                      Scales scale = MEDIUM, const std::string& ns = "Spheres");
  bool publishSpheres(const std::vector<geometry_msgs::msg::Point>& points,
                      const std::vector<std_msgs::msg::ColorRGBA>& colors,
                      const geometry_msgs::msg::Vector3& scale, const std::string& ns = "Spheres");

  /**
   * \brief Display an arrow along the x-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - the length of the arrow tail, if zero, will auto set with scale
   * \return true on success
   */
  bool publishXArrow(const Eigen::Isometry3d& pose, Colors color = RED, Scales scale = MEDIUM,
                     double length = 0.0);
  bool publishXArrow(const geometry_msgs::msg::Pose& pose, Colors color = RED,
                     Scales scale = MEDIUM, double length = 0.0);
  bool publishXArrow(const geometry_msgs::msg::PoseStamped& pose, Colors color = RED,
                     Scales scale = MEDIUM, double length = 0.0);

  /**
   * \brief Display an arrow along the y-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - the length of the arrow tail, if zero, will auto set with scale
   * \return true on success
   */
  bool publishYArrow(const Eigen::Isometry3d& pose, Colors color = GREEN, Scales scale = MEDIUM,
                     double length = 0.0);
  bool publishYArrow(const geometry_msgs::msg::Pose& pose, Colors color = GREEN,
                     Scales scale = MEDIUM, double length = 0.0);
  bool publishYArrow(const geometry_msgs::msg::PoseStamped& pose, Colors color = GREEN,
                     Scales scale = MEDIUM, double length = 0.0);

  /**
   * \brief Display an arrow along the z-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - the length of the arrow tail, if zero, will auto set with scale
   * \return true on success
   */
  bool publishZArrow(const Eigen::Isometry3d& pose, Colors color = BLUE, Scales scale = MEDIUM,
                     double length = 0.0, std::size_t id = 0);
  bool publishZArrow(const geometry_msgs::msg::Pose& pose, Colors color = BLUE,
                     Scales scale = MEDIUM, double length = 0.0);
  bool publishZArrow(const geometry_msgs::msg::PoseStamped& pose, Colors color = BLUE,
                     Scales scale = MEDIUM, double length = 0.0);
  bool publishZArrow(const geometry_msgs::msg::PoseStamped& pose, Colors color = BLUE,
                     Scales scale = MEDIUM, double length = 0.0, std::size_t id = 0);

  /**
   * \brief Display an arrow along the x-axis of a pose
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param length - how long the arrow tail should be. if zero, will auto set with scale
   * \param start - the starting point of the arrow
   * \param end - the ending point of the arrow
   * \return true on success
   */
  bool publishArrow(const Eigen::Isometry3d& pose, Colors color = BLUE, Scales scale = MEDIUM,
                    double length = 0.0, std::size_t id = 0);
  bool publishArrow(const geometry_msgs::msg::Pose& pose, Colors color = BLUE,
                    Scales scale = MEDIUM, double length = 0.0, std::size_t id = 0);
  bool publishArrow(const geometry_msgs::msg::PoseStamped& pose, Colors color = BLUE,
                    Scales scale = MEDIUM, double length = 0.0, std::size_t id = 0);
  bool publishArrow(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end,
                    Colors color = BLUE, Scales scale = MEDIUM, std::size_t id = 0);

  /**
   * \brief Display a rectangular cuboid
   * \param point1 - x,y,z top corner location of box
   * \param point2 - x,y,z bottom opposite corner location of box
   * \param color - an enum pre-defined name of a color or a color message
   * \return true on success
   */
  bool publishCuboid(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                     Colors color = BLUE);
  bool publishCuboid(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                     const std_msgs::msg::ColorRGBA& color);
  bool publishCuboid(const geometry_msgs::msg::Point& point1,
                     const geometry_msgs::msg::Point& point2, Colors color = BLUE,
                     const std::string& ns = "Cuboid", std::size_t id = 0);
  bool publishCuboid(const geometry_msgs::msg::Point& point1,
                     const geometry_msgs::msg::Point& point2, const std_msgs::msg::ColorRGBA& color,
                     const std::string& ns = "Cuboid", std::size_t id = 0);

  /**
   * \brief Display a rectangular cuboid
   * \param pose - pose of the box
   * \param depth - depth of the box
   * \param width - width of the box
   * \param height - height of the box
   * \param color - an enum pre-defined name of a color or a color message
   * \return true on success
   */
  bool publishCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                     const std_msgs::msg::ColorRGBA& color);
  bool publishCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                     Colors color = BLUE);
  bool publishCuboid(const geometry_msgs::msg::Pose& pose, double depth, double width,
                     double height, Colors color = BLUE);
  bool publishCuboid(const geometry_msgs::msg::Pose& pose, double depth, double width,
                     double height, const std_msgs::msg::ColorRGBA& color);

  /**
   * \brief Display a marker of line
   * \param point1 - x,y,z of start of line
   * \param point2 - x,y,z of end of line
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishLine(const Eigen::Isometry3d& point1, const Eigen::Isometry3d& point2,
                   Colors color = BLUE, Scales scale = MEDIUM);
  bool publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                   Colors color = BLUE, Scales scale = MEDIUM);
  bool publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, Colors color,
                   double radius);
  bool publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                   const std_msgs::msg::ColorRGBA& color, Scales scale = MEDIUM);
  bool publishLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                   const std_msgs::msg::ColorRGBA& color, double radius);
  bool publishLine(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2,
                   Colors color = BLUE, Scales scale = MEDIUM);
  bool publishLine(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2,
                   const std_msgs::msg::ColorRGBA& color, Scales scale = MEDIUM);
  bool publishLine(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2,
                   const std_msgs::msg::ColorRGBA& color, const geometry_msgs::msg::Vector3& scale);

  /**
   * \brief Display a marker of lines
   * \param aPoints - x,y,z of start of line, as a vector
   * \param bPoints - x,y,z of end of line, as a vector
   * \param colors - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishLines(const EigenSTL::vector_Vector3d& aPoints,
                    const EigenSTL::vector_Vector3d& bPoints, const std::vector<Colors>& colors,
                    Scales scale = MEDIUM);
  bool publishLines(const std::vector<geometry_msgs::msg::Point>& aPoints,
                    const std::vector<geometry_msgs::msg::Point>& bPoints,
                    const std::vector<std_msgs::msg::ColorRGBA>& colors,
                    const geometry_msgs::msg::Vector3& scale);

  /**
   * \brief Display a series of connected lines using the LINE_STRIP method - deprecated because
   * visual bugs
   * \param path - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishLineStrip(const std::vector<geometry_msgs::msg::Point>& path, Colors color = RED,
                        Scales scale = MEDIUM, const std::string& ns = "Path");

  /**
   * \brief Display a marker of a series of connected cylinders
   * \param path - a series of points to connect with cylinders
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPath(const std::vector<geometry_msgs::msg::Pose>& path, Colors color = RED,
                   Scales scale = MEDIUM, const std::string& ns = "Path");
  bool publishPath(const std::vector<geometry_msgs::msg::Point>& path, Colors color, Scales scale,
                   const std::string& ns = "Path");
  bool publishPath(const EigenSTL::vector_Isometry3d& path, Colors color, Scales scale,
                   const std::string& ns = "Path");
  bool publishPath(const EigenSTL::vector_Vector3d& path, Colors color, Scales scale,
                   const std::string& ns = "Path");
  bool publishPath(const std::vector<geometry_msgs::msg::Point>& path, Colors color = RED,
                   double radius = 0.01, const std::string& ns = "Path");
  bool publishPath(const EigenSTL::vector_Vector3d& path, Colors color = RED, double radius = 0.01,
                   const std::string& ns = "Path");
  bool publishPath(const EigenSTL::vector_Isometry3d& path, Colors color = RED,
                   double radius = 0.01, const std::string& ns = "Path");

  /**
   * \brief Display a marker of a series of connected colored cylinders
   * \param path - a series of points to connect with cylinders
   * \param colors - a series of colors
   * \param radius - the radius of the cylinders
   * \param ns - namespace of marker
   * \return true on success
   * \note path and colors vectors must be the same size
   */
  bool publishPath(const EigenSTL::vector_Vector3d& path, const std::vector<Colors>& colors,
                   double radius = 0.01, const std::string& ns = "Path");

  bool publishPath(const EigenSTL::vector_Vector3d& path,
                   const std::vector<std_msgs::msg::ColorRGBA>& colors, double radius,
                   const std::string& ns = "Path");

  /**
   * \brief Display a marker of a polygon
   * \param polygon - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPolygon(const geometry_msgs::msg::Polygon& polygon, Colors color = RED,
                      Scales scale = MEDIUM, const std::string& ns = "Polygon");

  /**
   * \brief Publish transformed wireframe cuboid. Useful eg to show an oriented bounding box.
   * \param pose - cuboid vertices are transformed according to it
   * \param depth - object depth
   * \param width - object width
   * \param height - object height
   * \param color - an enum pre-defined name of a color
   * \param ns - namespace
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults
   * to incremental counter
   * \return true on success
   */
  bool publishWireframeCuboid(const Eigen::Isometry3d& pose, double depth, double width,
                              double height, Colors color = BLUE,
                              const std::string& ns = "Wireframe Cuboid", std::size_t id = 0);

  /**
   * \brief Publish transformed wireframe cuboid. Useful eg to show an oriented bounding box.
   * \param pose - cuboid vertices are transformed according to it
   * \param min_point - minimum x, y, z coordinates
   * \param max_point - maximum x, y, z coordinates
   * \param color - an enum pre-defined name of a color
   * \param ns - namespace
   * \param id - unique counter that allows you to overwrite a previous marker. if 0, defaults to
   * incremental counter
   * \return true on success
   */
  bool publishWireframeCuboid(const Eigen::Isometry3d& pose, const Eigen::Vector3d& min_point,
                              const Eigen::Vector3d& max_point, Colors color = BLUE,
                              const std::string& ns = "Wireframe Cuboid", std::size_t id = 0);

  /**
   * \brief Publish outline of a rectangle
   * \param pose - cuboid vertices are transformed according to it
   * \param height
   * \param width
   * \param color - an enum pre-defined name of a color
   * \param id - unique counter that allows you to overwrite a previous marker. if 0, defaults to
   * incremental counter
   * \return true on success
   */
  bool publishWireframeRectangle(const Eigen::Isometry3d& pose, double height, double width,
                                 Colors color = BLUE, Scales scale = MEDIUM, std::size_t id = 0);
  bool publishWireframeRectangle(const Eigen::Isometry3d& pose, const Eigen::Vector3d& p1_in,
                                 const Eigen::Vector3d& p2_in, const Eigen::Vector3d& p3_in,
                                 const Eigen::Vector3d& p4_in, Colors color, Scales scale);
  /**
   * \brief Display a marker of a coordinate frame axis with a text label describing it
   * \param pose - the location to publish the marker with respect to the base frame
   * \param label - name of axis/coordinate frame
   * \param scale - size of axis
   * \param color - an enum pre-defined name of a color
   * \return true on success
   */
  bool publishAxisLabeled(const Eigen::Isometry3d& pose, const std::string& label,
                          Scales scale = MEDIUM, Colors color = WHITE);
  bool publishAxisLabeled(const geometry_msgs::msg::Pose& pose, const std::string& label,
                          Scales scale = MEDIUM, Colors color = WHITE);

  /**
   * \brief Display a red/green/blue coordinate frame axis
   * \param pose - the location to publish the marker with respect to the base frame
   * \param scale - size of axis
   * \param length - geometry of cylinder
   * \param radius - geometry of cylinder
   * \param ns - namespace
   * \return true on success
   */
  bool publishAxis(const geometry_msgs::msg::Pose& pose, Scales scale = MEDIUM,
                   const std::string& ns = "Axis");
  bool publishAxis(const Eigen::Isometry3d& pose, Scales scale = MEDIUM,
                   const std::string& ns = "Axis");
  bool publishAxis(const geometry_msgs::msg::Pose& pose, double length, double radius = 0.01,
                   const std::string& ns = "Axis");
  bool publishAxis(const Eigen::Isometry3d& pose, double length, double radius = 0.01,
                   const std::string& ns = "Axis");

private:
  /**
   * \brief Display a red/green/blue coordinate axis - the 'internal' version does not do a batch
   * publish
   * \param pose - the location to publish the marker with respect to the base frame
   * \param length - geometry of cylinder
   * \param radius - geometry of cylinder
   * \param ns - namespace
   * \return true on success
   */
  bool publishAxisInternal(const Eigen::Isometry3d& pose, double length = 0.1, double radius = 0.01,
                           const std::string& ns = "Axis");

public:
  /**
   * \brief Display a series of red/green/blue coordinate axis along a path
   * \param path - the location to publish each marker with respect to the base frame
   * \param length - geometry of cylinder
   * \param radius - geometry of cylinder
   * \param ns - namespace
   * \return true on success
   */
  bool publishAxisPath(const EigenSTL::vector_Isometry3d& path, Scales scale = MEDIUM,
                       const std::string& ns = "Axis Path");
  bool publishAxisPath(const EigenSTL::vector_Isometry3d& path, double length = 0.1,
                       double radius = 0.01, const std::string& ns = "Axis Path");

  /**
   * \brief Display a marker of a cylinder
   * \param point1 - starting side of cylinder
   * \param point2 - end side of cylinder
   * \param color - an enum pre-defined name of a color
   * \param radius - geometry of cylinder
   * \return true on success
   */
  bool publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                       Colors color = BLUE, Scales scale = MEDIUM,
                       const std::string& ns = "Cylinder");
  bool publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, Colors color,
                       double radius = 0.01, const std::string& ns = "Cylinder");
  bool publishCylinder(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                       const std_msgs::msg::ColorRGBA& color, double radius = 0.01,
                       const std::string& ns = "Cylinder");

  /**
   * \brief Display a marker of a cylinder
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param height - geometry of cylinder
   * \param radius - geometry of cylinder
   * \return true on success
   */
  bool publishCylinder(const Eigen::Isometry3d& pose, Colors color = BLUE, double height = 0.1,
                       double radius = 0.01, const std::string& ns = "Cylinder");
  bool publishCylinder(const geometry_msgs::msg::Pose& pose, Colors color = BLUE,
                       double height = 0.1, double radius = 0.01,
                       const std::string& ns = "Cylinder");
  bool publishCylinder(const geometry_msgs::msg::Pose& pose, const std_msgs::msg::ColorRGBA& color,
                       double height = 0.1, double radius = 0.01,
                       const std::string& ns = "Cylinder");

  /**
   * \brief Display a mesh from file
   * \param pose - the location to publish the marker with respect to the base frame
   * \param file name of mesh, starting with "file://" e.g. "file:///home/user/mesh.stl"
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults
   * to incremental counter
   * \return true on success
   */
  bool publishMesh(const Eigen::Isometry3d& pose, const std::string& file_name,
                   Colors color = CLEAR, double scale = 1, const std::string& ns = "mesh",
                   std::size_t id = 0);
  bool publishMesh(const geometry_msgs::msg::Pose& pose, const std::string& file_name,
                   Colors color = CLEAR, double scale = 1, const std::string& ns = "mesh",
                   std::size_t id = 0);

  /**
   * \brief Display a mesh from triangles and vertices
   * \param pose - the location to publish the marker with respect to the base frame
   * \param mesh - shape_msgs::msg::Mesh contains the triangles and vertices
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \param id - unique counter of mesh that allows you to overwrite a previous mesh. if 0, defaults
   * to incremental counter
   * \return true on success
   */
  bool publishMesh(const Eigen::Isometry3d& pose, const shape_msgs::msg::Mesh& mesh,
                   Colors color = CLEAR, double scale = 1, const std::string& ns = "mesh",
                   std::size_t id = 0);
  bool publishMesh(const geometry_msgs::msg::Pose& pose, const shape_msgs::msg::Mesh& mesh,
                   Colors color = CLEAR, double scale = 1, const std::string& ns = "mesh",
                   std::size_t id = 0);

  /**
   * \brief Display a graph
   * \param graph of nodes and edges
   * \param color - an enum pre-defined name of a color
   * \param radius - width of cylinders
   * \return true on success
   */
  // TODO(mlautman): port graph_msgs
  // bool publishGraph(const graph_msgs::msg::GeometryGraph& graph, colors color, double radius);

  /**
   * \brief Display a marker of a text
   * \param pose - the location to publish the marker with respect to the base frame
   * \param text - what message to display
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param static_id - if true, only one text can be published at a time
   * \return true on success
   */
  bool publishText(const Eigen::Isometry3d& pose, const std::string& text, Colors color = WHITE,
                   Scales scale = MEDIUM, bool static_id = true);
  bool publishText(const Eigen::Isometry3d& pose, const std::string& text, Colors color,
                   const geometry_msgs::msg::Vector3 scale, bool static_id = true);
  bool publishText(const geometry_msgs::msg::Pose& pose, const std::string& text,
                   Colors color = WHITE, Scales scale = MEDIUM, bool static_id = true);
  bool publishText(const geometry_msgs::msg::Pose& pose, const std::string& text, Colors color,
                   const geometry_msgs::msg::Vector3 scale, bool static_id = true);

  /**
   * \brief Convert an Eigen pose to a geometry_msg pose
   * \param pose
   * \return converted pose
   */
  static geometry_msgs::msg::Pose convertPose(const Eigen::Isometry3d& pose);

  /**
   * \brief Convert an Eigen pose to a geometry_msg pose - thread safe
   * \param Eigen pose - input
   * \param ROS msg pose - output
   */
  static void convertPoseSafe(const Eigen::Isometry3d& pose, geometry_msgs::msg::Pose& pose_msg);

  /**
   * \brief Convert a geometry_msg pose to an Eigen pose
   *        Note: Not thread safe but very convenient
   * \param ROS msg pose
   * \return converted pose
   */
  static Eigen::Isometry3d convertPose(const geometry_msgs::msg::Pose& pose);

  /**
   * \brief Convert a geometry_msg pose to an Eigen pose - thread safe
   * \param ROS msg pose - input
   * \param Eigen pose - output
   */
  static void convertPoseSafe(const geometry_msgs::msg::Pose& pose_msg, Eigen::Isometry3d& pose);

  /**
   * \brief Convert a geometry_msg point (32bit) to an Eigen pose
   *        Note: Not thread safe but very convenient
   * \param pose
   * \return converted point with default rotation matrix
   */
  static Eigen::Isometry3d convertPoint32ToPose(const geometry_msgs::msg::Point32& point);

  /**
   * \brief Add an identity rotation matrix to make a point have a full pose
   */
  static geometry_msgs::msg::Pose convertPointToPose(const geometry_msgs::msg::Point& point);
  static Eigen::Isometry3d convertPointToPose(const Eigen::Vector3d& point);

  /**
   * \brief Convert an Eigen pose to a geometry_msg point
   *        Note: Not thread safe but very convenient
   * \param pose
   * \return converted point with orientation discarded
   */
  static geometry_msgs::msg::Point convertPoseToPoint(const Eigen::Isometry3d& pose);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: Not thread safe but very convenient
   * \param point
   * \return converted pose
   */
  static Eigen::Vector3d convertPoint(const geometry_msgs::msg::Point& point);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: Not thread safe but very convenient
   * \param point
   * \return converted pose
   */
  static Eigen::Vector3d convertPoint32(const geometry_msgs::msg::Point32& point);

  /**
   * \brief Convert an Eigen point to a 32 bit geometry_msg point
   *        Note: Not thread safe but very convenient
   * \param point
   * \return converted pose
   */
  static geometry_msgs::msg::Point32 convertPoint32(const Eigen::Vector3d& point);

  /**
   * \brief Convert a Vector3 to a geometry_msg Point
   *        Note: Not thread safe but very convenient
   * \param point
   * \return converted point
   */
  static geometry_msgs::msg::Point convertPoint(const geometry_msgs::msg::Vector3& point);

  /**
   * \brief Convert a Eigen point to a geometry_msg Point
   *        Note: Not thread safe but very convenient
   * \param point
   * \return converted point
   */
  static geometry_msgs::msg::Point convertPoint(const Eigen::Vector3d& point);

  /**
  @brief Converts scalar translations and rotations to an Eigen Frame.  This is achieved by chaining
  a
  translation with individual euler rotations in ZYX order (this is equivalent to fixed rotatins
  XYZ)
  http://en.wikipedia.org/wiki/Euler_angles#Conversion_between_intrinsic_and_extrinsic_rotations
  Euler conversion code sourced from Descartes, Copyright (c) 2014, Southwest Research Institute

  @param tx, ty, tz - translations in x, y, z respectively
  @param rx, ry, rz - rotations about x, y, z, respectively
  @param convention - default is rviz_visual_tools::XYZ
  */
  static Eigen::Isometry3d convertFromXYZRPY(double tx, double ty, double tz, double rx, double ry,
                                             double rz,
                                             EulerConvention convention);  // ZYX is ROS standard
  static Eigen::Isometry3d convertFromXYZRPY(const std::vector<double>& transform6,
                                             EulerConvention convention);  // ZYX is ROS standard

  // TODO(davetcoleman): add opposite conversion that uses   Eigen::Vector3d rpy =
  // pose.rotation().eulerAngles(0, 1, 2);

  /**
   * \brief Convert an affine3d to xyz rpy components
   *        R-P-Y / X-Y-Z / 0-1-2 Euler Angle Standard
   * \param input Eigen pose
   * \param output vector of size 6 in order xyz rpy
   */
  static void convertToXYZRPY(const Eigen::Isometry3d& pose, std::vector<double>& xyzrpy);
  static void convertToXYZRPY(const Eigen::Isometry3d& pose, double& x, double& y, double& z,
                              double& roll, double& pitch, double& yaw);
  /**
   * \brief Create a random pose within bounds of random_pose_bounds_
   * \param Pose to fill in
   * \parma options bounds on the pose to generate
   */
  static void generateRandomPose(geometry_msgs::msg::Pose& pose,
                                 RandomPoseBounds pose_bounds = RandomPoseBounds());
  static void generateRandomPose(Eigen::Isometry3d& pose,
                                 RandomPoseBounds pose_bounds = RandomPoseBounds());

  /**
   * \brief Create a random rectangular cuboid of some shape
   */
  static void generateRandomCuboid(geometry_msgs::msg::Pose& cuboid_pose, double& depth,
                                   double& width, double& height,
                                   RandomPoseBounds pose_bounds = RandomPoseBounds(),
                                   RandomCuboidBounds cuboid_bounds = RandomCuboidBounds());

  /**
   * \brief Create a pose of position (0,0,0) and quaternion (0,0,0,1)
   * \param Pose to fill in
   */
  static geometry_msgs::msg::Pose getIdentityPose();

  /**
   * \brief Test if two Eigen poses are close enough
   * \param pose1
   * \param pose2
   * \param threshold - how close in value they must be in order to be considered the same
   * \return true if equal
   */
  static bool posesEqual(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2,
                         double threshold = 0.000001);

  /**
   * \brief Get random between min and max
   */
  static double dRand(double min, double max);
  static float fRand(float min, float max);
  static int iRand(int min, int max);

  /**
   * \brief Display in the console the x,y,z values of a point
   */
  static void printTranslation(const Eigen::Vector3d& translation);

  /**
   * \brief Display in the console a transform in quaternions
   */
  static void printTransform(const Eigen::Isometry3d& transform);

  /**
   * \brief Display in the console a transform in roll pitch yaw
   */
  static void printTransformRPY(const Eigen::Isometry3d& transform);

  /**
   * \brief Display in the console a transform with full 3x3 rotation matrix
   */
  static void printTransformFull(const Eigen::Isometry3d& transform);

  /** \brief Getter for PsychedelicMode */
  bool getPsychedelicMode() const
  {
    return psychedelic_mode_;
  }

  /** \brief Setter for PsychedelicMode */
  void setPsychedelicMode(bool psychedelic_mode = true)
  {
    psychedelic_mode_ = psychedelic_mode;
  }

  /** \brief Wait for user feedback i.e. through a button or joystick */
  bool prompt(const std::string& msg);

  /** \brief Ability to load remote control on the fly */
  RemoteControlPtr& getRemoteControl();

  /** \brief Pre-load remote control */
  void setRemoteControl(const RemoteControlPtr& remote_control);

  /** \brief Ability to load remote control on the fly */
  void loadRemoteControl();

  /** \brief Get the latest ID of arrow_marker_ */
  int32_t getArrowId() const;

  /** \brief Get the latest ID of sphere_marker_ */
  int32_t getSphereId() const;

  /** \brief Get the latest ID of block_marker_*/
  int32_t getBlockId() const;

  /** \brief Get the latest ID of cylinder_marker_*/
  int32_t getCylinderId() const;

  /** \brief Get the latest ID of mesh_marker_*/
  int32_t getMeshId() const;

  /** \brief Get the latest ID of text_marker_*/
  int32_t getTextId() const;

  /** \brief Get the latest ID of cuboid_marker_*/
  int32_t getCuboidId() const;

  /** \brief Get the latest ID of line_strip_marker_*/
  int32_t getLineStripId() const;

  /** \brief Get the latest ID of line_list_marker_*/
  int32_t getLineListId() const;

  /** \brief Get the latest ID of spheres_marker_*/
  int32_t getSpheresId() const;

  /** \brief Get the latest ID of triangle_marker_*/
  int32_t getTriangleId() const;

protected:
  // Node Interfaces
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::Logger logger_;

  // Optional remote control
  RemoteControlPtr remote_control_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_rviz_markers_;  // for rviz visualization
                          // markers
  bool pub_rviz_markers_connected_ = false;
  bool pub_rviz_markers_waited_ = false;
  bool wait_for_subscriber_ = false;

  // Strings
  std::string marker_topic_;  // topic to publish to rviz
  std::string base_frame_;    // name of base link

  // Duration to have Rviz markers persist, 0 for infinity
  builtin_interfaces::msg::Duration marker_lifetime_;

  // Settings
  bool batch_publishing_enabled_ = true;
  bool frame_locking_enabled_ = false;
  double alpha_ = 1.0;         // opacity of all markers
  double global_scale_ = 1.0;  // allow all markers to be increased by a constanct factor

  // Cached Rviz Marker Array
  visualization_msgs::msg::MarkerArray markers_;

  // Cached Rviz markers
  visualization_msgs::msg::Marker arrow_marker_;
  visualization_msgs::msg::Marker sphere_marker_;
  visualization_msgs::msg::Marker block_marker_;
  visualization_msgs::msg::Marker cylinder_marker_;
  visualization_msgs::msg::Marker mesh_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::Marker cuboid_marker_;
  visualization_msgs::msg::Marker line_strip_marker_;
  visualization_msgs::msg::Marker line_list_marker_;
  visualization_msgs::msg::Marker spheres_marker_;
  visualization_msgs::msg::Marker reset_marker_;
  visualization_msgs::msg::Marker triangle_marker_;

  // Just for fun.
  bool psychedelic_mode_ = false;

  // Chose random colors from this list
  static const std::array<Colors, 14> ALL_RAND_COLORS;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
};                                 // class

typedef std::shared_ptr<RvizVisualTools> RvizVisualToolsPtr;
typedef std::shared_ptr<const RvizVisualTools> RvizVisualToolsConstPtr;

}  // namespace rviz_visual_tools
