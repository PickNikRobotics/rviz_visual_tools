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
   Desc:   Demo implementation of rviz_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /rviz_visual_tools
*/

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <rviz_visual_tools/remote_control.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// C++
#include <string>
#include <vector>

// Msgs
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;

namespace rvt = rviz_visual_tools;

namespace rviz_visual_tools
{
class RvizVisualToolsDemo : public rclcpp::Node
{
private:
  // For visualizing things in rviz
  rvt::RvizVisualToolsPtr visual_tools_;

public:
  /**
   * \brief Constructor
   */
  explicit RvizVisualToolsDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("rviz_demo", options)
  {
    visual_tools_.reset(
        new rvt::RvizVisualTools("world", "/rviz_visual_tools", dynamic_cast<rclcpp::Node*>(this)));
    // create publisher before waiting
    visual_tools_->loadMarkerPub();
    bool has_sub = visual_tools_->waitForMarkerSub(10.0);
    if (!has_sub)
      RCLCPP_INFO(get_logger(), "/rviz_visual_tools does not have a subscriber after 10s. "
                                "Visualizations may be lost");

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
  }

  auto getRemoteControl()
  {
    return visual_tools_->getRemoteControl();
  }

  void prompt(const std::string& msg)
  {
    visual_tools_->prompt(msg);
  }

  void publishLabelHelper(const Eigen::Isometry3d& pose, const std::string& label)
  {
    Eigen::Isometry3d pose_copy = pose;
    pose_copy.translation().x() -= 0.2;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XXLARGE, false);
  }

  void testRows(double& x_location)
  {
    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    pose1.translation().x() = x_location;

    double space_between_rows = 0.2;
    double y = 0;
    double step;

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying range of colors red->green");
    step = 0.02;
    for (double i = 0; i <= 1.0; i += 0.02)
    {
      geometry_msgs::msg::Vector3 scale = visual_tools_->getScale(MEDIUM);
      std_msgs::msg::ColorRGBA color = visual_tools_->getColorScale(i);
      visual_tools_->publishSphere(visual_tools_->convertPose(pose1), color, scale, "Sphere");
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Sphere Color Range");
      }
      pose1.translation().x() += step;
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Coordinate Axis");
    pose1.translation().x() = 0;
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishAxis(pose1);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Coordinate Axis");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Arrows");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishArrow(pose1, rvt::RAND);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Arrows");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Rectangular Cuboid");
    double cuboid_max_size = 0.075;
    double cuboid_min_size = 0.01;
    pose1 = Eigen::Isometry3d::Identity();
    pose2 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    pose2.translation().y() = y;
    step = 0.1;
    for (double i = 0; i <= 1.0; i += step)
    {
      pose2 = pose1;
      pose2.translation().x() += i * cuboid_max_size + cuboid_min_size;
      pose2.translation().y() += i * cuboid_max_size + cuboid_min_size;
      pose2.translation().z() += i * cuboid_max_size + cuboid_min_size;
      visual_tools_->publishCuboid(pose1.translation(), pose2.translation(), rvt::RAND);

      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Cuboid");
      }

      pose1.translation().x() += step;
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Lines");
    double line_max_size = 0.075;
    double line_min_size = 0.01;
    pose1 = Eigen::Isometry3d::Identity();
    pose2 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    pose2.translation().y() = y;
    step = 0.1;
    for (double i = 0; i <= 1.0; i += step)
    {
      pose2 = pose1;
      pose2.translation().x() += i * line_max_size + line_min_size;
      pose2.translation().y() += i * line_max_size + line_min_size;
      pose2.translation().z() += i * line_max_size + line_min_size;
      visual_tools_->publishLine(pose1.translation(), pose2.translation(), rvt::RAND);

      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Line");
      }

      pose1.translation().x() += step;
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Cylinder");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishCylinder(pose1, rvt::RAND);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Cylinder");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Axis Cone");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    double angle_step = 0.1;
    double angle = 1;

    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishCone(pose1, M_PI / angle, rvt::RAND, 0.05);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Cone");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
      angle += angle_step;
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Wireframe Cuboid");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;
    // TODO(davetcoleman): use generateRandomCuboid()
    Eigen::Vector3d min_point, max_point;
    min_point << -0.05, -0.05, -0.05;
    max_point << 0.05, 0.05, 0.05;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishWireframeCuboid(pose1, min_point, max_point, rvt::RAND);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Wireframe Cuboid");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Sized Wireframe Cuboid");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;
    double depth = 0.05, width = 0.05, height = 0.05;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishWireframeCuboid(pose1, depth, width, height, rvt::RAND);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Wireframe Cuboid");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Planes");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.2;
    double max_plane_size = 0.075;
    double min_plane_size = 0.01;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishXYPlane(pose1, rvt::RED, i * max_plane_size + min_plane_size);
      visual_tools_->publishXZPlane(pose1, rvt::GREEN, i * max_plane_size + min_plane_size);
      visual_tools_->publishYZPlane(pose1, rvt::BLUE, i * max_plane_size + min_plane_size);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Planes");
      }

      pose1.translation().x() += step;
    }
    visual_tools_->trigger();

    /* TODO(mlautman): port graph_msgs
    // // --------------------------------------------------------------------
    // RCLCPP_INFO(get_logger(), "Displaying Graph");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;
    graph_msgs::msg::GeometryGraph graph;
    for (double i = 0; i <= 1.0; i += step)
    {
      graph.nodes.push_back(visual_tools_->convertPose(pose1).position);
      graph_msgs::msg::Edges edges;
      if (i > 0)
      {
        edges.node_ids.push_back(0);
      }
      graph.edges.push_back(edges);

      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Graph");
      }

      pose1.translation().x() += step;
      pose1.translation().z() += visual_tools_->dRand(-0.1, 0.1);
    }
    visual_tools_->publishGraph(graph, rvt::ORANGE, 0.005);
    visual_tools_->trigger();
    */

    // --------------------------------------------------------------------
    // TODO(davetcoleman): publishMesh

    // --------------------------------------------------------------------
    // TODO(davetcoleman): publishPolygon

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Labeled Coordinate Axis");
    pose1.translation().x() = 0;
    y += space_between_rows;
    pose1.translation().y() = y;
    pose1.translation().z() = 0;
    step = 0.2;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishAxisLabeled(pose1, "label of axis");
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Labeled Axis");
      }

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying Multi-Color Path");
    pose1 = Eigen::Isometry3d::Identity();
    pose2 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;

    EigenSTL::vector_Vector3d path;
    std::vector<rviz_visual_tools::Colors> colors;
    unsigned index(0);
    for (double i = 0; i < 1.0; i += step)
    {
      pose1.translation().y() = y;
      if (++index % 2 == 0)
      {
        pose1.translation().y() += step / 2.0;
        colors.push_back(rviz_visual_tools::WHITE);
      }
      else
      {
        pose1.translation().y() -= step / 2.0;
        colors.push_back(rviz_visual_tools::BLUE);
      }
      path.emplace_back(pose1.translation());
      pose1.translation().x() += step;

      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Path");
      }
    }
    visual_tools_->publishPath(path, colors);
    visual_tools_->trigger();

    // --------------------------------------------------------------------
    RCLCPP_INFO(get_logger(), "Displaying ABCD Plane");
    double x_width = 0.15;
    double y_width = 0.05;

    Eigen::Vector3d n;
    double a, b, c = 0, d;
    y += space_between_rows;
    double x_plane = 0, y_plane = y;

    pose1 = Eigen::Isometry3d::Identity();
    pose1.translation().x() = x_plane;
    pose1.translation().y() = y_plane;
    publishLabelHelper(pose1, "ABCD Plane");

    for (std::size_t i = 0; i < 10; ++i)
    {
      x_plane = i * step;
      a = x_plane;
      b = y_plane;
      // D takes this value to satisfy Ax+By+D=0
      d = -(x_plane * x_plane + y_plane * y_plane);
      visual_tools_->publishABCDPlane(a, b, c, d, rvt::MAGENTA, x_width, y_width);
      x_location += step;
    }

    // Set x location for next visualization function
    x_location += 1.25;

    visual_tools_->trigger();
  }

  /** \brief Compare sizes of markers using all MEDIUM-scale markers */
  void testSize(double& x_location, Scales scale)
  {
    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    // Reusable vector of 2 colors
    std::vector<Colors> colors;
    colors.push_back(RED);
    colors.push_back(GREEN);

    // Reusable points vector
    EigenSTL::vector_Vector3d points1;
    EigenSTL::vector_Vector3d points2;

    double step = 0.25;  // space between each row

    // Show test label
    pose1.translation().x() = x_location - 0.1;
    visual_tools_->publishText(
        pose1, "Testing consistency of " + visual_tools_->scaleToString(scale) + " marker scale",
        WHITE, XLARGE, false);

    pose1.translation().x() = x_location;

    // TODO(dave): publishCone() - no scale version available
    // TODO(dave): publishXYPlane() - no scale version available
    // TODO(dave): publishXZPlane() - no scale version available
    // TODO(dave): publishYZPlane() - no scale version available

    // Sphere
    visual_tools_->publishSphere(pose1, BLUE, scale);
    pose1.translation().y() += step;

    // Spheres
    points1.clear();
    points1.emplace_back(pose1.translation());
    pose1.translation().x() += step;
    points1.emplace_back(pose1.translation());
    visual_tools_->publishSpheres(points1, BLUE, scale);
    pose1.translation().x() = x_location;  // reset
    pose1.translation().y() += step;

    // Spheres with colors
    points1.clear();
    points1.emplace_back(pose1.translation());
    pose1.translation().x() += step;
    points1.emplace_back(pose1.translation());
    visual_tools_->publishSpheres(points1, colors, scale);
    pose1.translation().x() = x_location;  // reset
    pose1.translation().y() += step;

    // YArrow
    visual_tools_->publishYArrow(pose1, BLUE, scale);
    pose1.translation().y() += step;

    // ZArrow
    visual_tools_->publishZArrow(pose1, GREEN, scale);
    pose1.translation().y() += step;

    // XArrow
    visual_tools_->publishXArrow(pose1, RED, scale);
    pose1.translation().y() += step;

    // Arrow (x arrow)
    visual_tools_->publishArrow(pose1, RED, scale);
    pose1.translation().y() += step;

    // Line
    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    visual_tools_->publishLine(pose1, pose2, PURPLE, scale);
    pose1.translation().y() += step;

    // Lines
    points1.clear();
    points2.clear();
    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    points1.emplace_back(pose1.translation());
    points2.emplace_back(pose2.translation());
    pose1.translation().x() += step / 2.0;

    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    // points1.push_back(pose1.translation());
    // points2.push_back(pose2.translation());
    colors.clear();  // temp
    colors.push_back(ORANGE);
    visual_tools_->publishLines(points1, points2, colors, scale);
    pose1.translation().x() = x_location;  // reset
    pose1.translation().y() += step;

    // TODO(dave): publishPath
    // TODO(dave): publishPolygon
    // TODO(dave): publishWireframeCuboid
    // TODO(dave): publishWireframeRectangle

    // Axis Labeled
    visual_tools_->publishAxisLabeled(pose1, "Axis", scale);
    pose1.translation().y() += step;

    // Axis
    visual_tools_->publishAxis(pose1, scale);
    pose1.translation().y() += step;

    // TODO(dave): publishAxis

    // Cylinder
    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    visual_tools_->publishCylinder(pose1.translation(), pose2.translation(), BLUE, scale);
    pose1.translation().y() += step;

    // TODO(dave): publishMesh

    // TODO(dave): publishGraph

    // Text
    visual_tools_->publishText(pose1, "Text", WHITE, scale, false);
    pose1.translation().y() += step;

    // Display test
    visual_tools_->trigger();

    // Set x location for next visualization function
    x_location += 0.5;
  }

  /** \brief Compare every size range */
  void testSizes(double& x_location)
  {
    RCLCPP_INFO(get_logger(), "Testing sizes of marker scale");

    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    // Show test label
    pose1.translation().x() = x_location - 0.1;
    visual_tools_->publishText(pose1, "Testing sizes of marker scale", WHITE, XLARGE, false);

    pose1.translation().x() = x_location;
    pose2.translation().x() = x_location;

    // Sphere
    for (Scales scale = XXXXSMALL; scale <= XXXXLARGE; /*inline*/)
    {
      pose1.translation().y() += visual_tools_->getScale(scale).x + 0.1;

      // Text location
      pose2.translation().y() = pose1.translation().y();
      pose2.translation().x() = x_location + visual_tools_->getScale(scale).x * 1.3;
      if (scale == MEDIUM)
      {
        visual_tools_->publishSphere(pose1, GREEN, scale);
      }
      else
      {
        visual_tools_->publishSphere(pose1, GREY, scale);
      }
      visual_tools_->publishText(pose2, "Size " + visual_tools_->scaleToString(scale), WHITE, scale,
                                 false);

      scale = static_cast<Scales>(static_cast<int>(scale) + 1);
    }

    // Display test
    visual_tools_->trigger();

    // Set x location for next visualization function
    x_location += 0.5;
  }
};  // end class

}  // namespace rviz_visual_tools

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // NOLINT

  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rviz_demo"), "Visual Tools Demo");

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Create demo node
  rclcpp::NodeOptions options = rclcpp::NodeOptions().arguments(args);
  auto demo = std::make_shared<rviz_visual_tools::RvizVisualToolsDemo>(options);

  // Initialize RemoteControl
  auto remote_control = demo->getRemoteControl();

  // Allow the action server to recieve and send ros messages
  executor->add_node(demo);
  std::thread([executor]() { executor->spin(); }).detach();

  demo->prompt("Click 'Next' using the RvizVisualToolsGui dashboard!");

  double x_location = 0;
  demo->testRows(x_location);
  demo->testSize(x_location, rviz_visual_tools::MEDIUM);
  demo->testSize(x_location, rviz_visual_tools::LARGE);
  demo->testSizes(x_location);

  executor->remove_node(demo);

  RCLCPP_INFO(rclcpp::get_logger("rviz_demo"), "Shutting down.");

  return 0;
}
