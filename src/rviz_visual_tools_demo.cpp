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
   Desc:   Demo implementation of rviz_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /rviz_visual_tools
*/

// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// C++
#include <string>

namespace rvt = rviz_visual_tools;

namespace rviz_visual_tools
{
class RvizVisualToolsDemo
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  rvt::RvizVisualToolsPtr visual_tools_;

  std::string name_;

public:
  /**
   * \brief Constructor
   */
  RvizVisualToolsDemo() : name_("rviz_demo")
  {
    visual_tools_.reset(new rvt::RvizVisualTools("base", "/rviz_visual_tools"));

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
  }

  void publishLabelHelper(const Eigen::Affine3d& pose, const std::string& label)
  {
    Eigen::Affine3d pose_copy = pose;
    pose_copy.translation().x() -= 0.2;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::REGULAR, false);
  }

  void runTests()
  {
    // Create pose
    Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();

    double space_between_rows = 0.2;
    double y = 0;
    double step;

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying range of colors red->green");
    step = 0.02;
    for (double i = 0; i <= 1.0; i += 0.02)
    {
      geometry_msgs::Vector3 scale = visual_tools_->getScale(XLARGE, false, 0.05);
      std_msgs::ColorRGBA color = visual_tools_->getColorScale(i);
      visual_tools_->publishSphere(visual_tools_->convertPose(pose1), color, scale, "Sphere");
      if (!i)
        publishLabelHelper(pose1, "Sphere Color Range");
      pose1.translation().x() += step;
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Coordinate Axis");
    pose1.translation().x() = 0;
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishAxis(pose1);
      if (!i)
        publishLabelHelper(pose1, "Coordinate Axis");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Arrows");
    pose1 = Eigen::Affine3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishArrow(pose1, rvt::RAND);
      if (!i)
        publishLabelHelper(pose1, "Arrows");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Rectangular Cuboid");
    double cuboid_max_size = 0.075;
    double cuboid_min_size = 0.01;
    pose1 = Eigen::Affine3d::Identity();
    pose2 = Eigen::Affine3d::Identity();
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

      if (!i)
        publishLabelHelper(pose1, "Cuboid");

      pose1.translation().x() += step;
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Lines");
    double line_max_size = 0.075;
    double line_min_size = 0.01;
    pose1 = Eigen::Affine3d::Identity();
    pose2 = Eigen::Affine3d::Identity();
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

      if (!i)
        publishLabelHelper(pose1, "Line");

      pose1.translation().x() += step;
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Cylinder");
    pose1 = Eigen::Affine3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishCylinder(pose1, rvt::RAND);
      if (!i)
        publishLabelHelper(pose1, "Cylinder");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Axis Cone");
    pose1 = Eigen::Affine3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.025;
    double angle_step = 0.1;
    double angle = 1;
    
    for (double i = 0; i <= 1.0; i += step)
    {
	
      visual_tools_->publishCone(pose1, M_PI / angle, rvt::RAND, 0.05);
      if (!i)
        publishLabelHelper(pose1, "Cone");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step*2*M_PI, Eigen::Vector3d::UnitZ());
      angle +=angle_step;
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Wireframe Cuboid");
    pose1 = Eigen::Affine3d::Identity();
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
      if (!i)
        publishLabelHelper(pose1, "Wireframe Cuboid");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Sized Wireframe Cuboid");
    pose1 = Eigen::Affine3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;
    double depth = 0.05, width = 0.05, height = 0.05;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishWireframeCuboid(pose1, depth, width, height, rvt::RAND);
      if (!i)
        publishLabelHelper(pose1, "Wireframe Cuboid");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Planes");
    pose1 = Eigen::Affine3d::Identity();
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
      if (!i)
        publishLabelHelper(pose1, "Planes");

      pose1.translation().x() += step;
    }
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Graph");
    pose1 = Eigen::Affine3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;
    graph_msgs::GeometryGraph graph;
    for (double i = 0; i <= 1.0; i += step)
    {
      graph.nodes.push_back(visual_tools_->convertPose(pose1).position);
      graph_msgs::Edges edges;
      if (i > 0)
        edges.node_ids.push_back(0);
      graph.edges.push_back(edges);

      if (!i)
        publishLabelHelper(pose1, "Graph");

      pose1.translation().x() += step;
      pose1.translation().z() += visual_tools_->dRand(-0.1, 0.1);
    }
    visual_tools_->publishGraph(graph, rvt::ORANGE, 0.005);
    visual_tools_->triggerBatchPublish();

    // --------------------------------------------------------------------
    // TODO(davetcoleman): publishMesh

    // --------------------------------------------------------------------
    // TODO(davetcoleman): publishPolygon

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Labeled Coordinate Axis");
    pose1.translation().x() = 0;
    y += space_between_rows;
    pose1.translation().y() = y;
    pose1.translation().z() = 0;
    step = 0.2;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishAxisLabeled(pose1, "label of axis");
      if (!i)
        publishLabelHelper(pose1, "Labeled Coordinate Axis");

      pose1.translation().x() += step;
      pose1 = pose1 * Eigen::AngleAxisd(step*2*M_PI, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(step*2*M_PI, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(step*2*M_PI, Eigen::Vector3d::UnitZ());
    }
    visual_tools_->triggerBatchPublish();
  }

  /**
   * \brief Destructor
   */
  ~RvizVisualToolsDemo()
  {
  }
};  // end class

}  // namespace rviz_visual_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rviz_visual_tools::RvizVisualToolsDemo demo;
  demo.runTests();

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
