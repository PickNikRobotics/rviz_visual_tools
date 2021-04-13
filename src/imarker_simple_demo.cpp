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
   Desc:   Demonstrate how to use the imarker simple interface
*/

#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <rviz_visual_tools/imarker_simple.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace rviz_visual_tools
{
class IMarkerSimpleDemo : public rclcpp::Node
{
public:
  /** \brief Constructor */
  IMarkerSimpleDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("imarker_simple_demo", options)
  {
    visual_tools_.reset(
        new RvizVisualTools("world", "/rviz_visual_tools", dynamic_cast<rclcpp::Node*>(this)));
    visual_tools_->loadMarkerPub();

    // Create a random initial pose
    geometry_msgs::msg::Pose init_pose;
    visual_tools_->generateRandomPose(init_pose);

    // Create a 6DOF interactive marker
    static const double SCALE = 0.2;
    imarker_simple_.reset(
        new IMarkerSimple(dynamic_cast<rclcpp::Node*>(this), "imarker", SCALE, init_pose));

    // Add callback to this class
    imarker_simple_->setIMarkerCallback(
        std::bind(&IMarkerSimpleDemo::processIMarkerPose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "IMarkerSimpleDemo Ready.");
  }

  void processIMarkerPose(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    // Show some spheres for fun
    visual_tools_->publishSphere(feedback->pose, visual_tools_->getRandColor());
    visual_tools_->trigger();
  }

private:
  // --------------------------------------------------------
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  rviz_visual_tools::IMarkerSimplePtr imarker_simple_;

};  // end class

}  // namespace rviz_visual_tools

int main(int argc, char** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // NOLINT

  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.arguments(args);

  // Create demo node
  auto demo = std::make_shared<rviz_visual_tools::IMarkerSimpleDemo>(options);
  // Allow the action server to recieve and send ros messages
  exec.add_node(demo);
  while (rclcpp::ok())
  {
    exec.spin_some();
    rclcpp::sleep_for(1ms);
  }

  exec.remove_node(demo);

  // Shutdown
  RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down.");

  return 0;
}
