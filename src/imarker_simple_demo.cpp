/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
   Desc:   Demonstrate how to use the imarker simple interface
*/

#ifndef RVIZ_VISUAL_TOOLS_IMARKER_SIMPLE_DEMO_H
#define RVIZ_VISUAL_TOOLS_IMARKER_SIMPLE_DEMO_H

// ROS
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/imarker_simple.h>

namespace rviz_visual_tools
{
class IMarkerSimpleDemo
{
public:
  /** \brief Constructor */
  IMarkerSimpleDemo() : nh_("~")
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();

    // Create a random initial pose
    geometry_msgs::Pose init_pose;
    visual_tools_->generateRandomPose(init_pose);

    // Create a 6DOF interactive marker
    static const double SCALE = 0.2;
    imarker_simple_.reset(new rviz_visual_tools::IMarkerSimple("imarker", SCALE, init_pose));

    // Add callback to this class
    imarker_simple_->setIMarkerCallback(std::bind(&IMarkerSimpleDemo::processIMarkerPose, this, std::placeholders::_1));

    ROS_INFO_STREAM_NAMED(name_, "IMarkerSimpleDemo Ready.");
  }

  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    // Show some spheres for fun
    visual_tools_->publishSphere(feedback->pose, visual_tools_->getRandColor());
    visual_tools_->trigger();
  }

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "imarker_simple_demo";

  // A shared node handle
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  rviz_visual_tools::IMarkerSimplePtr imarker_simple_;

};  // end class

// Create std pointers for this class
typedef std::shared_ptr<IMarkerSimpleDemo> IMarkerSimpleDemoPtr;
typedef std::shared_ptr<const IMarkerSimpleDemo> IMarkerSimpleDemoConstPtr;

}  // namespace rviz_visual_tools
#endif  // RVIZ_VISUAL_TOOLS_IMARKER_SIMPLE_DEMO_H

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "imarker_simple_demo");
  ROS_INFO_STREAM_NAMED("main", "Starting IMarkerSimpleDemo...");

  // Seed random number generator
  // srand (time(NULL));

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  rviz_visual_tools::IMarkerSimpleDemo server;

  ros::waitForShutdown();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
