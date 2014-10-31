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

/* Author: Dave Coleman
   Desc:   Demo implementation of rviz_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /rviz_visual_tools
*/

// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rviz_visual_tools
{

class RvizVisualToolsTest
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

public:

  /**
   * \brief Constructor
   */
  RvizVisualToolsTest()
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base","/rviz_visual_tools"));

    // Allow time to publish messages
    ros::Duration(1.0).sleep();

    while (ros::ok())
    {
      visual_tools_->publishTest();
    }
  }

  /**
   * \brief Destructor
   */
  ~RvizVisualToolsTest()
  {
  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_test");
  ROS_INFO_STREAM("Visual Tools Test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rviz_visual_tools::RvizVisualToolsTest tester;

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
