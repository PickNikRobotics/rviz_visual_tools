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
 *   * Neither the name of the PickNik Consulting nor the names of its
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

/* Author: Dave Coleman <dave@picknik.ai>
   Desc:   Tool for creating break points and user verification points through
           manipulation pipelines or other live robotitc tool.
           Think GDB for robots, or like, a state machine.
*/

#pragma once

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace rviz_visual_tools
{
typedef std::function<void(bool)> DisplayWaitingState;

class RemoteControl
{
public:
  /**
   * \brief Constructor
   */
  explicit RemoteControl(const ros::NodeHandle& nh);

  /**
   * \brief Callback from ROS topic
   */
  void rvizDashboardCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Step to next step
   * \return true on success
   */
  bool setReadyForNextStep();

  /**
   * \brief Enable autonomous mode
   */
  void setAutonomous(bool autonomous = true);
  void setFullAutonomous(bool autonomous = true);

  /**
   * \brief Get the autonomous mode
   * \return true if is in autonomous mode
   */
  bool getAutonomous();
  bool getFullAutonomous();

  /**
   * \brief Stop something in pipeline
   */
  void setStop(bool stop = true);

  /**
   * \brief See if we are in stop mode
   */
  bool getStop();

  /**
   * \brief Wait until user presses a button
   * \return true on success
   */
  bool waitForNextStep(const std::string& caption = "go to next step");
  bool waitForNextFullStep(const std::string& caption = "go to next full step");

  /** \brief Return true if debug interface is waiting for user input */
  bool isWaiting()
  {
    return is_waiting_;
  }

  void setDisplayWaitingState(DisplayWaitingState displayWaitingState)
  {
    displayWaitingState_ = displayWaitingState;
  }

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Short name for this class
  std::string name_ = "remote_control";

  // Input
  ros::Subscriber rviz_dashboard_sub_;

  // Debug interface
  bool is_waiting_ = false;
  bool next_step_ready_ = false;
  bool autonomous_ = false;
  bool full_autonomous_ = false;
  bool stop_ = false;

  // Callback to visualize waiting state
  DisplayWaitingState displayWaitingState_;
};  // end class

typedef std::shared_ptr<RemoteControl> RemoteControlPtr;
typedef std::shared_ptr<const RemoteControl> RemoteControlConstPtr;

}  // namespace rviz_visual_tools
