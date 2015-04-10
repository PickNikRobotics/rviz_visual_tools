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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helpers for loading parameters from the parameter server
*/

#include <rviz_visual_tools/ros_param_utilities.h>

namespace rviz_visual_tools
{

bool getBoolParameter(const std::string& parent_name, ros::NodeHandle &nh, const std::string &param_name, bool &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Missing parameter '" << param_name << "'. Searching in namespace: " 
                           << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name, "Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getDoubleParameter(const std::string& parent_name, ros::NodeHandle &nh, const std::string &param_name, 
                        double &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name,"Missing parameter '" << param_name << "'. Searching in namespace: " 
                           << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name,"Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getDoubleParameters(const std::string& parent_name, ros::NodeHandle &nh, const std::string &param_name, 
                         std::vector<double> &values)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name,"Missing parameter '" << param_name << "'. Searching in namespace: " 
                           << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, values);
  ROS_DEBUG_STREAM_NAMED(parent_name,"Loaded parameter '" << param_name << "' with values [" << getDebugArrayString(values) << "]");

  return true;
}

bool getIntParameter(const std::string& parent_name, ros::NodeHandle &nh, const std::string &param_name, int &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name,"Missing parameter '" << param_name << "'. Searching in namespace: " 
                           << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name,"Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getIntParameter(const std::string& parent_name, ros::NodeHandle &nh, const std::string &param_name, unsigned int &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name,"Missing parameter '" << param_name << "'. Searching in namespace: " 
                           << nh.getNamespace());
    return false;
  }
  int nonsigned_value;
  nh.getParam(param_name, nonsigned_value);
  value = nonsigned_value;
  ROS_DEBUG_STREAM_NAMED(parent_name,"Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getStringParameter(const std::string& parent_name, ros::NodeHandle &nh, const std::string &param_name, 
                        std::string &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED(parent_name,"Missing parameter '" << param_name << "'. Searching in namespace: " << 
                           nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED(parent_name,"Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

std::string getDebugArrayString(std::vector<double> values)
{
  std::stringstream debug_values;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    debug_values << values[i] << ",";
  }
  return debug_values.str();
}

bool convertDoublesToEigen(const std::string& parent_name, std::vector<double> values, Eigen::Affine3d& transform)
{
  if (values.size() != 6)
  {
    ROS_ERROR_STREAM_NAMED(parent_name, "Invalid number of doubles provided for transform, size=" << values.size());
    return false;
  }
  Eigen::AngleAxisd roll_angle (values[3], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yaw_angle  (values[5], Eigen::Vector3d::UnitY());
  Eigen::Quaternion<double> quat = roll_angle * yaw_angle * pitch_angle;

  transform = Eigen::Translation3d(values[0], values[1], values[2]) * quat;
  return true;
}

} // end namespace















