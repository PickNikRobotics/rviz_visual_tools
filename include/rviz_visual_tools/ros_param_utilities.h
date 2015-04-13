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

#ifndef RVIZ_VISUAL_TOOLS__ROS_PARAM_UTILITIES
#define RVIZ_VISUAL_TOOLS__ROS_PARAM_UTILITIES

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Geometry>

namespace rviz_visual_tools
{

// -------------------------------------------------------------------------------------------------
// Helper Functions
// -------------------------------------------------------------------------------------------------

/**
 * \brief Get a paremeter from the ROS param server. Note that does not provide for default values
 * \param parent_name - the name of the class that is calling this function, used for filtering out logging output by namespacing it
 * \param nh - a ROS node handle
 * \param param_name - name of parameter to get
 * \param value - resulting loaded values, or no change if error (function returns false)
 * \return true on success
 */
bool getBoolParameter(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name, bool &value);

bool getDoubleParameter(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name,
                        double &value);

bool getDoubleParameters(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name,
                         std::vector<double> &values);

bool getIntParameter(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name, int &value);

bool getIntParameter(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name, unsigned int &value);

bool getStringParameter(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name,
                        std::string &value);

bool getStringParameters(const std::string& parent_name, const ros::NodeHandle &nh, const std::string &param_name,
                         std::vector<std::string> &values);

/**
 * \brief Output a string of values from an array for debugging
 * \param array of values
 * \return string of numbers separated by commas
 */
std::string getDebugArrayString(std::vector<double> values);

std::string getDebugArrayString(std::vector<std::string> values);

/**
 * \brief Convert from 6 doubles of [x,y,z] [r,p,y] to a transform
 * \return true on success
 */
bool convertDoublesToEigen(const std::string& parent_name, std::vector<double> values, Eigen::Affine3d& transform);

} // end namespace

#endif
