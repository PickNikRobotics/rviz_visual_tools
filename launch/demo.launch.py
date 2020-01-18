# Copyright 2020 PickNik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

"""
<launch>

  <arg name="debug" default="false" />
  <arg unless="$(arg debug)" name="launch_prefix" value="" />
  <arg     if="$(arg debug)" name="launch_prefix" value="gdb --ex run --args" />

  <!-- Start demo -->
  <node name="rviz_visual_tools_demo" launch-prefix="$(arg launch_prefix)" pkg="rviz_visual_tools"
       type="demo" output="screen">
  </node>

</launch>
"""

def generate_launch_description():
    return LaunchDescription([
        Node(package='rviz_visual_tools', node_executable='rviz_visual_tools_demo', output='screen')
    ])
