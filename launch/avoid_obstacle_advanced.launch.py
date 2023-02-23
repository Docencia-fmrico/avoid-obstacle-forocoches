# Copyright 2023 Intelligent Robotics Lab
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


def generate_launch_description():

    # TODO: Add file
    # pkg_dir = get_package_share_directory('avoid_obstacle_forocoches')
    # param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    avoid_obstacle_advanced_cmd = Node(
                                      package='avoid_obstacle_forocoches',
                                      executable='avoid_obstacle_advanced',
                                      output='screen',
                                      parameters=[{
                                        'use_sim_time': False
                                      }],
                                      arguments=['--ros-args', '--log-level', 'info'],
                                      remappings=[
                                        ('input_scan', 'scan'),
                                        ('output_vel', 'cmd_vel')
                                      ])

    ld = LaunchDescription()
    ld.add_action(avoid_obstacle_advanced_cmd)

    return ld
