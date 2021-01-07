# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
import launch


def generate_launch_description():
    worldforge_world = get_package_share_directory(
        'aws_robomaker_worldforge_worlds')
    worldforge_world_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(worldforge_world, 'launch', 'launch_world.py'))
    )
    turtlebot3_description_reduced_mesh = get_package_share_directory(
        'turtlebot3_description_reduced_mesh')
    turtlebot3_description_reduced_mesh_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_description_reduced_mesh,
                'launch',
                'spawn_turtlebot.launch.py')))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        turtlebot3_description_reduced_mesh_launch,
        worldforge_world_launch
    ])
