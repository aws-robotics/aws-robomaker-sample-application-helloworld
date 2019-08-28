# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""Launch turtlebot3_description_reduced_mesh and a rotate node."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import tempfile
from launch_ros import get_default_launch_description
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
import xacro
import lifecycle_msgs.msg
import subprocess

def generate_launch_description():
    """Main."""
    use_sim_time_false = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time',
        )

    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    turtlebot3_location = get_package_share_directory('turtlebot3_description_reduced_mesh') \
        + '/urdf/turtlebot3_' + turtlebot3_model + '.urdf.xacro'
    with subprocess.Popen(['ros2', 'run', 'xacro', 'xacro', turtlebot3_location], stdout=subprocess.PIPE) as proc:
        urdf = proc.stdout.read()
    if urdf is 0:
        raise 'xacro issue {} return_code={}'.format(turtlebot3_location, urdf)
    msg = '{name: "robot", xml: "' + ''.join(urdf.decode("utf-8").splitlines()).replace('    ', '')\
        .replace(r'"', r'\"') + '"}'
    spawn_robot = launch.actions.ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/srv/SpawnEntity',
             msg],
        output='screen'
    )

    ld = launch.LaunchDescription(
        [use_sim_time_false,
         spawn_robot
         ])
    return ld 


if __name__ == '__main__':
    generate_launch_description()
