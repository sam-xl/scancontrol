# Copyright 2023 SAM XL (Eugenio Bernardi)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="micro_epsilon_scancontrol_description",
            description="Description package with URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    # Declare argument for the URDF file selection
    declared_arguments.append(
        DeclareLaunchArgument(
            "scancontrol_type",
            default_value="scancontrol_26x0_29x0_25.xacro",
            description="scanCONTROL URDF type file to visualize",
            choices=[
                "scancontrol_26x0_29x0_25.xacro",
                "scancontrol_27x0_100.xacro",
                "scancontrol_30xx_25.xacro",
            ],
        )
    )

    # General arguments
    description_package = LaunchConfiguration("description_package")
    # Initialize Arguments
    scancontrol_type = LaunchConfiguration("scancontrol_type")

    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory("micro_epsilon_scancontrol_description"), "urdf"
    )
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", scancontrol_type]
            )
        ]
    )

    # Node to publish the URDF to the robot_description parameter
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        # output='both',
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Node to start RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            get_package_share_directory("micro_epsilon_scancontrol_description")
            + "/config/rviz.rviz",
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            rviz_node,
        ]
    )
