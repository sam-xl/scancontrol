import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='micro_epsilon_scancontrol_driver',
            executable='driver_node',
            name='scancontrol_driver',
            output='screen',
            on_exit=launch.actions.Shutdown()
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {
                    'use_gui': 'false'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            on_exit=launch.actions.Shutdown()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'micro_epsilon_scancontrol_description'), 'launch/load_scancontrol_26x0_29x0_25.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
