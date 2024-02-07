import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='micro_epsilon_scancontrol_driver',
            executable='driver_node',
            name='scancontrol_driver',
            output='screen',
            on_exit=launch.actions.Shutdown()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
