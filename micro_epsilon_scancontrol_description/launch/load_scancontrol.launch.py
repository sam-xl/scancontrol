from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Declare an argument for the URDF file
args =[ 
    DeclareLaunchArgument(
        'model',
        default_value='scancontrol_26x0_29x0_25.macro.xacro',
        description='URDF file to visualize'
        ),
]

def generate_launch_description():


    # Get the path to the URDF file
    urdf_file_path = os.path.join(get_package_share_directory('micro_epsilon_scancontrol_description'), 'urdf')
    robot_description = Command(['xacro', ' ', xacro_path, '/', 'samxl_eef_acf.xacro', ' ', 'fixed:=', LaunchConfiguration("fixed")])


    # Node to publish the URDF to the robot_description parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        #output='both',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Node to start RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', get_package_share_directory('micro_epsilon_scancontrol_description') + '/config/rviz.rviz'],
    )

    return LaunchDescription(args + [ 
        robot_state_publisher_node,
        rviz_node,
    ])