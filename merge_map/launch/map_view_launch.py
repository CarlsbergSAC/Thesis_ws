import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():

    number_robots=1

    for arg in sys.argv:
        if arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        
    ld = LaunchDescription()

    occupancy_viewer_node = Node(
        package='merge_map',
        executable='occupancy_viewer',
        output='screen',
        parameters=[{'use_sim_time': True, 'number_robots':str(number_robots)}],
        #namespace=['robot_', str(i)],
        remappings=[
            #("/map1", "map"),
            #("/merge_map", "merge_map"),
        ],
    )

    ld.add_action(occupancy_viewer_node)

    return ld