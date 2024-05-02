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

     
    

    # set default
    number_robots=1
    gazebo_world='square_boxes.world'

    for arg in sys.argv:
        if arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("gazebo_world:="):  # Name of the gazebo world
            gazebo_world = arg.split(":=")[1]        
        
    ####### launch gazebo
    launch_file_dir = os.path.join(get_package_share_directory('launch_gazebo'))

    # Add gazebo start script
    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/start_gazebo.launch.py']),
        launch_arguments={'world_name': gazebo_world}.items(),
    )
    
    
    state_node = Node(package="launch_gazebo",
                        executable="ground_truth_publisher",
                        output="screen",
                        parameters=[{
                        'use_sim_time': True,
                        }])
    

    
    ld = LaunchDescription()
    ld.add_action(gazebo_start) # does not start /map_server
    ld.add_action(state_node) # does not start /map_server


    return ld