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

    # initilse args
    number_robots=1

    for arg in sys.argv:
        if arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        


    ld = LaunchDescription()

    for i in range(number_robots):

        nav2_dir = get_package_share_directory('swarm_slam')
        #nav2_params = '~/Thesis/SwarmExploration_ws/src/Thesis_ws/swarm_slam/config/nav2_params.yaml'
        nav2_params = os.path.expanduser('~/Thesis/SwarmExploration_ws/src/Thesis_ws/swarm_slam/config/nav2_params2.yaml')
        nav2_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'navigation.launch.py')),
            launch_arguments={
                'namespace': ['robot_', str(i)],
                'use_namespace': 'true',
                'use_sim_time': 'true',
                'params_file': nav2_params
                }.items()
        )
        ld.add_action(nav2_node)




    return ld
