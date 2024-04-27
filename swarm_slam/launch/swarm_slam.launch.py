# note this does not work. use swarm_toolbox.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    
    #print(os.path.join(swarm_slam_directory, 'config'))
    # parameters
    swarm_slam_directory = get_package_share_directory('swarm_slam')
 
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  swarm_slam_directory, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_slam_config.lua')

    ld = LaunchDescription()

    # Define namespaces and topics
    namespaces = ['robot_0']  # Add more namespaces as needed

    # Add the log level argument to the launch description
    log = LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value='info',
            description="Logging level",
        )
    ])
    ld.add_action(log)
    

    # Launch SLAM for each robot
    for ns in namespaces:
        node_name = 'cartographer_node_' + ns
        
        ld.add_action(DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load')
        )

        ld.add_action(DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer')
        )

        ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value= 'true',
            description='Use simulation (Gazebo) clock if true'),
        )
            
        slam_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name=node_name,
            namespace=ns,
            parameters=[],#[{'use_sim_time' : true}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings = [
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("/imu", "imu"),
                ("/map", "map")
            ]
        )        
    
        ld.add_action(slam_node)

    return ld
