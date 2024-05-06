import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # initilse args
    number_robots=1

    for arg in sys.argv:
        if arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        


    
    ld = LaunchDescription()

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [
        ("/tf", "tf"), 
        ("/tf_static", "tf_static"),
        ('/global_costmap/get_costmap', 'global_costmap/get_costmap'),
        ('/odom', 'odom'),
        ('/map', 'map')
        ]

    for i in range(number_robots):
        explore_node = Node(
            package="nav2_wfd",
            name="explore_node",
            namespace=['robot_', str(i)],
            executable="explore",
            #parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            remappings=remappings,
            # arguments=['--ros-args', '--log-level', 'DEBUG' ]
        )

        ld.add_action(explore_node)

    return ld
