import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():

    for arg in sys.argv:
        if arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    #default_params_file = os.path.join(get_package_share_directory("slam_toolbox"),'config', 'mapper_params_online_async.yaml')
    default_params_file = os.path.join(get_package_share_directory("swarm_slam"),'config', 'slam_toolbox_async_params.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # If the provided param file doesn't have slam_toolbox params, we must pass the
    # default_params_file instead. This could happen due to automatic propagation of
    # LaunchArguments. See:
    # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(log_param_change)

    for i in range(number_robots):

        start_async_slam_toolbox_node = Node(
            parameters=[
            actual_params_file,
            {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            namespace=['robot_', str(i)],
            remappings=[
                ('/tf', 'tf'), 
                ('/tf_static', 'tf_static'),
                ('/scan', 'scan'),
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata')
            ]
            )

        ld.add_action(start_async_slam_toolbox_node)

    return ld