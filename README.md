# Thesis

Swarm Exploration of an Unknown Environment

Abstract
The aim of the experiment is to design and implement a swarm robotic system that explores an unknown environment. To limit the scope, the aim is broken down into several goals, simulate an environment to evaluate exploration, design a decentralised swarm system, utilise mapping for an informed search and to investigate frontier goal selection methods. This was implemented using ROS2, allowing for the use of 3rd party packages to assist in running the simulation, SLAM and navigation. The system designed was proven to be capable of swarm exploration with further upgrades to certain nodes of the system. The object avoidance from the navigation package used was ineffective at preventing collisions. This led to difficulty in running and evaluating frontier goal selection due to the global merged map having multiple malfunctioning input local maps from fallen over robots. It was found that the dispersion parameter is by far the most important for increasing swarm exploration rate, with distance of the robot from the frontier goal also aiding in reducing travel distance and exploration inefficiency.  ****

Notes

ROS2Swarm
    bash start_simulation.sh
    bash start_command.sh
Thesis_ws
    bash slam.sh
    
rqt_graph



Turtlebot3
    Cartographer
        Algorithm Walkthrouh
            - https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html

bash x2

ros2 launch merge_map map_view_launch.py number_robots:=3
ros2 launch swarm_slam nav2.launch.py number_robots:=3
ros2 launch swarm_slam explore.launch.py number_robots:=3

![Alt text](3_robot_mapping.png)
