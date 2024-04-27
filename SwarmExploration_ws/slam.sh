colcon build --symlink-install --allow-overriding swarm_slam &&
source ./install/setup.bash &&
ROS_DOMAIN_ID=42 ros2 launch swarm_slam swarm_toolbox.launch.py \
number_robots:=4


