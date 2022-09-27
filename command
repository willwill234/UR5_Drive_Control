roslaunch ur_gazebo ur5_bringup.launch limited:=true
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true

source /opt/ros/noetic/setup.bash

==========Real robot connection code============
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.100 \
kinematics_config:=$(rospack find ur_calibration)/etc/ur5_calibration.yaml

roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
