$ ros2 launch kortex_bringup gen3.launch.py robot_ip:=172.16.0.1 robot_controller:=twist_controller launch_rviz:=false

$ ros2 launch kinova_vision kinova_vision.launch.py device:=172.16.0.1 launch_depth:=false

$ ros2 launch ur_robot_driver ur3.launch.py robot_ip:=172.16.0.3 initial_joint_controller:=forward_velocity_controller launch_rviz:=false
