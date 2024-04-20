#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

	# Start the rviz node
    sl.rviz()
    
    # We get the robot type to generate description
    sl.robot_state_publisher('mobile_robot', 'diff_robot.xacro')

    # Start the joint state publisher node
    sl.joint_state_publisher(use_gui=True)

    return sl.launch_description()
