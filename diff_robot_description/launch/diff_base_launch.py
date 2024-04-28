#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    # Declare arguments
    sl.declare_arg('use_joint_pub', True, description = 'Use joint state publisher')

	# Start the rviz node
    sl.rviz(sl.find('diff_robot_description', 'diff.rviz'))
    
    # We get the robot type to generate description
    sl.robot_state_publisher('diff_robot_description', 'diff_robot.xacro')
    
    with sl.group(if_condition=sl.arg('use_joint_pub')):
        sl.joint_state_publisher(use_gui=True)

    return sl.launch_description()