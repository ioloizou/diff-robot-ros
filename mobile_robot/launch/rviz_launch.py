#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

	# Start the rviz node
    sl.rviz()
    
    # We get the robot type to generate description
    sl.robot_state_publisher('mobile_robot', 'diff_robot.xacro')

    # Start the slider publisher node
    sl.node('slider_publisher', 'slider_publisher', name ='Twist', arguments = [sl.find('mobile_robot', 'Twist.yaml')])

    # Start differentail control node
    sl.node('mobile_robot','diff_control', output='screen')

    return sl.launch_description()
