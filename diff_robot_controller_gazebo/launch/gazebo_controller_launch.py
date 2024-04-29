#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    # Declare the argument for using the slider
    sl.declare_arg('use_slider', True, description = 'Use slider for publishing linear and angular velocity')

	# Start the diff robot description launch file
    sl.include('diff_robot_description','diff_base_launch.py', launch_arguments={'use_joint_pub': False})

    # Start the slider publisher node
    with sl.group(if_condition=sl.arg('use_slider')):
        sl.node('slider_publisher', 'slider_publisher', name ='Twist', arguments = [sl.find('diff_robot_controller', 'Twist.yaml')])

    # Start the gazebo launch file
    sl.include('diff_robot_gazebo','gazebo_launch.py')

    return sl.launch_description()