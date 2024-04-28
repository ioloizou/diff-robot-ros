#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=False)

    sl.declare_arg('use_slider', True, description = 'Use slider for publishing linear and angular velocity')
    sl.declare_arg('gazebo', False, description = 'Open Gazebo')
    sl.declare_arg('v', 0., description = 'Set constant linear velocity')
    sl.declare_arg('w', 0., description = 'Set cosnstant angular velocity')
    sl.declare_arg('use_param_yaml', False, description = 'Use the parameters yaml instead of the argument parameters')

	# Start the diff robot description launch file
    sl.include('diff_robot_description','diff_base_launch.py', launch_arguments={'use_joint_pub': False})

    # Start the slider publisher node
    with sl.group(if_condition=sl.arg('use_slider')):
        sl.node('slider_publisher', 'slider_publisher', name ='Twist', arguments = [sl.find('diff_robot_controller', 'Twist.yaml')])

    # Start the gazebo launch file
    with sl.group(if_condition=sl.arg('gazebo')):
        sl.include('diff_robot_gazebo','gazebo_launch.py')

    # Start differential control node with argument parameters
    with sl.group(unless_condition=sl.arg('use_param_yaml')):
        sl.node('diff_robot_controller','diff_control', output='screen', parameters = sl.arg_map('use_slider','v','w'))

    # Start differential control node with parameters from yaml
    with sl.group(if_condition=sl.arg('use_param_yaml')):
        sl.node('diff_robot_controller','diff_control', output='screen', parameters = [sl.find('diff_robot_controller', 'parameters.yaml')])

    return sl.launch_description()
