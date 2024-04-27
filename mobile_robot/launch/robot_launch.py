#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg('use_slider', True, description = 'Use slider for publishing linear and angular velocity')
    sl.declare_arg('gazebo', False, description = 'Open Gazebo')
    sl.declare_arg('v', 0., description = 'Set constant linear velocity')
    sl.declare_arg('w', 0., description = 'Set cosnstant angular velocity')

	# Start the rviz node
    sl.rviz(sl.find('mobile_robot', 'diff.rviz'))
    
    # We get the robot type to generate description
    sl.robot_state_publisher('mobile_robot', 'diff_robot.xacro')

    # Start the slider publisher node
    with sl.group(if_condition=sl.arg('use_slider')):
        sl.node('slider_publisher', 'slider_publisher', name ='Twist', arguments = [sl.find('mobile_robot', 'Twist.yaml')])

    with sl.group(if_condition=sl.arg('gazebo')):
        sl.include('mobile_robot','gazebo_launch.py')

    # Start differentail control node
    sl.node('mobile_robot','diff_control', output='screen', parameters = {'use_slider': sl.arg('use_slider'), 'v': sl.arg('v'), 'w':sl.arg('w')})

    return sl.launch_description()
