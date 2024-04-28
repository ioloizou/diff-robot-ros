#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

	# Start the keyboard teleop node
    sl.node('teleop_twist_keyboard','teleop_twist_keyboard', prefix='xterm -e')
    
    return sl.launch_description()
