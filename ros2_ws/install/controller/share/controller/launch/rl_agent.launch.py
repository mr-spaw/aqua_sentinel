#!/usr/bin/env python3
"""
Launch file for RL Agent
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='deploy',
            description='Operation mode'
        ),
        
        DeclareLaunchArgument(
            'api_port',
            default_value='5000',
            description='REST API port'
        ),
        
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'controller', 'rl_agent',
                '--mode', LaunchConfiguration('mode'),
                '--api-port', LaunchConfiguration('api_port'),
            ],
            name='rl_agent',
            output='screen',
            shell=False,
        ),
    ])