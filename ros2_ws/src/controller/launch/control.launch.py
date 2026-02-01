#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare launch arguments
    safety_mode_arg = DeclareLaunchArgument(
        'safety_mode',
        default_value='true',
        description='Enable safety mode (check before action)'
    )
    
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable file logging for valve and maintenance actions'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='9999',
        description='TCP server port for broadcasting alerts'
    )
    
    api_port_arg = DeclareLaunchArgument(
        'api_port',
        default_value='5000',
        description='REST API server port for controller'
    )
    
    pipe_monitor_node = Node(
        package='controller',
        executable='controller',
        name='controller_node',
        output='screen',
        parameters=[{
            'safety_mode': LaunchConfiguration('safety_mode'),
            'enable_logging': LaunchConfiguration('enable_logging'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'api_port': LaunchConfiguration('api_port')
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        # Arguments
        safety_mode_arg,
        enable_logging_arg,
        tcp_port_arg,
        api_port_arg,
        
        # Core system nodes (always run)
        pipe_monitor_node,
    ])