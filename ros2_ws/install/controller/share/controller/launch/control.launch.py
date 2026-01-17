#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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
        description='REST API server port'
    )
    
    # Pipe monitor node
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
        respawn=True,  # Auto-restart if crashes
        respawn_delay=5.0,  # Wait 5 seconds before restarting
    )

    echo_esp1 = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/esp1/sensors'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_echo', default='false'))
    )
    
    echo_valve = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/valve_control'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_echo', default='false'))
    )
    
    echo_maintenance = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/maintenance/request'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_echo', default='false'))
    )
    
    # Delayed health check (starts after 10 seconds)
    health_check = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['curl', '-s', 'http://localhost:5000/status'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('health_check', default='false'))
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        safety_mode_arg,
        enable_logging_arg,
        tcp_port_arg,
        api_port_arg,
        
        # Nodes
        pipe_monitor_node,
        
    ])