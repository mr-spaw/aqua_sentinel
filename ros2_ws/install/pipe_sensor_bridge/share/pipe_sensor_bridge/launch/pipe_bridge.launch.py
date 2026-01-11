#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    json_path_arg = DeclareLaunchArgument(
        'json_path',
        default_value='/home/arka/aqua_sentinel/simulator/init_sensor.json',
        description='Path to init_sensor.json file'
    )
    
    total_sensors_arg = DeclareLaunchArgument(
        'total_sensors',
        default_value='300',
        description='Total number of sensors'
    )
    
    json_path = LaunchConfiguration('json_path')
    total_sensors = LaunchConfiguration('total_sensors')
    
    init_publisher_node = Node(
        package='pipe_sensor_bridge',
        executable='init_config_publisher',
        name='init_config_publisher',
        output='screen',
        parameters=[{
            'json_path': json_path,
            'total_sensors': total_sensors
        }]
    )
    
    mqtt_subscriber_node = Node(
        package='pipe_sensor_bridge',
        executable='mqtt_subscriber_bridge',
        name='mqtt_subscriber_bridge',
        output='screen'
    )
    
    return LaunchDescription([
        json_path_arg,
        total_sensors_arg,
        init_publisher_node,
        mqtt_subscriber_node
    ])
