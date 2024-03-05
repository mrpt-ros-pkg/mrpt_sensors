# Importing necessary libraries
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    myDir = get_package_share_directory("mrpt_sensors")

    config_file_path = os.path.join(myDir, 'ini', 'nmea_gnns.ini')

    # Specify the config section
    config_section = 'GNNS'

    return LaunchDescription([
        # Declare an argument for the config file
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Path to the config .ini file'
        ),

        # Declare an argument for the config section
        DeclareLaunchArgument(
            'config_section',
            default_value=config_section,
            description='INI file config section name to read'
        ),

        DeclareLaunchArgument(
            'gnns_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port to open'
        ),

        DeclareLaunchArgument(
            'gnns_serial_rate',
            default_value='4800',
            description='Serial port baud rate (typ: 4800, 9600, etc.)'
        ),

        # Node to launch the mrpt_generic_sensor_node
        Node(
            package='mrpt_generic_sensor',
            executable='mrpt_generic_sensor_node',
            name='mrpt_generic_sensor',
            output='screen',
            parameters=[
                {'config_file': LaunchConfiguration('config_file')},
                {'config_section': LaunchConfiguration('config_section')},
                {'gnns_serial_port': LaunchConfiguration('gnns_serial_port')},
                {'gnns_serial_rate': LaunchConfiguration('gnns_serial_rate')},
            ]
        )
    ])
