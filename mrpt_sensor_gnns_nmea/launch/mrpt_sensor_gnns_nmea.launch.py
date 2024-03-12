# Importing necessary libraries
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    return LaunchDescription([
        # COMMON PARAMS TO ALL MRPT_SENSOR NODES:
        # --------------------------------------------
        # Declare an argument for the config file
        DeclareLaunchArgument(
            'process_rate',
            default_value='"100"',
            description='Rate (Hz) for the process() main sensor loop.'
        ),

        DeclareLaunchArgument(
            'out_rawlog_prefix',
            default_value='',
            description='If not empty, a .rawlog file will be created with all recorded data, apart of publishing it as ROS messages.'
        ),

        DeclareLaunchArgument(
            'publish_mrpt_obs_topic',
            default_value='',
            description='If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name with the binary serialization of mrtp::obs::CObservation objects from the sensor.'
        ),

        DeclareLaunchArgument(
            'publish_topic',
            default_value='sensor',
            description='If not empty, messages of the appropriate type will be published to this topic for each sensor observation.'
        ),

        DeclareLaunchArgument(
            'sensor_frame_id',
            default_value='sensor',
            description='The sensor frame_id name. Used to populate msg header and to publish to /tf too.'
        ),

        # PARAMS FOR THIS NODE:
        # --------------------------------------------
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Serial port to open'
        ),

        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='"4800"',
            description='Serial port baud rate (typ: 4800, 9600, etc.)'
        ),

        DeclareLaunchArgument(
            'sensor_pose_x',
            default_value='"0.0"',
            description='Sensor pose coordinate on the vehicle frame.'
        ),
        DeclareLaunchArgument(
            'sensor_pose_y',
            default_value='"0.0"',
            description='Sensor pose coordinate on the vehicle frame.'
        ),
        DeclareLaunchArgument(
            'sensor_pose_z',
            default_value='"0.0"',
            description='Sensor pose coordinate on the vehicle frame.'
        ),

        # Node to launch the mrpt_generic_sensor_node
        Node(
            package='mrpt_sensor_gnns_nmea',
            executable='mrpt_sensor_gnns_nmea_node',
            name='mrpt_sensor_gnns_nmea',
            output='screen',
            parameters=[
                # ------------------------------------------------
                # common params:
                # ------------------------------------------------
                {'process_rate': LaunchConfiguration('process_rate')},
                {'out_rawlog_prefix': LaunchConfiguration(
                    'out_rawlog_prefix')},
                {'publish_mrpt_obs_topic': LaunchConfiguration(
                    'publish_mrpt_obs_topic')},
                {'publish_topic': LaunchConfiguration('publish_topic')},
                {'sensor_frame_id': LaunchConfiguration('sensor_frame_id')},

                # ------------------------------------------------
                # node params:
                # ------------------------------------------------
                {'serial_port': LaunchConfiguration('serial_port')},
                {'serial_baud_rate': LaunchConfiguration('serial_baud_rate')},

                {'sensor_pose_x': LaunchConfiguration('sensor_pose_x')},
                {'sensor_pose_y': LaunchConfiguration('sensor_pose_y')},
                {'sensor_pose_z': LaunchConfiguration('sensor_pose_z')},
            ]
        )
    ])
