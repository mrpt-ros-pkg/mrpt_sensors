# Importing necessary libraries
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import os


def generate_launch_description():
    namespace = 'gnss'

    ld = LaunchDescription([
        # COMMON PARAMS TO ALL MRPT_SENSOR NODES:
        # --------------------------------------------
        DeclareLaunchArgument(
            'process_rate',
            default_value='"100"',
            description='Rate (Hz) for the process() main sensor loop.'
        ),

        DeclareLaunchArgument(
            'out_rawlog_prefix',
            default_value='',
            description='If not empty, a .rawlog file will be created with all recorded data.'
        ),

        DeclareLaunchArgument(
            'publish_mrpt_obs_topic',
            default_value='',
            description='If not empty, mrpt_msgs/GenericObservation messages will be published.'
        ),

        DeclareLaunchArgument(
            'publish_topic',
            default_value='sensor',
            description='Topic name for sensor_msgs/NavSatFix messages.'
        ),

        DeclareLaunchArgument(
            'sensor_frame_id',
            default_value='novatel',
            description='The sensor frame_id name.'
        ),

        DeclareLaunchArgument(
            'robot_frame_id',
            default_value='base_link',
            description='The robot frame_id name.'
        ),

        DeclareLaunchArgument(
            'sensor_label',
            default_value='novatel',
            description='The sensorLabel field of mrpt::obs::CObservation.'
        ),

        # NOVATEL SERIAL PORT PARAMS:
        # --------------------------------------------
        DeclareLaunchArgument(
            'novatel_main_serial_port',
            default_value='',
            description='Main Novatel comms port (USB1 typically). REQUIRED.'
        ),
        DeclareLaunchArgument(
            'novatel_ntrip_serial_port',
            default_value='',
            description='Novatel port for NTRIP corrections (USB3 typically). REQUIRED.'
        ),
        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='"4800"',
            description='Serial port baud rate.'
        ),

        DeclareLaunchArgument(
            'raw_dump_file',
            default_value='""',
            description='If not empty, raw GNSS data will be dumped to this file prefix.'
        ),

        # IMU/INS CONFIGURATION:
        # --------------------------------------------
        DeclareLaunchArgument(
            'novatel_imu_orientation',
            default_value='"6"',
            description='IMU mounting orientation code. See Novatel manual Table 9. 6 = Z-down, Y-forward.'
        ),
        DeclareLaunchArgument(
            'novatel_veh_body_rotation',
            default_value='"0.000000 0.000000 90.000000 0.000000 0.000000 0.000000"',
            description='VEHICLEBODYROTATION: roll pitch azimuth roll_std pitch_std az_std (degrees).'
        ),
        DeclareLaunchArgument(
            'novatel_imu_to_ant_offset',
            default_value='"-0.28 -0.08 -0.01 0.000000 0.000000 0.000000"',
            description='SETIMUTOANTOFFSET: x y z x_std y_std z_std (meters). '
                        'IMU center to antenna phase center offset.'
        ),
        DeclareLaunchArgument(
            'novatel_ins_offset',
            default_value='"0.0 0.0 0.0"',
            description='SETINSOFFSET: x y z (meters). INS solution output offset.'
        ),

        # INS ALIGNMENT CONFIGURATION:
        # --------------------------------------------
        DeclareLaunchArgument(
            'alignment_mode',
            default_value='"UNAIDED"',
            description='INS alignment mode: UNAIDED, KINEMATIC, AUTOMATIC, AIDED_TRANSFER, STATIC.'
        ),
        DeclareLaunchArgument(
            'novatel_init_azimuth',
            default_value='"0.0 25.0"',
            description='Manual initial azimuth: "azimuth_deg std_deg". '
                        'Used if external IMU not available/fresh.'
        ),

        # EXTERNAL IMU FOR AZIMUTH INITIALIZATION:
        # --------------------------------------------
        DeclareLaunchArgument(
            'external_imu_topic',
            default_value='',
            description='Topic name for external IMU (sensor_msgs/Imu) to get initial azimuth. '
                        'Leave empty to disable.'
        ),
        DeclareLaunchArgument(
            'use_external_imu_azimuth',
            default_value='false',
            description='If true and external_imu_topic is set, use external IMU for azimuth init.'
        ),
        DeclareLaunchArgument(
            'external_imu_max_age_sec',
            default_value='2.0',
            description='Maximum age (seconds) of external IMU data to be considered valid.'
        ),
        DeclareLaunchArgument(
            'external_imu_azimuth_deg_for_yaw_zero',
            default_value='0.0',
            description='Offset between the IMU yaw=0 direction and true North (the azimuth reference), in degrees.'
        ),

        # NTRIP CONFIGURATION:
        # --------------------------------------------
        DeclareLaunchArgument(
            'ntrip_server',
            default_value='"www.euref-ip.net"',
            description='DNS or IP of the NTRIP server.'
        ),
        DeclareLaunchArgument(
            'ntrip_port',
            default_value='"2101"',
            description='TCP port for connecting to the NTRIP server.'
        ),
        DeclareLaunchArgument(
            'ntrip_mount_point',
            default_value='"ALME00ESP0"',
            description='Mount point to connect inside the NTRIP server. REQUIRED.'
        ),
        DeclareLaunchArgument(
            'ntrip_user',
            default_value='""',
            description='NTRIP server username.'
        ),
        DeclareLaunchArgument(
            'ntrip_password',
            default_value='""',
            description='NTRIP server password.'
        ),

        # SENSOR POSE:
        # --------------------------------------------
        DeclareLaunchArgument(
            'sensor_pose_x',
            default_value='"0.0"',
            description='Sensor pose X coordinate on the vehicle frame (meters).'
        ),
        DeclareLaunchArgument(
            'sensor_pose_y',
            default_value='"0.0"',
            description='Sensor pose Y coordinate on the vehicle frame (meters).'
        ),
        DeclareLaunchArgument(
            'sensor_pose_z',
            default_value='"0.0"',
            description='Sensor pose Z coordinate on the vehicle frame (meters).'
        ),

        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("INFO")),
            description="Logging level"
        ),

        # Node to launch the mrpt_sensor_gnss_novatel_node
        Node(
            package='mrpt_sensor_gnss_novatel',
            executable='mrpt_sensor_gnss_novatel_node',
            name='mrpt_sensor_gnss_novatel',
            output='screen',
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            parameters=[
                # Common params:
                {'process_rate': LaunchConfiguration('process_rate')},
                {'out_rawlog_prefix': LaunchConfiguration(
                    'out_rawlog_prefix')},
                {'publish_mrpt_obs_topic': LaunchConfiguration(
                    'publish_mrpt_obs_topic')},
                {'publish_topic': LaunchConfiguration('publish_topic')},
                {'sensor_frame_id': LaunchConfiguration('sensor_frame_id')},
                {'robot_frame_id': LaunchConfiguration('robot_frame_id')},
                {'sensor_label': LaunchConfiguration('sensor_label')},

                # Novatel serial ports:
                {'novatel_main_serial_port': LaunchConfiguration(
                    'novatel_main_serial_port')},
                {'novatel_ntrip_serial_port': LaunchConfiguration(
                    'novatel_ntrip_serial_port')},
                {'serial_baud_rate': LaunchConfiguration('serial_baud_rate')},
                {'raw_dump_file': LaunchConfiguration('raw_dump_file')},

                # IMU/INS configuration:
                {'novatel_imu_orientation': LaunchConfiguration(
                    'novatel_imu_orientation')},
                {'novatel_veh_body_rotation': LaunchConfiguration(
                    'novatel_veh_body_rotation')},
                {'novatel_imu_to_ant_offset': LaunchConfiguration(
                    'novatel_imu_to_ant_offset')},
                {'novatel_ins_offset': LaunchConfiguration(
                    'novatel_ins_offset')},

                # INS alignment:
                {'alignment_mode': LaunchConfiguration('alignment_mode')},
                {'novatel_init_azimuth': LaunchConfiguration(
                    'novatel_init_azimuth')},

                # External IMU:
                {'external_imu_topic': LaunchConfiguration(
                    'external_imu_topic')},
                {'use_external_imu_azimuth': LaunchConfiguration(
                    'use_external_imu_azimuth')},
                {'external_imu_max_age_sec': LaunchConfiguration(
                    'external_imu_max_age_sec')},
                {'external_imu_azimuth_deg_for_yaw_zero': LaunchConfiguration(
                    'external_imu_azimuth_deg_for_yaw_zero')},

                # NTRIP:
                {'ntrip_server': LaunchConfiguration('ntrip_server')},
                {'ntrip_port': LaunchConfiguration('ntrip_port')},
                {'ntrip_mount_point': LaunchConfiguration(
                    'ntrip_mount_point')},
                {'ntrip_user': LaunchConfiguration('ntrip_user')},
                {'ntrip_password': LaunchConfiguration('ntrip_password')},

                # Sensor pose:
                {'sensor_pose_x': LaunchConfiguration('sensor_pose_x')},
                {'sensor_pose_y': LaunchConfiguration('sensor_pose_y')},
                {'sensor_pose_z': LaunchConfiguration('sensor_pose_z')},
            ]
        )
    ])

    # Namespace to avoid clash launch argument names with the parent scope:
    return LaunchDescription([GroupAction(
        actions=[
            # condition=IfCondition(use_namespace),
            PushRosNamespace(namespace=namespace),
            ld
        ])])
