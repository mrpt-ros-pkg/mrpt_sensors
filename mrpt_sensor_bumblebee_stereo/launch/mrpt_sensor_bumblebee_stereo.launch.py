# Bumblebee / Bumblebee2 FireWire stereo camera.
#
# Publishes, in the given namespace:
#   left/image_raw, right/image_raw: every frame, only while subscribed.
#   low_rate/left/image_raw, low_rate/right/image_raw: one frame every
#     1/low_rate_hz seconds.
# Images are the raw Bayer mosaics (bayer_<pattern>8, lossless) unless
# bayer_decoding is true (rgb8). Frames: <sensor_frame_id> (camera body, at the
# left camera) and the optical frames <sensor_frame_id>_left / _right.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

# Launch argument name, default value, description. String values are quoted,
# so ROS keeps them as strings.
ARGS = [
    ('process_rate', '"80"', 'Rate (Hz) of the main sensor loop: above the camera rate.'),
    ('out_rawlog_prefix', '',
     'If not empty, a .rawlog file will be created with all recorded data, apart of publishing '
     'it as ROS messages.'),
    ('publish_mrpt_obs_topic', '',
     'If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name '
     'with the binary serialization of mrtp::obs::CObservation objects from the sensor.'),
    ('sensor_frame_id', 'camera',
     'Camera body frame (x forward, at the left camera). Images are in the optical frames '
     '<sensor_frame_id>_left and _right.'),
    ('robot_frame_id', 'base_link',
     'The robot frame_id name. Used to publish the sensor pose to /tf_static.'),
    ('sensor_label', 'stereo',
     'The sensorLabel field of mrpt::obs::CObservation: a "name" for the sensor.'),
    ('dc1394_framerate', '"15"', 'Camera frame rate (Hz): 1.875, 3.75, 7.5, 15, 30 or 60.'),
    ('dc1394_camera_guid', '"0"',
     '0: the first camera. A hexadecimal number (0x11223344): the GUID of the camera to open.'),
    ('bayer_decoding', '"false"',
     'false: publish the raw Bayer images (lossless, 1 byte/pixel); true: RGB images.'),
    ('bayer_pattern', 'gbrg',
     'Color filter of the raw images (rggb, bggr, gbrg or grbg). Bumblebee2: gbrg.'),
    ('low_rate_hz', '1.0', 'Rate (Hz) of the low_rate/ images; 0 to disable them.'),
    ('publish_full_rate', 'true', 'Whether to advertise the full rate images.'),
    ('stereo_baseline', '0.12', 'Distance between the cameras (m), for the right optical frame.'),
    ('camera_preview_decimation', '"0"',
     'N<=0: No preview; N>0, display 1 out of N captured frames (needs a display).'),
    ('sensor_pose_x', '"0.0"', 'Sensor pose coordinate on the vehicle frame (meters).'),
    ('sensor_pose_y', '"0.0"', 'Sensor pose coordinate on the vehicle frame (meters).'),
    ('sensor_pose_z', '"0.0"', 'Sensor pose coordinate on the vehicle frame (meters).'),
    ('sensor_pose_yaw', '"0.0"', 'Sensor pose coordinate on the vehicle frame (degrees).'),
    ('sensor_pose_pitch', '"0.0"', 'Sensor pose coordinate on the vehicle frame (degrees).'),
    ('sensor_pose_roll', '"0.0"', 'Sensor pose coordinate on the vehicle frame (degrees).'),
]


def generate_launch_description():
    params = {name: LaunchConfiguration(name) for name, _, _ in ARGS}
    # The diagnostics expect one observation per camera frame:
    params['diag_expected_rate'] = PythonExpression(
        ['float(', LaunchConfiguration('dc1394_framerate'), ')'])

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value, description=desc)
         for name, value, desc in ARGS] + [
            DeclareLaunchArgument('namespace', default_value='bumblebee_stereo',
                                  description='Namespace of the node and its topics.'),
            DeclareLaunchArgument('node_name', default_value='mrpt_sensor_bumblebee_stereo',
                                  description='Node name.'),
            DeclareLaunchArgument('log_level', default_value='INFO',
                                  description='Logging level'),
            # The group keeps the namespace to this node:
            GroupAction([
                PushRosNamespace(LaunchConfiguration('namespace')),
                Node(
                    package='mrpt_sensor_bumblebee_stereo',
                    executable='mrpt_sensor_bumblebee_stereo_node',
                    name=LaunchConfiguration('node_name'),
                    output='screen',
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                    parameters=[params]),
            ]),
        ])
