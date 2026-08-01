"""Fail-closed, complete D435i stereo-inertial VIO to PX4 launch.

This is the single entry point for the verified D435i VIO chain: RealSense,
Madgwick IMU filter, the missing IMU optical TF, RTAB-Map stereo odometry,
the measured body extrinsic, source health, and the optional PX4 writer.
PX4 writing remains disabled by default.
"""

import math
import os

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value):
    value = value.strip().lower()
    if value == 'true':
        return True
    if value == 'false':
        return False
    raise RuntimeError('enable_vision_dds must be true or false')


def _load_extrinsics(path):
    if not path or not os.path.isabs(path) or not os.path.isfile(path):
        raise RuntimeError('d435i_to_base_link_extrinsics_file must be an absolute readable YAML path')
    with open(path, 'r') as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = document['d435i_to_base_link']['ros__parameters']
        parent = parameters['parent_frame_id']
        child = parameters['child_frame_id']
        translation = parameters['translation_m']
        rotation = parameters['rotation_xyzw']
    except (KeyError, TypeError):
        raise RuntimeError('invalid D435i extrinsics YAML')
    if parent != 'camera_link' or child != 'base_link':
        raise RuntimeError('D435i extrinsics must define camera_link -> base_link')
    if not isinstance(translation, list) or len(translation) != 3 or \
            not isinstance(rotation, list) or len(rotation) != 4:
        raise RuntimeError('D435i extrinsics require translation_m[3] and rotation_xyzw[4]')
    try:
        values = [float(value) for value in translation + rotation]
    except (TypeError, ValueError):
        raise RuntimeError('D435i extrinsics must be numeric')
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError('D435i extrinsics must be finite')
    if abs(math.sqrt(sum(value * value for value in values[3:])) - 1.0) > 0.01:
        raise RuntimeError('D435i extrinsics quaternion must be unit length')
    return parent, child, values


def _start(context):
    package_share = get_package_share_directory('vision_to_dds')
    parent, child, values = _load_extrinsics(
        LaunchConfiguration('d435i_to_base_link_extrinsics_file').perform(context))
    enabled = _parse_bool(LaunchConfiguration('enable_vision_dds').perform(context))
    with open(package_share + '/config/vision_to_dds.yaml', 'r') as stream:
        document = yaml.safe_load(stream)
    parameters = dict(document['vision_to_dds_node']['ros__parameters'])
    parameters.update({
        'enable_vision_dds': enabled,
        'world_frame_id': 'odom_frame',
        'body_frame_id': 'base_link',
        'vehicle_visual_odometry_topic': '/fmu/in/vehicle_visual_odometry',
        'quality_topic': '/vision/quality',
        'source_epoch_topic': '/vision/source_epoch',
        # RTAB-Map D435i odometry runs at roughly 3--7 Hz on this platform.
        # RTAB-Map may briefly stall for a little over one second on this
        # platform. Keep all health/sample timeouts aligned; stale input is
        # fail-silent and self-recovers through the bridge warm-up path.
        'maximum_sample_age_s': 1.50,
        'quality_timeout_s': 1.50,
    })
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'false',
            # D435i has no tracking pose/fisheye sensors. Explicitly disable
            # the rs_launch defaults so an unsupported stream cannot block the
            # infrared and IMU streams needed by stereo VIO.
            'enable_pose': 'false',
            'enable_fisheye1': 'false',
            'enable_fisheye2': 'false',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_sync': 'true',
            'depth_module.profile': '640x480x15',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
        }.items())

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='d435i_vio_imu_filter', output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
        }],
        remappings=[
            ('/imu/data_raw', '/camera/imu'),
            ('/imu/data', '/d435i_vio/imu'),
        ])

    imu_optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='d435i_gyro_to_imu_optical_static_tf', output='screen',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_gyro_optical_frame', 'camera_imu_optical_frame'])

    stereo_odometry = Node(
        package='rtabmap_odom', executable='stereo_odometry',
        name='d435i_stereo_odometry', output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom_frame',
            'publish_tf': True,
            'subscribe_stereo': True,
            'wait_imu_to_init': True,
            # D435i infrared timestamps are close but not bit-identical on
            # this platform. Use approximate pairing with a strict 10 ms
            # bound: it restores VIO callbacks while rejecting the observed
            # ~66 ms adjacent-frame mismatches.
            'approx_sync': True,
            'approx_sync_max_interval': 0.01,
            # RealSense sensor streams may publish with SENSOR_DATA QoS.
            # Best Effort subscribers remain compatible with either QoS.
            'qos': 2,
            'qos_camera_info': 2,
        }],
        remappings=[
            ('imu', '/d435i_vio/imu'),
            ('left/image_rect', '/camera/infra1/image_rect_raw'),
            ('left/camera_info', '/camera/infra1/camera_info'),
            ('right/image_rect', '/camera/infra2/image_rect_raw'),
            ('right/camera_info', '/camera/infra2/camera_info'),
            ('odom', '/d435i_vio/odometry'),
        ])

    static_transform_publisher = os.path.join(
        get_package_prefix('tf2_ros'), 'lib', 'tf2_ros', 'static_transform_publisher')

    return [
        camera,
        TimerAction(
            period=LaunchConfiguration('imu_filter_delay'),
            actions=[imu_filter, imu_optical_tf]),
        TimerAction(
            period=LaunchConfiguration('vio_start_delay'),
            actions=[stereo_odometry]),
        # In ROS 2 Foxy, a leading negative translation is mistaken for a
        # command-line option unless it follows the explicit ROS-argument
        # delimiter. ExecuteProcess avoids launch_ros appending a second
        # ``--ros-args`` block after that delimiter.
        ExecuteProcess(
            cmd=[static_transform_publisher, '--ros-args', '--'] +
                [str(value) for value in values] + [parent, child],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', package_share + '/launch/d435i_vio_health.py', '--ros-args',
                 '-p', 'odometry_topic:=/d435i_vio/odometry',
                 '-p', 'quality_topic:=/vision/quality',
                 '-p', 'source_epoch_topic:=/vision/source_epoch',
                 '-p', 'stream_timeout_s:=1.50'],
            output='screen'),
        Node(
            package='vision_to_dds', executable='vision_to_dds_node',
            name='d435i_vio_to_px4', output='screen', parameters=[parameters]),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('vision_to_dds')
    return LaunchDescription([
        DeclareLaunchArgument(
            'd435i_to_base_link_extrinsics_file',
            default_value=package_share + '/config/d435i_to_base_link.extrinsics.yaml',
            description='Measured camera_link -> base_link transform.'),
        DeclareLaunchArgument(
            'enable_vision_dds', default_value='true',
            description='Set true only after stationary and hand-held validation.'),
        DeclareLaunchArgument(
            'imu_filter_delay', default_value='2.0',
            description='Seconds to let the D435i driver initialise before starting IMU consumers.'),
        DeclareLaunchArgument(
            'vio_start_delay', default_value='4.0',
            description='Seconds to let the D435i and filtered IMU initialise before RTAB-Map.'),
        OpaqueFunction(function=_start),
    ])
