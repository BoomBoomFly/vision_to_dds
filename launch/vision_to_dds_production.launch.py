"""Explicitly armed production launch for vision_to_dds.

Both ``production:=true`` and ``enable_vision_dds:=true`` are mandatory.  A
measured t265_pose_frame -> base_link transform YAML is also mandatory and is
validated before either the static TF publisher or DDS writer is started.
"""

import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_T265_ODOMETRY_TOPIC = '/t265/pose/sample'
PRODUCTION_CONTRACT_PARAMETERS = {
    'enable_vision_dds': True,
    'world_frame_id': 'odom_frame',
    'body_frame_id': 'base_link',
    'vehicle_visual_odometry_topic': '/fmu/in/vehicle_visual_odometry',
    'quality_topic': '/vision/quality',
    'source_epoch_topic': '/vision/source_epoch',
}


def _required_true(context, name):
    value = context.launch_configurations.get(name)
    if value is None or value.lower() != 'true':
        raise RuntimeError('production launch requires {}:=true'.format(name))


def _required_value(context, name, expected):
    value = LaunchConfiguration(name).perform(context)
    if value != expected:
        raise RuntimeError(
            'production launch requires {}:={}'.format(name, expected))


def _load_measured_extrinsics(path):
    if not path or not os.path.isabs(path) or not os.path.isfile(path):
        raise RuntimeError(
            'production launch requires '
            't265_to_base_link_extrinsics_file:=/absolute/measured.yaml')
    with open(path, 'r') as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = document['t265_to_base_link']['ros__parameters']
        parent = parameters['parent_frame_id']
        child = parameters['child_frame_id']
        translation = parameters['translation_m']
        rotation = parameters['rotation_xyzw']
    except (KeyError, TypeError):
        raise RuntimeError(
            'invalid measured extrinsics YAML; use '
            't265_to_base_link.extrinsics.yaml.template')
    if parent != 't265_pose_frame' or child != 'base_link':
        raise RuntimeError('measured extrinsics must define t265_pose_frame -> base_link')
    if not isinstance(translation, list) or len(translation) != 3 or \
            not isinstance(rotation, list) or len(rotation) != 4:
        raise RuntimeError(
            'extrinsics translation_m must have 3 and rotation_xyzw must have 4 values')
    try:
        values = [float(value) for value in translation + rotation]
    except (TypeError, ValueError):
        raise RuntimeError('extrinsics must contain only measured numeric values')
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError('extrinsics contain non-finite values')
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if abs(norm - 1.0) > 0.01:
        raise RuntimeError('extrinsics rotation_xyzw must be a unit quaternion')
    return parent, child, values


def _load_production_parameters(path):
    if not path or not os.path.isfile(path):
        raise RuntimeError('params_file must name a readable vision_to_dds YAML file')
    with open(path, 'r') as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = dict(document['vision_to_dds_node']['ros__parameters'])
    except (KeyError, TypeError):
        raise RuntimeError('params_file must contain vision_to_dds_node.ros__parameters')
    # Merge before constructing the Node.  ROS 2 Foxy does not reliably apply
    # a dictionary after a parameter file as a final precedence layer.
    parameters.update(PRODUCTION_CONTRACT_PARAMETERS)
    return parameters


def _start_production(context):
    _required_true(context, 'production')
    _required_true(context, 'enable_vision_dds')
    _required_value(context, 'world_frame_id', 'odom_frame')
    _required_value(context, 'body_frame_id', 'base_link')
    parent, child, values = _load_measured_extrinsics(
        LaunchConfiguration('t265_to_base_link_extrinsics_file').perform(context))
    production_parameters = _load_production_parameters(
        LaunchConfiguration('params_file').perform(context))
    return [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='t265_to_base_link_static_tf', output='screen',
            arguments=[str(value) for value in values] + [parent, child],
        ),
        Node(
            package='vision_to_dds', executable='t265_health_adapter_node',
            name='t265_health_adapter_node', output='screen',
            parameters=[{
                'odometry_topic': LaunchConfiguration('t265_odometry_topic'),
                'quality_topic': '/vision/quality',
                'source_epoch_topic': '/vision/source_epoch',
            }],
        ),
        Node(
            package='vision_to_dds', executable='vision_to_dds_node',
            name='vision_to_dds_node', output='screen',
            parameters=[production_parameters],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('vision_to_dds')
    return LaunchDescription([
        # The measured installation file is packaged; production and writer enablement remain explicit.
        DeclareLaunchArgument(
            'production',
            description='Set exactly to true to arm this production launch.'),
        DeclareLaunchArgument(
            'enable_vision_dds',
            description='Set exactly to true to create the PX4 DDS writer.'),
        DeclareLaunchArgument(
            't265_to_base_link_extrinsics_file',
            default_value=package_share + '/config/t265_to_base_link.extrinsics.yaml',
            description='Measured T265-to-body extrinsics YAML installed with vision_to_dds.'),
        DeclareLaunchArgument(
            'params_file', default_value=package_share + '/config/vision_to_dds.enabled.yaml',
            description='Production parameter YAML; it must retain enable_vision_dds: true.'),
        DeclareLaunchArgument(
            'world_frame_id', default_value='odom_frame',
            description='TF parent frame; must match the physical T265 odometry frame.'),
        DeclareLaunchArgument(
            'body_frame_id', default_value='base_link',
            description='TF child frame; production requires base_link.'),
        DeclareLaunchArgument(
            't265_odometry_topic', default_value=DEFAULT_T265_ODOMETRY_TOPIC,
            description='Observed T265 nav_msgs/Odometry topic used for measured health.'),
        OpaqueFunction(function=_start_production),
    ])
