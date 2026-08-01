"""Fail-closed development launch for the T265-to-PX4 vision bridge.

This launch deliberately does not create camera_pose_frame -> base_link.  That
transform is an installation-specific measured extrinsic, not a source-code
default.  Supply it from a separately reviewed TF publisher after Workstation
1 has measured it.
"""

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value):
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise RuntimeError('enable_vision_dds must be true or false')


def _node_from_parameters(context):
    params_path = LaunchConfiguration('params_file').perform(context)
    with open(params_path, 'r') as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = dict(document['vision_to_dds_node']['ros__parameters'])
    except (KeyError, TypeError):
        raise RuntimeError('params_file must contain vision_to_dds_node.ros__parameters')
    parameters['world_frame_id'] = LaunchConfiguration('world_frame_id').perform(context)
    parameters['body_frame_id'] = LaunchConfiguration('body_frame_id').perform(context)
    parameters['enable_vision_dds'] = _parse_bool(
        LaunchConfiguration('enable_vision_dds').perform(context))
    return [Node(
        package='vision_to_dds', executable='vision_to_dds_node',
        name='vision_to_dds_node', output='screen', parameters=[parameters])]


def generate_launch_description():
    package_share = get_package_share_directory('vision_to_dds')
    default_parameters = package_share + '/config/vision_to_dds.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_parameters,
            description='vision_to_dds parameter YAML.'),
        DeclareLaunchArgument(
            'world_frame_id', default_value='odom_frame',
            description='TF parent frame; T265 records use odom_frame.'),
        DeclareLaunchArgument(
            'body_frame_id', default_value='base_link',
            description='TF child frame after the measured T265-to-body extrinsic.'),
        DeclareLaunchArgument(
            'enable_vision_dds', default_value='false',
            description='Must remain false unless deliberately enabling the PX4 DDS writer.'),
        OpaqueFunction(function=_node_from_parameters),
    ])
