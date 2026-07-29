import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] /
    'launch' /
    'vision_to_dds_production.launch.py'
)
SPEC = importlib.util.spec_from_file_location('vision_to_dds_production_launch', LAUNCH_PATH)
PRODUCTION_LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PRODUCTION_LAUNCH)


def _write_extrinsics(tmp_path, translation=None, rotation=None):
    path = tmp_path / 'measured.yaml'
    document = {
        't265_to_base_link': {
            'ros__parameters': {
                'parent_frame_id': 't265_pose_frame',
                'child_frame_id': 'base_link',
                'translation_m': translation or [0.1, -0.2, 0.3],
                'rotation_xyzw': rotation or [0.0, 0.0, 0.0, 1.0],
            },
        },
    }
    path.write_text(yaml.safe_dump(document))
    return path


def _write_custom_params(tmp_path):
    path = tmp_path / 'custom.yaml'
    document = {
        'vision_to_dds_node': {
            'ros__parameters': {
                'enable_vision_dds': False,
                'world_frame_id': 'redirected_world',
                'body_frame_id': 'redirected_body',
                'vehicle_visual_odometry_topic': '/redirected/visual_odometry',
                'quality_topic': '/redirected/quality',
                'source_epoch_topic': '/redirected/source_epoch',
                'minimum_quality': 75,
            },
        },
    }
    path.write_text(yaml.safe_dump(document))
    return path


def test_production_contract_cannot_be_redirected_by_params_file():
    assert PRODUCTION_LAUNCH.DEFAULT_T265_ODOMETRY_TOPIC == '/t265/pose/sample'
    assert PRODUCTION_LAUNCH.PRODUCTION_CONTRACT_PARAMETERS == {
        'enable_vision_dds': True,
        'world_frame_id': 'odom_frame',
        'body_frame_id': 'base_link',
        'vehicle_visual_odometry_topic': '/fmu/in/vehicle_visual_odometry',
        'quality_topic': '/vision/quality',
        'source_epoch_topic': '/vision/source_epoch',
    }


def test_custom_parameter_file_is_merged_then_production_contract_wins(tmp_path):
    path = _write_custom_params(tmp_path)
    parameters = PRODUCTION_LAUNCH._load_production_parameters(str(path))
    assert parameters['minimum_quality'] == 75
    assert {
        key: parameters[key] for key in PRODUCTION_LAUNCH.PRODUCTION_CONTRACT_PARAMETERS
    } == PRODUCTION_LAUNCH.PRODUCTION_CONTRACT_PARAMETERS


def test_production_requires_both_explicit_confirmations():
    context = SimpleNamespace(launch_configurations={})
    with pytest.raises(RuntimeError, match='production:=true'):
        PRODUCTION_LAUNCH._required_true(context, 'production')

    context.launch_configurations['production'] = 'true'
    PRODUCTION_LAUNCH._required_true(context, 'production')
    with pytest.raises(RuntimeError, match='enable_vision_dds:=true'):
        PRODUCTION_LAUNCH._required_true(context, 'enable_vision_dds')

    context.launch_configurations['enable_vision_dds'] = 'true'
    PRODUCTION_LAUNCH._required_true(context, 'enable_vision_dds')


def test_production_rejects_missing_extrinsics():
    with pytest.raises(RuntimeError, match='absolute/measured.yaml'):
        PRODUCTION_LAUNCH._load_measured_extrinsics('')
    with pytest.raises(RuntimeError, match='absolute/measured.yaml'):
        PRODUCTION_LAUNCH._load_measured_extrinsics('/does/not/exist/measured.yaml')


def test_measured_extrinsics_accepts_only_expected_chain_and_unit_quaternion(tmp_path):
    path = _write_extrinsics(tmp_path)
    parent, child, values = PRODUCTION_LAUNCH._load_measured_extrinsics(str(path))
    assert parent == 't265_pose_frame'
    assert child == 'base_link'
    assert values == [0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]

    wrong_chain = yaml.safe_load(path.read_text())
    wrong_chain['t265_to_base_link']['ros__parameters']['child_frame_id'] = 'camera_link'
    path.write_text(yaml.safe_dump(wrong_chain))
    with pytest.raises(RuntimeError, match='t265_pose_frame -> base_link'):
        PRODUCTION_LAUNCH._load_measured_extrinsics(str(path))

    nonunit = _write_extrinsics(tmp_path, rotation=[0.0, 0.0, 0.0, 2.0])
    with pytest.raises(RuntimeError, match='unit quaternion'):
        PRODUCTION_LAUNCH._load_measured_extrinsics(str(nonunit))


def test_extrinsics_rejects_relative_placeholder_and_nonfinite_values(tmp_path, monkeypatch):
    path = _write_extrinsics(tmp_path)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(RuntimeError, match='absolute/measured.yaml'):
        PRODUCTION_LAUNCH._load_measured_extrinsics(path.name)

    placeholder = _write_extrinsics(tmp_path, translation=['REQUIRED_X_M', 0.0, 0.0])
    with pytest.raises(RuntimeError, match='measured numeric values'):
        PRODUCTION_LAUNCH._load_measured_extrinsics(str(placeholder))

    nonfinite = _write_extrinsics(tmp_path, translation=[math.inf, 0.0, 0.0])
    with pytest.raises(RuntimeError, match='non-finite'):
        PRODUCTION_LAUNCH._load_measured_extrinsics(str(nonfinite))
