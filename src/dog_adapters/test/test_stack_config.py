from pathlib import Path

import pytest
import yaml

from dog_adapters.stack_config import ADAPTERS, COMMAND_TOPIC, SAFE_FLAGS, load_stack_config

CONFIG = Path(__file__).parents[1] / 'config' / 'loopback.yaml'


@pytest.fixture
def document():
    return yaml.safe_load(CONFIG.read_text())


def save(tmp_path, document):
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump(document))
    return str(path)


@pytest.mark.parametrize('adapter', ADAPTERS)
@pytest.mark.parametrize('with_web', [True, False])
def test_selection_and_shared_override(tmp_path, document, adapter, with_web):
    follow = document['rs_follow_node']['ros__parameters']
    for flag in SAFE_FLAGS:
        follow[flag] = True
    for name in ('rs_follow_node', ADAPTERS[adapter][2], 'web_ui'):
        document[name]['ros__parameters']['cmd_vel_topic'] = '/standalone'
    algorithm, selected, web = load_stack_config(save(tmp_path, document), adapter, with_web)
    assert all(algorithm[flag] is False for flag in SAFE_FLAGS)
    assert algorithm['cmd_vel_topic'] == selected['cmd_vel_topic'] == COMMAND_TOPIC
    assert (web is not None) == with_web
    if web is not None:
        assert web['cmd_vel_topic'] == COMMAND_TOPIC


@pytest.mark.parametrize('key', ['input_topic', 'control_frame', 'odom_topic',
                                'robot_length', 'robot_width', 'frame_front', 'frame_back',
                                'frame_left', 'frame_right', 'height_min', 'height_max',
                                'low_height_min', 'low_height_max', 'enable_low_band',
                                'auto_frame'])
def test_geometry_must_be_explicit(tmp_path, document, key):
    del document['rs_follow_node']['ros__parameters'][key]
    with pytest.raises(ValueError, match=key):
        load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('key', ['robot_length', 'robot_width', 'frame_front', 'frame_back',
                                'frame_left', 'frame_right'])
@pytest.mark.parametrize('value', [0, -1, float('inf'), float('nan'), True, '1'])
def test_invalid_geometry(tmp_path, document, key, value):
    document['rs_follow_node']['ros__parameters'][key] = value
    with pytest.raises(ValueError, match=key):
        load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('key', ['input_topic', 'control_frame', 'odom_topic'])
@pytest.mark.parametrize('value', ['', ' ', None, 42])
def test_invalid_strings(tmp_path, document, key, value):
    document['rs_follow_node']['ros__parameters'][key] = value
    with pytest.raises(ValueError, match=key):
        load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('low,high', [('height_min', 'height_max'),
                                     ('low_height_min', 'low_height_max')])
@pytest.mark.parametrize('value', [float('inf'), float('nan'), 100, True])
def test_invalid_height_band(tmp_path, document, low, high, value):
    document['rs_follow_node']['ros__parameters'][low] = value
    with pytest.raises(ValueError):
        load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('metadata', [{}, {'nonhardware': 'true', 'tf_source': 'measured'},
                                    {'nonhardware': False, 'tf_source': 'loopback_identity'},
                                    {'nonhardware': False, 'tf_source': 'identity'},
                                    {'nonhardware': False, 'tf_source': ''}])
def test_tf_source_required(tmp_path, document, metadata):
    document['deployment'] = metadata
    with pytest.raises(ValueError):
        load_stack_config(save(tmp_path, document), 'twist', False)


def test_real_tf_source(tmp_path, document):
    document['deployment'] = {'nonhardware': False, 'tf_source': 'robot_state_publisher measured lidar mount'}
    load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('output', ['/rs_follow/cmd_vel', 'rs_follow/cmd_vel', '', ' '])
def test_twist_output_required_and_not_looped(tmp_path, document, output):
    document['twist_adapter']['ros__parameters']['output_topic'] = output
    with pytest.raises(ValueError):
        load_stack_config(save(tmp_path, document), 'twist', False)


def test_unitree_request_required(tmp_path, document):
    del document['unitree_adapter']['ros__parameters']['request_topic']
    with pytest.raises(ValueError, match='request_topic'):
        load_stack_config(save(tmp_path, document), 'unitree_sport', False)


def test_nonhardware_m20_cannot_target_hardware(tmp_path, document):
    document['m20_bridge']['ros__parameters']['ip'] = '10.21.31.103'
    with pytest.raises(ValueError, match='loopback ip'):
        load_stack_config(save(tmp_path, document), 'm20', False)


def test_only_selected_section_required(tmp_path, document):
    del document['m20_bridge'], document['unitree_adapter'], document['web_ui']
    load_stack_config(save(tmp_path, document), 'twist', False)
    with pytest.raises(ValueError, match='web_ui'):
        load_stack_config(save(tmp_path, document), 'twist', True)


def test_missing_file_and_invalid_arguments(tmp_path):
    with pytest.raises(ValueError, match='existing YAML'):
        load_stack_config(str(tmp_path / 'absent.yaml'), 'twist', False)
    with pytest.raises(ValueError, match='adapter'):
        load_stack_config(str(CONFIG), 'other', False)
    with pytest.raises(ValueError, match='bool'):
        load_stack_config(str(CONFIG), 'twist', 'false')


@pytest.mark.parametrize('contents', ['', '- not a mapping', 'rs_follow_node: wrong'])
def test_invalid_document(tmp_path, contents):
    path = tmp_path / 'invalid.yaml'
    path.write_text(contents)
    with pytest.raises(ValueError):
        load_stack_config(str(path), 'twist', False)


@pytest.mark.parametrize('value', [None, 0, 1, 'true', 'false'])
def test_auto_frame_requires_bool(tmp_path, document, value):
    document['rs_follow_node']['ros__parameters']['auto_frame'] = value
    with pytest.raises(ValueError, match='auto_frame'):
        load_stack_config(save(tmp_path, document), 'twist', False)


def test_auto_frame_requires_explicit_margin(tmp_path, document):
    document['rs_follow_node']['ros__parameters']['auto_frame'] = True
    with pytest.raises(ValueError, match='self_occlusion_margin'):
        load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('value', [-0.01, float('inf'), float('nan'), True, '0.08', None])
def test_auto_frame_rejects_invalid_margin(tmp_path, document, value):
    params = document['rs_follow_node']['ros__parameters']
    params['auto_frame'] = True
    params['self_occlusion_margin'] = value
    with pytest.raises(ValueError, match='self_occlusion_margin'):
        load_stack_config(save(tmp_path, document), 'twist', False)


@pytest.mark.parametrize('value', [0, 0.08])
def test_auto_frame_accepts_nonnegative_finite_margin(tmp_path, document, value):
    params = document['rs_follow_node']['ros__parameters']
    params['auto_frame'] = True
    params['self_occlusion_margin'] = value
    follow, _, _ = load_stack_config(save(tmp_path, document), 'twist', False)
    assert follow['auto_frame'] is True
    assert follow['self_occlusion_margin'] == value


def test_loopback_profile_matches_approved_input_contract():
    follow, selected, _ = load_stack_config(str(CONFIG), 'm20', False)
    assert follow['input_topic'] == '/rslidar_points'
    assert follow['control_frame'] == 'rslidar'
    assert follow['odom_topic'] == selected['odom_topic'] == '/m20/odom'
