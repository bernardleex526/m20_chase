"""Validate explicit deployment facts before starting any stack process."""
import ipaddress
import math
from pathlib import Path

import yaml

COMMAND_TOPIC = '/rs_follow/cmd_vel'
ADAPTERS = {
    'm20': ('m20_bridge', 'bridge_node', 'm20_bridge'),
    'twist': ('dog_adapters', 'twist_adapter', 'twist_adapter'),
    'unitree_sport': ('dog_adapters', 'unitree_adapter', 'unitree_adapter'),
}
SAFE_FLAGS = ('active', 'auto_select_front', 'search_enable',
              'recovery_enable', 'compensate_slip')


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def _text(params, key, label):
    value = params.get(key)
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError(f'{label}.{key} must be explicitly supplied as a nonempty string')
    return value


def _number(params, key, positive=False):
    value = params.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'rs_follow_node.{key} must be explicitly supplied and finite')
    if positive and value <= 0:
        raise ValueError(f'rs_follow_node.{key} must be positive')
    return value


def _section(document, name):
    node = _mapping(document.get(name), name)
    return dict(_mapping(node.get('ros__parameters'), f'{name}.ros__parameters'))


def resolved_topic(topic, node_name):
    """Root-namespace resolution; runtime adapter also checks ROS remappings."""
    if topic.startswith('~/'):
        return '/' + node_name + topic[1:]
    return '/' + topic.lstrip('/')


def load_stack_config(path, adapter, with_web):
    if adapter not in ADAPTERS:
        raise ValueError('adapter must be m20, twist, or unitree_sport')
    if type(with_web) is not bool:
        raise ValueError('with_web must be a bool')
    if not path or not Path(path).is_file():
        raise ValueError('robot_config must name an existing YAML file')
    with Path(path).open(encoding='utf-8') as stream:
        document = _mapping(yaml.safe_load(stream), 'robot_config')
    follow = _section(document, 'rs_follow_node')
    for key in ('input_topic', 'control_frame', 'odom_topic'):
        _text(follow, key, 'rs_follow_node')
    for key in ('robot_length', 'robot_width', 'frame_front', 'frame_back',
                'frame_left', 'frame_right'):
        _number(follow, key, positive=True)
    if type(follow.get('auto_frame')) is not bool:
        raise ValueError('rs_follow_node.auto_frame must be explicitly supplied as bool')
    if follow['auto_frame'] and _number(follow, 'self_occlusion_margin') < 0:
        raise ValueError('rs_follow_node.self_occlusion_margin must be nonnegative')
    for low, high in (('height_min', 'height_max'), ('low_height_min', 'low_height_max')):
        if _number(follow, low) >= _number(follow, high):
            raise ValueError(f'rs_follow_node.{low} must be below {high}')
    if type(follow.get('enable_low_band')) is not bool:
        raise ValueError('rs_follow_node.enable_low_band must be explicitly supplied as bool')
    deployment = _mapping(document.get('deployment'), 'deployment')
    if type(deployment.get('nonhardware')) is not bool:
        raise ValueError('deployment.nonhardware must be explicitly supplied as bool')
    source = _text(deployment, 'tf_source', 'deployment')
    if source == 'loopback_identity':
        if not deployment['nonhardware']:
            raise ValueError('loopback_identity TF is permitted only for explicit nonhardware deployments')
    elif source.lower() in ('identity', 'none', 'unknown', 'todo'):
        raise ValueError('deployment.tf_source must identify a real measured TF publisher/source')
    for key in SAFE_FLAGS:
        follow[key] = False
    follow['cmd_vel_topic'] = COMMAND_TOPIC
    _, _, name = ADAPTERS[adapter]
    selected = _section(document, name)
    selected['cmd_vel_topic'] = COMMAND_TOPIC
    if adapter == 'twist':
        output = _text(selected, 'output_topic', name)
        if resolved_topic(output, name) == COMMAND_TOPIC:
            raise ValueError('Twist input and output topics must resolve to distinct topics')
    elif adapter == 'unitree_sport':
        _text(selected, 'request_topic', name)
    elif deployment['nonhardware']:
        try:
            loopback = ipaddress.ip_address(selected.get('ip', '')).is_loopback
        except ValueError:
            loopback = selected.get('ip') == 'localhost'
        if not loopback:
            raise ValueError('nonhardware M20 deployments require an explicit loopback ip')
    web = _section(document, 'web_ui') if with_web else None
    if web is not None:
        web['cmd_vel_topic'] = COMMAND_TOPIC
    return follow, selected, web
