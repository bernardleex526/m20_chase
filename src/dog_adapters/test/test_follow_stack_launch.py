"""Launch construction tests; actual process/topic checks belong to stack smoke."""
import importlib.util
from pathlib import Path

import pytest

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from dog_adapters.stack_config import ADAPTERS, COMMAND_TOPIC, SAFE_FLAGS

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location('follow_stack_launch', ROOT / 'launch' / 'follow_stack.launch.py')
STACK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STACK)


def context(adapter='twist', with_web='false', path=None):
    ctx = LaunchContext()
    ctx.launch_configurations.update(adapter=adapter, with_web=with_web,
                                     robot_config=str(path or ROOT / 'config' / 'loopback.yaml'))
    return ctx


@pytest.mark.parametrize('adapter', ADAPTERS)
@pytest.mark.parametrize('with_web', ['false', 'true'])
def test_exactly_one_adapter_and_shared_parameters(monkeypatch, adapter, with_web):
    monkeypatch.setattr(STACK, 'Node', lambda **kwargs: kwargs)
    packages = []
    monkeypatch.setattr(STACK, 'get_package_share_directory', lambda name: packages.append(name))
    nodes = STACK.launch_stack(context(adapter, with_web))
    expected = ['rs_follow_node', ADAPTERS[adapter][2]]
    if with_web == 'true':
        expected.append('web_ui')
    assert [node['name'] for node in nodes] == expected
    assert nodes[1]['package'] == ADAPTERS[adapter][0]
    assert nodes[1]['executable'] == ADAPTERS[adapter][1]
    assert all(node['parameters'][0]['cmd_vel_topic'] == COMMAND_TOPIC for node in nodes)
    assert all(nodes[0]['parameters'][0][flag] is False for flag in SAFE_FLAGS)
    assert packages == (['m20_bridge'] if adapter == 'm20' else [])


def test_missing_m20_is_clear_launch_error(monkeypatch):
    def absent(name):
        raise PackageNotFoundError(name)
    monkeypatch.setattr(STACK, 'get_package_share_directory', absent)
    with pytest.raises(RuntimeError, match='optional m20_bridge package'):
        STACK.launch_stack(context('m20'))


@pytest.mark.parametrize('value', ['', 'yes', '1', 'invalid'])
def test_invalid_web_boolean(value):
    with pytest.raises(ValueError, match='with_web must be true or false'):
        STACK.launch_stack(context(with_web=value))


def test_invalid_adapter_precedes_process_creation(monkeypatch):
    monkeypatch.setattr(STACK, 'Node', lambda **kwargs: pytest.fail('process action created'))
    with pytest.raises(ValueError, match='adapter must be'):
        STACK.launch_stack(context(adapter='bad'))


def test_required_configuration_precedes_process_creation(monkeypatch, tmp_path):
    monkeypatch.setattr(STACK, 'Node', lambda **kwargs: pytest.fail('process action created'))
    with pytest.raises(ValueError, match='existing YAML'):
        STACK.launch_stack(context(path=tmp_path / 'absent.yaml'))


def test_launch_arguments_and_deferred_validation():
    entities = STACK.generate_launch_description().entities
    arguments = {entity.name: entity for entity in entities if isinstance(entity, DeclareLaunchArgument)}
    assert set(arguments) == {'adapter', 'robot_config', 'with_web'}
    assert arguments['robot_config'].default_value is None
    assert arguments['adapter'].default_value is None
    assert arguments['adapter'].choices == list(ADAPTERS)
    assert arguments['with_web'].choices == ['true', 'false']
    assert [value.text for value in arguments['with_web'].default_value] == ['true']
    assert sum(isinstance(entity, OpaqueFunction) for entity in entities) == 1
