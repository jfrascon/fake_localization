"""Test the public launch interface of fake_localization."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_exposes_the_current_node_arguments_contract() -> None:
    """Require explicit parameter handling and one JSON node-arguments object."""
    path = PACKAGE_ROOT / 'launch' / 'fake_localization.launch.py'
    spec = importlib.util.spec_from_file_location('fake_localization_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert 'node_args' in declarations
    assert 'params_file' in declarations
    assert 'params_file_allow_substs' in declarations
    assert 'use_sim_time' in declarations

    context = LaunchContext()
    declarations['node_args'].visit(context)
    assert context.launch_configurations['node_args'] == (
        '{"output":"both","ros_arguments":["--log-level","info"]}'
    )
