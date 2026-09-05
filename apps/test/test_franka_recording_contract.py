"""Event-free Franka episodes are valid; missing continuous evidence is not."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'franka_episode_validate', ROOT / 'src/recording/episode_recorder/scripts/episode_validate.py')
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


@pytest.mark.parametrize('variant', ['mujoco', 'real', 'no_cam'])
def test_authority_events_may_be_empty_but_captured_events_must_be_stored(variant):
    streams = yaml.safe_load((ROOT / f'apps/recording/franka_manipulation_{variant}.yaml').read_text())['streams']
    streams = {stream['id']: stream for stream in streams}
    events = streams['authority_events']
    metrics = {'message_count': 0, 'type': None, 'header_non_monotonic': 0}
    assert validator.evaluate_base_stream(metrics, events, {}, {})['status'] == 'PASS'
    # Optional does not disable accounting: a received event missing from MCAP fails.
    capture = {'received': 1, 'enqueued': 1, 'written': 1, 'recorder_drops': 0}
    assert validator.evaluate_base_stream(metrics, events, {}, capture)['status'] == 'FAIL'
    metrics.update(message_count=1, type=events['expected_type'])
    assert validator.evaluate_base_stream(metrics, events, {}, capture)['status'] == 'PASS'
    for name in ('authority_status', 'robot_joint_states'):
        assert streams[name]['required'] and streams[name]['start_gate']
        assert validator.evaluate_base_stream(
            {'message_count': 0, 'type': None, 'header_non_monotonic': 0},
            streams[name], {}, {})['status'] == 'FAIL'
