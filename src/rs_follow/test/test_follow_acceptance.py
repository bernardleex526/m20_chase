"""Acceptance dispatch checks; runtime scenarios still exercise the real node."""

import importlib.util
from pathlib import Path
import sys


SPEC = importlib.util.spec_from_file_location(
    'follow_acceptance_under_test',
    Path(__file__).resolve().parents[1] / 'scripts' / 'follow_acceptance.py')
acceptance = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(acceptance)


def test_all_runs_every_case_and_succeeds(monkeypatch):
    calls = []
    monkeypatch.setattr(sys, 'argv', ['follow_acceptance.py', '--case', 'all'])
    monkeypatch.setattr(acceptance, 'run_case', lambda case, domain: calls.append((case, domain)) or True)
    assert acceptance.main() == 0
    assert [case for case, _ in calls] == [
        'direct_timeout', 'binding', 'hard_stop', 'estop', 'cloud_loss', 'crossing', 'clock_pause']
    assert len({domain for _, domain in calls}) == len(calls)


def test_all_continues_after_case_failure(monkeypatch):
    calls = []

    def run(case, domain):
        calls.append(case)
        return case != 'crossing'

    monkeypatch.setattr(sys, 'argv', ['follow_acceptance.py', '--case', 'all'])
    monkeypatch.setattr(acceptance, 'run_case', run)
    assert acceptance.main() == 1
    assert calls[-2:] == ['crossing', 'clock_pause']


def test_new_cases_dispatch_without_unavailable_status(monkeypatch):
    for case in ('crossing', 'clock_pause'):
        calls = []
        monkeypatch.setattr(sys, 'argv', ['follow_acceptance.py', '--case', case])
        monkeypatch.setattr(acceptance, 'run_case', lambda name, domain: calls.append(name) or True)
        assert acceptance.main() == 0
        assert calls == [case]
