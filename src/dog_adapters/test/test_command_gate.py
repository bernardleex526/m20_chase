"""Pure contract tests: no ROS or fabricated robot SDK involved."""
import math

import pytest

from dog_adapters.command_gate import CommandGate

ZERO = (0.0, 0.0, 0.0)
CMD = (0.2, 0.1, 0.4)


def gate(lateral=True):
    return CommandGate(0.3, 0.3, 0.15, 0.5, lateral)


@pytest.mark.parametrize('values', [
    (0, .3, .15, .5), (-1, .3, .15, .5),
    (.3, -.3, .15, .5), (.3, .3, -.15, .5), (.3, .3, .15, -.5),
    (math.nan, .3, .15, .5), (math.inf, .3, .15, .5),
    (.3, math.nan, .15, .5), (.3, .3, math.inf, .5),
    (.3, .3, .15, -math.inf),
])
def test_invalid_configuration(values):
    with pytest.raises(ValueError):
        CommandGate(*values, True)


def test_initial_and_exact_timeout():
    g = gate()
    assert g.output(0, True) == ZERO
    assert g.accept(*CMD, 1)
    assert g.output(1.299, True) == CMD
    assert g.output(1.3, True) == ZERO
    assert not g.fresh(1.3)
    assert g.output(1.4, True) == ZERO


@pytest.mark.parametrize('sign', [-1, 1])
def test_clamps_each_axis(sign):
    g = gate()
    assert g.accept(99 * sign, 99 * sign, 99 * sign, 0)
    assert g.output(0, True) == (.3 * sign, .15 * sign, .5 * sign)


def test_lateral_disabled():
    g = gate(False)
    assert g.accept(*CMD, 0)
    assert g.output(0, True) == (.2, 0, .4)


def test_zero_limits_and_zero_command_are_fresh():
    g = CommandGate(.3, 0, 0, 0, True)
    assert g.accept(*CMD, 0)
    assert g.fresh(.1)
    assert g.output(.1, True) == ZERO


@pytest.mark.parametrize('index', range(4))
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_input_clears_cache(index, bad):
    g = gate()
    assert g.accept(*CMD, 0)
    values = [*CMD, .1]
    values[index] = bad
    assert not g.accept(*values)
    assert g.output(.2, True) == ZERO
    assert g.accept(*CMD, .3)
    assert g.output(.3, True) == CMD


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_output_time_clears_cache(bad):
    g = gate()
    assert g.accept(*CMD, 0)
    assert g.output(bad, True) == ZERO
    assert g.output(.1, True) == ZERO


def test_regressing_accept_clears_cache_and_cannot_replay():
    g = gate()
    assert g.accept(*CMD, 10)
    assert not g.accept(*CMD, 9)
    assert g.output(10.1, True) == ZERO
    assert g.accept(*CMD, 10.1)
    assert g.output(10.1, True) == CMD


def test_regressing_output_clears_cache():
    g = gate()
    assert g.accept(*CMD, 10)
    assert g.output(9, True) == ZERO
    assert g.output(10, True) == ZERO


def test_same_monotonic_timestamp_is_valid():
    g = gate()
    assert g.accept(*CMD, 1)
    assert g.accept(.1, 0, 0, 1)
    assert g.output(1, True) == (.1, 0, 0)


def test_readiness_loss_requires_new_command():
    g = gate()
    assert g.accept(*CMD, 0)
    assert g.output(.01, False) == ZERO
    assert g.output(.02, True) == ZERO
    assert g.accept(*CMD, .03)
    assert g.output(.03, True) == CMD


def test_estop_unlock_requires_new_command():
    g = gate()
    assert g.accept(*CMD, 0)
    g.set_estop(True)
    assert g.output(.01, True) == ZERO
    assert not g.accept(*CMD, .02)
    g.set_estop(False)
    assert g.output(.03, True) == ZERO
    assert g.accept(*CMD, .04)
    assert g.output(.04, True) == CMD


def test_repeated_unlocked_status_does_not_drop_live_command():
    g = gate()
    assert g.accept(*CMD, 0)
    g.set_estop(False)
    assert g.output(.1, True) == CMD


def test_watchdog_refresh_replaces_command():
    g = gate()
    assert g.accept(*CMD, 0)
    assert g.accept(.1, 0, -.1, .2)
    assert g.output(.4, True) == (.1, 0, -.1)
    assert g.output(.5, True) == ZERO


def test_timeout_is_permanent_even_if_clock_regresses():
    g = gate()
    assert g.accept(*CMD, 0)
    assert not g.fresh(.3)
    assert g.output(.1, True) == ZERO


def test_fresh_check_regression_invalidates():
    g = gate()
    assert g.accept(*CMD, 10)
    assert not g.fresh(9)
    assert g.output(10, True) == ZERO
