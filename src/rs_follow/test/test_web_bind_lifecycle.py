"""Unit regressions for web request lifecycle (not ROS binding acceptance)."""
import ast
from collections import deque
from concurrent.futures import Future
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

source = ast.parse((Path(__file__).resolve().parents[1] / 'scripts/web_ui.py').read_text())
web = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'WebUi')
methods = [n for n in web.body if isinstance(n, ast.FunctionDef) and n.name in ('_invalidate', '_drain', '_execute')]
class Harness:
    pass
namespace = dict(time=time, Bool=lambda **kw: NS(**kw))
for method in methods:
    exec(compile(ast.Module(body=[method], type_ignores=[]), 'web_ui.py', 'exec'), namespace)
    setattr(Harness, method.name, namespace[method.name])


def harness():
    h = Harness()
    h.lock = threading.RLock()
    h.actions = deque()
    h.generation = 0
    h.pending = h.cancelled_bind = None
    h.binding_blocked = False
    h.bind_reason = ''
    h.target = [3, 0, 0]
    h.clears = []
    h.pauses = []
    h.clear_pub = NS(publish=h.clears.append)
    h._pause = lambda: h.pauses.append(True)
    h.control_received = time.monotonic()
    h._state = lambda: dict(mode=1, target_valid=True, estop=False, cloud_age_ms=0)
    return h


def test_pending_binding_cannot_enable_previous_target():
    h = harness()
    h.pending = (Future(), time.monotonic() + 1, None, 0)
    result = h._execute(dict(type='enable', value=True), None)
    assert result == dict(success=False, reason='NOT_READY')


def test_cancel_retains_future_then_clears_after_late_commit():
    h = harness()
    future = Future()
    h.pending = (future, time.monotonic() + 1, None, 0)
    h._invalidate()
    assert h.cancelled_bind is future and not future.cancelled()
    assert h.binding_blocked and h.pending is None
    assert h._execute(dict(type='set_target'), None)['reason'] == 'BIND_PENDING'
    future.set_result(NS(success=True, reason='OK'))
    clears_before = len(h.clears)
    h._drain()
    assert len(h.clears) == clears_before + 1
    assert h.cancelled_bind is None and h.binding_blocked
    assert h.pauses


def test_deadline_wins_even_when_future_already_done():
    h = harness()
    future = Future()
    future.set_result(NS(success=True, reason='OK', target=NS(point=NS(x=9, y=0, z=0))))
    h.pending = (future, time.monotonic() - 0.01, None, 0)
    h._drain()
    assert h.bind_reason == 'TIMEOUT'
    assert h.target == [3, 0, 0]
    assert h.binding_blocked and h.clears and h.pauses


def test_new_request_invalidates_old_web_target_before_service_check():
    h = harness()
    h.bind_client = NS(service_is_ready=lambda: False)
    result = h._execute(dict(type='set_target'), None)
    assert result['reason'] == 'SERVICE_UNAVAILABLE'
    assert h.binding_blocked and h.target is None
