"""ROS-independent, monotonic command safety contract shared by robot adapters."""
import math

STOP_ACTIONS = frozenset({'softstop', 'soft_stop', 'estop', 'e_stop', 'stop'})
CLEAR_ACTIONS = frozenset({'clear', 'clear_estop', 'unlock', 'reset_estop', 'resume'})


class CommandGate:
    def __init__(self, timeout_s, max_vx, max_vy, max_wz, lateral):
        values = (timeout_s, max_vx, max_vy, max_wz)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('CommandGate limits and timeout must be finite')
        if timeout_s <= 0 or min(max_vx, max_vy, max_wz) < 0:
            raise ValueError('CommandGate needs positive timeout and nonnegative limits')
        self.timeout_s = timeout_s
        self.limits = (max_vx, max_vy, max_wz)
        self.lateral = lateral
        self.estopped = False
        self._command = None
        self._accepted_at = None
        self._last_now = None

    def _clear(self):
        self._command = None
        self._accepted_at = None

    def _observe(self, now_s):
        if not math.isfinite(now_s) or (
                self._last_now is not None and now_s < self._last_now):
            self._clear()
            return False
        self._last_now = now_s
        return True

    def accept(self, vx, vy, wz, now_s):
        if not self._observe(now_s) or not all(math.isfinite(v) for v in (vx, vy, wz)):
            self._clear()
            return False
        if self.estopped:
            self._clear()
            return False
        self._command = tuple(max(-limit, min(limit, value))
                              for value, limit in zip((vx, vy, wz), self.limits))
        if not self.lateral:
            self._command = (self._command[0], 0.0, self._command[2])
        self._accepted_at = now_s
        return True

    def set_estop(self, enabled):
        enabled = bool(enabled)
        if enabled or self.estopped:
            # Both stop and unlock transitions invalidate cached motion.
            self._clear()
        self.estopped = enabled

    def output(self, now_s, ready):
        if (not self._observe(now_s) or not ready or self.estopped
                or self._accepted_at is None
                or now_s - self._accepted_at >= self.timeout_s):
            self._clear()
            return (0.0, 0.0, 0.0)
        return self._command

    def fresh(self, now_s):
        """Check freshness independently of velocity (a zero command is valid)."""
        if (not self._observe(now_s) or self.estopped or self._accepted_at is None
                or now_s - self._accepted_at >= self.timeout_s):
            self._clear()
            return False
        return True
