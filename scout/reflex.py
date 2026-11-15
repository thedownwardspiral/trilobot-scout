"""
The reflex layer: local safety rules that decide whether a move may start or
must end, whatever asked for it. Nothing the model says can change them.

StopSignal is the one flag that halts the robot. Button A raises it now; the
web stop button and the gamepad will raise the same signal later. ForwardReflex
holds the distance rules for driving forward, using should_avoid() from
robot_logic for the missing-echo handling.
"""

import threading

from scout.robot_logic import is_valid_distance, should_avoid

STOP_BUTTON = 0  # trilobot.BUTTON_A


class StopSignal:
    """A thread-safe stop flag. Once raised it stays raised until cleared, so
    one press stops a whole run, not just the current move."""

    def __init__(self):
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._reason = None

    def raise_stop(self, reason="stop requested"):
        with self._lock:
            if not self._event.is_set():
                self._reason = reason
            self._event.set()

    def clear(self):
        with self._lock:
            self._reason = None
            self._event.clear()

    def is_raised(self):
        return self._event.is_set()

    @property
    def reason(self):
        """Why the signal was first raised, or None while it is clear."""
        return self._reason


class ForwardReflex:
    """Decides from distance readings whether the path ahead is blocked.

    The limit is threshold_cm plus margin_cm: the margin covers the distance
    the robot travels between one reading and the next, so a move cut at the
    limit still ends at or before threshold_cm. A missing echo counts as
    blocked only when the last valid reading was within caution_cm.
    """

    def __init__(self, threshold_cm, caution_cm, margin_cm=0.0):
        self.threshold_cm = threshold_cm
        self.caution_cm = caution_cm
        self.margin_cm = margin_cm
        self.last_valid_cm = None

    @property
    def limit_cm(self):
        return self.threshold_cm + self.margin_cm

    def blocks(self, reading):
        """Whether this reading means the robot must not drive forward. Also
        remembers the reading if it is a valid one."""
        blocked = should_avoid(reading, self.last_valid_cm, self.limit_cm, self.caution_cm)
        if is_valid_distance(reading):
            self.last_valid_cm = reading
        return blocked

    def forget(self):
        """Drop the remembered reading, for example after a turn, when it no
        longer describes what is ahead."""
        self.last_valid_cm = None
