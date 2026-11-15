"""
Moves as short, timed motor runs, each guarded by the reflex layer.

The Mover is the one entry point: drive(distance_cm) and turn(degrees). With
no wheel encoders, a move is a motor run for distance / rate seconds at one
fixed slow speed, so the distances and angles are requests, not measurements.
Every move ends with stop(), however it ends, and returns a MoveResult.

The robot is anything with the trilobot.Trilobot API, such as FakeTrilobot in
tests. sleep and now default to real time; tests pass the fake's own.
"""

import dataclasses
import enum
import time

from scout.reflex import STOP_BUTTON, ForwardReflex, StopSignal
from scout.robot_logic import is_valid_distance

TIME_EPSILON_S = 1e-9


@dataclasses.dataclass(frozen=True)
class Settings:
    """Speeds, rates and safety limits. The rates are placeholders until
    calibration (milestone 3) measures them on the robot's floor."""

    speed: float = 0.5  # the one motor speed used for every move, 0..1
    cm_per_s: float = 25.0  # drive rate at that speed; placeholder
    deg_per_s: float = 138.0  # turn rate at that speed; from the old repo's 90 degrees in 0.65 s
    max_drive_cm: float = 30.0
    max_turn_deg: float = 90.0
    threshold_cm: float = 30.0  # never end a forward drive closer than this
    caution_cm: float = 60.0  # a lost echo after a reading this close counts as blocked
    poll_interval_s: float = 0.05  # how often a move checks the stop signal and sensor
    sensor_read_s: float = 0.03  # roughly how long one read_distance() takes
    stopping_margin_cm: float | None = None  # None: what the robot covers between checks

    @property
    def margin_cm(self):
        """Extra distance before the threshold at which a forward drive is
        refused or cut, so that it ends at or before threshold_cm."""
        if self.stopping_margin_cm is not None:
            return self.stopping_margin_cm
        return self.cm_per_s * (self.poll_interval_s + self.sensor_read_s)


class Outcome(str, enum.Enum):
    MOVED = 'moved'  # ran for the full time
    CUT_SHORT = 'cut_short'  # a forward drive stopped early for an obstacle
    REFUSED = 'refused'  # a forward drive not started because the path is blocked
    STOPPED = 'stopped'  # ended or not started because the stop signal is raised


@dataclasses.dataclass(frozen=True)
class MoveResult:
    kind: str  # 'drive' or 'turn'
    outcome: Outcome
    requested: float  # cm or degrees, as asked
    used: float  # after clamping
    seconds: float  # how long the motors ran
    distance_cm: float | None  # last distance reading taken, if any (-999 for no echo)
    reason: str

    @property
    def clamped(self):
        return self.used != self.requested


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class Mover:
    def __init__(self, robot, settings=None, stop_signal=None, sleep=time.sleep, now=time.monotonic):
        self.robot = robot
        self.settings = settings or Settings()
        self.stop_signal = stop_signal or StopSignal()
        self.sleep = sleep
        self.now = now
        self.reflex = ForwardReflex(self.settings.threshold_cm, self.settings.caution_cm,
                                    self.settings.margin_cm)

    def drive(self, distance_cm):
        """Drive distance_cm forward, or backward if negative, clamped to max_drive_cm.
        Forward drives are refused or cut short when the path ahead is blocked."""
        used = _clamp(distance_cm, self.settings.max_drive_cm)
        result = self._result_maker('drive', distance_cm, used)
        if self._stop_requested():
            return result(Outcome.STOPPED, 0.0, None, f"not started: {self.stop_signal.reason}")
        if used == 0:
            return result(Outcome.MOVED, 0.0, None, "nothing to do")

        reading = None
        if used > 0:
            reading = self.robot.read_distance()
            if self.reflex.blocks(reading):
                return result(Outcome.REFUSED, 0.0, reading, self._blocked_reason(reading))
            # The read takes time; a stop raised meanwhile must still keep the motors off
            if self._stop_requested():
                return result(Outcome.STOPPED, 0.0, reading, f"not started: {self.stop_signal.reason}")
            start_motors = self.robot.forward
        else:
            start_motors = self.robot.backward

        outcome, seconds, reading = self._run(start_motors, abs(used) / self.settings.cm_per_s,
                                              watch_ahead=used > 0, reading=reading)
        return result(outcome, seconds, reading, self._run_reason(outcome, reading, 'drove', used, 'cm'))

    def turn(self, degrees):
        """Turn on the spot, right for positive degrees and left for negative,
        clamped to max_turn_deg. Turns are not distance-checked: the sensor
        faces forward."""
        used = _clamp(degrees, self.settings.max_turn_deg)
        result = self._result_maker('turn', degrees, used)
        if self._stop_requested():
            return result(Outcome.STOPPED, 0.0, None, f"not started: {self.stop_signal.reason}")
        if used == 0:
            return result(Outcome.MOVED, 0.0, None, "nothing to do")

        start_motors = self.robot.turn_right if used > 0 else self.robot.turn_left
        try:
            outcome, seconds, _ = self._run(start_motors, abs(used) / self.settings.deg_per_s,
                                            watch_ahead=False, reading=None)
        finally:
            # The robot now faces somewhere else, so the old reading no longer applies
            self.reflex.forget()
        return result(outcome, seconds, None, self._run_reason(outcome, None, 'turned', used, 'degrees'))

    def _run(self, start_motors, duration_s, watch_ahead, reading):
        """Run the motors for duration_s, checking the stop signal (and the
        path ahead, when watch_ahead) every poll interval. Always ends with
        stop(); an exception stops the motors and then propagates.

        A distance read blocks with the motors on, so the last stretch of a
        watched run, once no more than one poll and one read are left, still
        checks the stop signal but takes no reads. The run then ends on time,
        and that stretch is no longer than the gap between two reads, which
        the stopping margin already covers."""
        poll_s = self.settings.poll_interval_s
        last_stretch_s = poll_s + self.settings.sensor_read_s
        outcome = Outcome.MOVED
        began = self.now()
        try:
            start_motors(self.settings.speed)
            while True:
                remaining = duration_s - (self.now() - began)
                if remaining <= TIME_EPSILON_S:
                    break
                read_after = watch_ahead and remaining > last_stretch_s
                self.sleep(min(poll_s, remaining))
                if self._stop_requested():
                    outcome = Outcome.STOPPED
                    break
                if read_after:
                    reading = self.robot.read_distance()
                    blocked = self.reflex.blocks(reading)
                    # A move that has already run its full time counts as moved,
                    # even if it ended at the limit
                    if blocked and duration_s - (self.now() - began) > TIME_EPSILON_S:
                        outcome = Outcome.CUT_SHORT
                        break
        finally:
            self.robot.stop()
        return outcome, self.now() - began, reading

    def _stop_requested(self):
        if self.robot.read_button(STOP_BUTTON):
            self.stop_signal.raise_stop("Button A pressed")
        return self.stop_signal.is_raised()

    @staticmethod
    def _result_maker(kind, requested, used):
        def make(outcome, seconds, distance_cm, reason):
            return MoveResult(kind, outcome, requested, used, seconds, distance_cm, reason)
        return make

    def _blocked_reason(self, reading):
        if is_valid_distance(reading):
            return f"path blocked: {reading:.0f} cm ahead, limit {self.reflex.limit_cm:.0f} cm"
        return f"no echo, and the last reading was {self.reflex.last_valid_cm:.0f} cm"

    def _run_reason(self, outcome, reading, verb, used, unit):
        if outcome is Outcome.STOPPED:
            return f"stopped: {self.stop_signal.reason}"
        if outcome is Outcome.CUT_SHORT:
            return f"cut short: {self._blocked_reason(reading)}"
        return f"{verb} {used:g} {unit}"
