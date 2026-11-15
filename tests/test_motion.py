"""Milestone 2 behaviour, tested only through Mover.drive() and Mover.turn()
on a FakeTrilobot in simulated time."""

import itertools
import math
import unittest

from scout.fake_robot import BUTTON_A, NO_READING, FakeTrilobot
from scout.motion import Mover, Outcome, Settings
from scout.reflex import StopSignal

SLACK_CM = 1e-6


def make(settings=None, obstacle_cm=None, read_time_s=None, **fake_kwargs):
    """A mover over a fake robot whose wheel speed and turn rate match the
    settings, so requested and simulated motion agree."""
    settings = settings or Settings()
    wheel_cm_s = settings.cm_per_s  # each wheel's ground speed at settings.speed
    # Spinning on the spot, the wheels run in opposite directions: omega = 2v / track
    track_cm = 2 * wheel_cm_s / math.radians(settings.deg_per_s)
    bot = FakeTrilobot(
        obstacle_cm=obstacle_cm,
        max_speed_cm_s=wheel_cm_s / settings.speed,
        track_width_cm=track_cm,
        read_time_s=settings.sensor_read_s if read_time_s is None else read_time_s,
        **fake_kwargs,
    )
    return bot, Mover(bot, settings, sleep=bot.sleep, now=bot.now)


def call_names(bot):
    return [call[1] for call in bot.calls]


class TestDrive(unittest.TestCase):
    def test_drive_in_open_space_runs_for_distance_over_rate(self):
        bot, mover = make()
        result = mover.drive(20)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertEqual((result.kind, result.requested, result.used), ('drive', 20, 20))
        self.assertFalse(result.clamped)
        self.assertAlmostEqual(result.seconds, 20 / 25.0)
        self.assertAlmostEqual(bot.distance_travelled_cm, 20)
        self.assertEqual(bot.last_motor_call(), 'stop')

    def test_drive_uses_the_fixed_speed(self):
        bot, mover = make(Settings(speed=0.4, cm_per_s=20))
        mover.drive(10)
        first_motor_call = bot.motor_calls()[0]
        self.assertEqual(first_motor_call[1:3], ('forward', (0.4,)))

    def test_requests_beyond_30_cm_are_clamped(self):
        bot, mover = make()
        result = mover.drive(80)
        self.assertTrue(result.clamped)
        self.assertEqual(result.used, 30)
        self.assertAlmostEqual(bot.distance_travelled_cm, 30)

        bot, mover = make()
        result = mover.drive(-80)
        self.assertEqual(result.used, -30)
        self.assertAlmostEqual(bot.distance_travelled_cm, -30)

    def test_zero_does_nothing(self):
        bot, mover = make()
        result = mover.drive(0)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertEqual(result.seconds, 0)
        self.assertEqual(bot.motor_calls(), [])

    def test_backward_drive_ignores_the_sensor(self):
        bot, mover = make(obstacle_cm=10)
        result = mover.drive(-20)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertAlmostEqual(bot.distance_travelled_cm, -20)
        self.assertNotIn('read_distance', call_names(bot))
        self.assertIsNone(result.distance_cm)
        self.assertEqual(bot.last_motor_call(), 'stop')


class TestForwardVeto(unittest.TestCase):
    def test_refused_at_the_threshold_without_touching_the_motors(self):
        for distance in (5, 25, 30):
            with self.subTest(distance=distance):
                bot, mover = make(obstacle_cm=distance)
                result = mover.drive(20)
                self.assertEqual(result.outcome, Outcome.REFUSED)
                self.assertEqual(result.distance_cm, distance)
                self.assertEqual(result.seconds, 0)
                self.assertEqual(bot.motor_calls(), [])
                self.assertIn('path blocked', result.reason)

    def test_refused_within_the_stopping_margin(self):
        settings = Settings(stopping_margin_cm=3)
        bot, mover = make(settings, obstacle_cm=33)
        self.assertEqual(mover.drive(10).outcome, Outcome.REFUSED)

    def test_allowed_just_beyond_the_limit(self):
        settings = Settings(stopping_margin_cm=3)
        bot, mover = make(settings, obstacle_cm=33.5)
        result = mover.drive(10)
        self.assertNotEqual(result.outcome, Outcome.REFUSED)
        self.assertGreaterEqual(bot.obstacle_cm, 30 - SLACK_CM)

    def test_default_margin_covers_one_poll_and_one_read(self):
        settings = Settings(cm_per_s=20, poll_interval_s=0.05, sensor_read_s=0.03)
        self.assertAlmostEqual(settings.margin_cm, 1.6)


class TestDistanceWatch(unittest.TestCase):
    def test_drive_towards_a_wall_is_cut_short_before_30_cm(self):
        bot, mover = make(obstacle_cm=45)
        result = mover.drive(30)
        self.assertEqual(result.outcome, Outcome.CUT_SHORT)
        self.assertGreaterEqual(bot.obstacle_cm, 30 - SLACK_CM)
        self.assertFalse(bot.collided)
        self.assertLessEqual(result.distance_cm, mover.reflex.limit_cm)
        self.assertLess(result.seconds, 30 / 25.0)
        self.assertEqual(bot.last_motor_call(), 'stop')

    def test_a_full_move_that_ends_at_the_limit_counts_as_moved(self):
        # Reads at 0 and 0.4 s see 60 and 50 cm. The poll at 0.8 s sees 40 cm. The
        # last 0.4 s is one poll long, so it takes no read, and the move ends at 30 cm.
        settings = Settings(poll_interval_s=0.4, sensor_read_s=0.0, stopping_margin_cm=0)
        bot, mover = make(settings, obstacle_cm=60)
        result = mover.drive(30)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertAlmostEqual(result.distance_cm, 40)
        self.assertAlmostEqual(bot.obstacle_cm, 30)
        self.assertEqual(mover.drive(10).outcome, Outcome.REFUSED)

    def test_reads_never_run_the_motors_past_the_move_time(self):
        # A read blocks with the motors on, so a read at the end would add its time to the move
        for distance in (0.5, 1, 2, 3.3, 7, 20, 29.9, 30):
            with self.subTest(distance=distance):
                bot, mover = make(obstacle_cm=300)
                result = mover.drive(distance)
                self.assertEqual(result.outcome, Outcome.MOVED)
                self.assertAlmostEqual(result.seconds, distance / 25.0)
                self.assertAlmostEqual(bot.distance_travelled_cm, distance)

    def test_something_appearing_mid_move_cuts_it_short(self):
        bot, mover = make()
        bot.queue_distances(200.0, 150.0, 20.0)  # start check, then two polls
        result = mover.drive(30)
        self.assertEqual(result.outcome, Outcome.CUT_SHORT)
        self.assertEqual(result.distance_cm, 20.0)
        self.assertLess(bot.distance_travelled_cm, 5)

    def test_never_closer_than_30_cm_from_any_start_speed_or_timing(self):
        # The wall test in simulation: hop at a wall until it stops us
        cases = itertools.product(
            (35, 47.3, 80, 150, 300),  # start distance, cm
            (0.3, 0.5, 1.0),  # speed
            (0.02, 0.05, 0.1),  # poll interval, s
            (0.0, 0.03),  # sensor read time, s
        )
        for start, speed, poll, read in cases:
            with self.subTest(start=start, speed=speed, poll=poll, read=read):
                settings = Settings(speed=speed, cm_per_s=50 * speed, poll_interval_s=poll,
                                    sensor_read_s=read)
                bot, mover = make(settings, obstacle_cm=start)
                for _ in range(30):
                    result = mover.drive(30)
                    if result.outcome is not Outcome.MOVED:
                        break
                self.assertIn(result.outcome, (Outcome.CUT_SHORT, Outcome.REFUSED))
                self.assertGreaterEqual(bot.obstacle_cm, 30 - SLACK_CM)
                self.assertFalse(bot.collided)


class TestMissingEcho(unittest.TestCase):
    def test_no_echo_in_open_space_keeps_going(self):
        bot, mover = make()
        self.assertEqual(bot.read_distance(), NO_READING)
        result = mover.drive(20)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertEqual(result.distance_cm, NO_READING)

    def test_no_echo_after_a_close_reading_is_blocked_across_moves(self):
        bot, mover = make()
        bot.queue_distances(55.0, 54.0)
        mover.drive(3)  # one read mid-move: sees 55 at the start, then 54; remembers 54
        self.assertEqual(mover.reflex.last_valid_cm, 54.0)
        bot.queue_distances(NO_READING)
        result = mover.drive(20)
        self.assertEqual(result.outcome, Outcome.REFUSED)
        self.assertIn('no echo', result.reason)

    def test_no_echo_mid_move_after_a_close_reading_cuts_it_short(self):
        bot, mover = make()
        bot.queue_distances(50.0, NO_READING)
        result = mover.drive(20)
        self.assertEqual(result.outcome, Outcome.CUT_SHORT)

    def test_a_turn_forgets_the_last_reading(self):
        bot, mover = make()
        bot.queue_distances(50.0)
        mover.drive(1)
        mover.turn(90)
        bot.queue_distances(NO_READING)
        self.assertEqual(mover.drive(10).outcome, Outcome.MOVED)


class TestTurn(unittest.TestCase):
    def test_positive_is_right_and_negative_is_left(self):
        bot, mover = make()
        result = mover.turn(90)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertAlmostEqual(bot.heading_deg, 270)  # clockwise
        self.assertAlmostEqual(result.seconds, 90 / 138.0)

        bot, mover = make()
        mover.turn(-45)
        self.assertAlmostEqual(bot.heading_deg, 45)

    def test_turns_are_clamped_to_90_degrees(self):
        bot, mover = make()
        result = mover.turn(-200)
        self.assertEqual(result.used, -90)
        self.assertTrue(result.clamped)
        self.assertAlmostEqual(bot.heading_deg, 90)

    def test_turns_do_not_read_the_sensor(self):
        bot, mover = make(obstacle_cm=5)
        result = mover.turn(30)
        self.assertEqual(result.outcome, Outcome.MOVED)
        self.assertNotIn('read_distance', call_names(bot))
        self.assertEqual(bot.last_motor_call(), 'stop')

    def test_zero_does_nothing(self):
        bot, mover = make()
        mover.turn(0)
        self.assertEqual(bot.motor_calls(), [])


class TestStopSignal(unittest.TestCase):
    def test_button_a_halts_a_drive_mid_move(self):
        bot, mover = make()
        bot.at(0.3, lambda: bot.press_button(BUTTON_A))
        result = mover.drive(30)
        self.assertEqual(result.outcome, Outcome.STOPPED)
        self.assertIn('Button A', result.reason)
        self.assertTrue(mover.stop_signal.is_raised())
        # Seen within one poll interval of the press
        self.assertLessEqual(result.seconds, 0.3 + Settings().poll_interval_s + Settings().sensor_read_s)
        self.assertLess(bot.distance_travelled_cm, 30)
        self.assertEqual(bot.last_motor_call(), 'stop')

    def test_button_a_halts_a_turn_mid_move(self):
        bot, mover = make()
        bot.at(0.2, lambda: bot.press_button(BUTTON_A))
        result = mover.turn(90)
        self.assertEqual(result.outcome, Outcome.STOPPED)
        self.assertLess(result.seconds, 90 / 138.0)
        self.assertEqual(bot.last_motor_call(), 'stop')

    def test_button_a_halts_a_backward_drive(self):
        bot, mover = make()
        bot.at(0.2, lambda: bot.press_button(BUTTON_A))
        self.assertEqual(mover.drive(-30).outcome, Outcome.STOPPED)

    def test_button_held_before_a_move_stops_it_without_moving(self):
        bot, mover = make()
        bot.press_button(BUTTON_A)
        for result in (mover.drive(20), mover.turn(45)):
            self.assertEqual(result.outcome, Outcome.STOPPED)
            self.assertEqual(result.seconds, 0)
        self.assertEqual(bot.motor_calls(), [])

    def test_stop_during_the_first_distance_read_keeps_the_motors_off(self):
        for raise_stop in (lambda bot, mover: bot.press_button(BUTTON_A),
                           lambda bot, mover: mover.stop_signal.raise_stop("web stop button")):
            with self.subTest():
                bot, mover = make(obstacle_cm=200)  # each read takes 0.03 s
                bot.at(0.01, lambda: raise_stop(bot, mover))
                result = mover.drive(20)
                self.assertEqual(result.outcome, Outcome.STOPPED)
                self.assertEqual(result.seconds, 0)
                self.assertEqual(result.distance_cm, 200)
                self.assertEqual(bot.motor_calls(), [])

    def test_signal_stays_raised_until_cleared(self):
        bot, mover = make()
        bot.at(0.1, lambda: bot.press_button(BUTTON_A))
        bot.at(0.2, lambda: bot.release_button(BUTTON_A))
        mover.drive(30)
        bot.sleep(1)
        self.assertFalse(bot.read_button(BUTTON_A))
        self.assertEqual(mover.drive(10).outcome, Outcome.STOPPED)
        mover.stop_signal.clear()
        self.assertEqual(mover.drive(10).outcome, Outcome.MOVED)

    def test_signal_raised_from_elsewhere_halts_a_move(self):
        signal = StopSignal()
        bot = FakeTrilobot(max_speed_cm_s=50)
        mover = Mover(bot, Settings(), stop_signal=signal, sleep=bot.sleep, now=bot.now)
        bot.at(0.4, lambda: signal.raise_stop("web stop button"))
        result = mover.drive(30)
        self.assertEqual(result.outcome, Outcome.STOPPED)
        self.assertEqual(result.reason, "stopped: web stop button")

    def test_first_reason_is_kept(self):
        signal = StopSignal()
        signal.raise_stop("first")
        signal.raise_stop("second")
        self.assertEqual(signal.reason, "first")
        signal.clear()
        self.assertIsNone(signal.reason)
        self.assertFalse(signal.is_raised())


class TestFailures(unittest.TestCase):
    def test_sensor_failure_mid_drive_stops_the_motors_and_raises(self):
        bot, mover = make()

        def broken_sensor(*args, **kwargs):
            raise RuntimeError("sensor unplugged")

        bot.at(0.2, lambda: setattr(bot, 'read_distance', broken_sensor))
        with self.assertRaises(RuntimeError):
            mover.drive(30)
        self.assertEqual(bot.last_motor_call(), 'stop')
        self.assertEqual((bot.left_speed, bot.right_speed), (0.0, 0.0))

    def test_failure_mid_turn_stops_the_motors_and_raises(self):
        bot = FakeTrilobot()
        sleeps = itertools.count()

        def failing_sleep(seconds):
            if next(sleeps) == 2:
                raise KeyboardInterrupt
            bot.sleep(seconds)

        mover = Mover(bot, Settings(), sleep=failing_sleep, now=bot.now)
        with self.assertRaises(KeyboardInterrupt):
            mover.turn(90)
        self.assertEqual(bot.last_motor_call(), 'stop')
        bot.sleep(1)
        self.assertEqual(bot.distance_travelled_cm, 0)


if __name__ == '__main__':
    unittest.main()
