import inspect
import math
import unittest

from scout import fake_robot
from scout.fake_robot import (
    BUTTON_A, BUTTON_Y, LIGHT_FRONT_LEFT, LIGHT_REAR_RIGHT, LIGHTS_LEFT, MOTOR_LEFT, MOTOR_RIGHT,
    NO_READING, FakeTrilobot,
)

try:
    import trilobot
except ImportError:  # not installed off the Pi, or no RPi.GPIO
    trilobot = None


class TestMotors(unittest.TestCase):
    def setUp(self):
        self.bot = FakeTrilobot()

    def test_helpers_set_speeds_like_the_real_class(self):
        cases = [
            ('forward', (0.5, 0.5)), ('backward', (-0.5, -0.5)),
            ('turn_left', (-0.5, 0.5)), ('turn_right', (0.5, -0.5)),
            ('curve_forward_left', (0.0, 0.5)), ('curve_forward_right', (0.5, 0.0)),
            ('curve_backward_left', (0.0, -0.5)), ('curve_backward_right', (-0.5, 0.0)),
        ]
        for name, expected in cases:
            with self.subTest(name):
                getattr(self.bot, name)(0.5)
                self.assertEqual((self.bot.left_speed, self.bot.right_speed), expected)

    def test_default_speed_is_full(self):
        self.bot.forward()
        self.assertEqual((self.bot.left_speed, self.bot.right_speed), (1.0, 1.0))

    def test_speeds_are_clamped_not_rejected(self):
        self.bot.set_motor_speeds(2.5, -7)
        self.assertEqual((self.bot.left_speed, self.bot.right_speed), (1.0, -1.0))

    def test_single_motor_setters(self):
        self.bot.set_left_speed(0.3)
        self.bot.set_right_speed(-0.2)
        self.bot.set_motor_speed(MOTOR_LEFT, 0.1)
        self.assertEqual((self.bot.left_speed, self.bot.right_speed), (0.1, -0.2))

    def test_bad_motor_raises_like_the_real_class(self):
        with self.assertRaises(TypeError):
            self.bot.set_motor_speed(0.0, 0.5)
        with self.assertRaises(ValueError):
            self.bot.set_motor_speed(2, 0.5)

    def test_stop_and_coast_zero_the_speeds(self):
        self.bot.forward()
        self.bot.stop()
        self.assertEqual((self.bot.left_speed, self.bot.right_speed), (0.0, 0.0))
        self.assertTrue(self.bot.motors_enabled)
        self.bot.forward()
        self.bot.coast()
        self.assertEqual((self.bot.left_speed, self.bot.right_speed), (0.0, 0.0))
        self.assertFalse(self.bot.motors_enabled)


class TestCallLog(unittest.TestCase):
    def test_only_outermost_calls_are_logged(self):
        bot = FakeTrilobot()
        bot.forward(0.5)
        bot.stop()
        self.assertEqual([call[1] for call in bot.calls], ['forward', 'stop'])

    def test_last_motor_call_ignores_sensor_and_light_calls(self):
        bot = FakeTrilobot()
        bot.forward()
        bot.stop()
        bot.read_distance()
        bot.fill_underlighting(255, 0, 0)
        self.assertEqual(bot.last_motor_call(), 'stop')

    def test_calls_record_sim_time_and_arguments(self):
        bot = FakeTrilobot()
        bot.sleep(1.5)
        bot.forward(speed=0.4)
        self.assertEqual(bot.calls[-1], (1.5, 'forward', (), {'speed': 0.4}))

    def test_no_motor_calls(self):
        self.assertIsNone(FakeTrilobot().last_motor_call())

    def test_calls_that_raise_are_not_logged(self):
        bot = FakeTrilobot()
        bot.stop()
        with self.assertRaises(ValueError):
            bot.set_motor_speed(2, 0.5)
        with self.assertRaises(TypeError):
            bot.forward('fast')
        self.assertEqual(bot.last_motor_call(), 'stop')
        self.assertEqual([call[1] for call in bot.calls], ['stop'])

    def test_calls_are_stamped_when_they_finish(self):
        bot = FakeTrilobot(read_time_s=0.02)
        bot.read_distance()
        self.assertAlmostEqual(bot.calls[-1][0], 0.02)
        self.assertAlmostEqual(bot.calls[-1][0], bot.now())


class TestSimulation(unittest.TestCase):
    def test_clock_only_moves_on_sleep(self):
        bot = FakeTrilobot()
        self.assertEqual(bot.now(), 0.0)
        bot.forward()
        self.assertEqual(bot.now(), 0.0)
        self.assertEqual(bot.distance_travelled_cm, 0.0)
        bot.sleep(0.25)
        bot.sleep(0.25)
        self.assertAlmostEqual(bot.now(), 0.5)

    def test_negative_sleep_is_rejected(self):
        with self.assertRaises(ValueError):
            FakeTrilobot().sleep(-1)

    def test_straight_drive(self):
        bot = FakeTrilobot(max_speed_cm_s=40)
        bot.forward(0.5)
        bot.sleep(2)
        self.assertAlmostEqual(bot.distance_travelled_cm, 40)
        self.assertAlmostEqual(bot.x_cm, 40)
        self.assertAlmostEqual(bot.y_cm, 0)
        self.assertAlmostEqual(bot.heading_deg, 0)

    def test_reverse_counts_as_negative_travel(self):
        bot = FakeTrilobot(max_speed_cm_s=40, obstacle_cm=100)
        bot.backward(0.5)
        bot.sleep(1)
        self.assertAlmostEqual(bot.distance_travelled_cm, -20)
        self.assertAlmostEqual(bot.obstacle_cm, 120)

    def test_spin_in_place_changes_heading_only(self):
        # Wheels 0.5 * 20 = 10 cm/s in opposite directions on a 10 cm track: 2 rad/s
        bot = FakeTrilobot(max_speed_cm_s=20, track_width_cm=10, obstacle_cm=50)
        bot.turn_left(0.5)
        bot.sleep(math.pi / 4)  # a quarter turn
        self.assertAlmostEqual(bot.heading_deg, 90)
        self.assertAlmostEqual(bot.distance_travelled_cm, 0)
        self.assertAlmostEqual(bot.x_cm, 0)
        self.assertEqual(bot.obstacle_cm, 50)

    def test_turn_right_is_clockwise(self):
        bot = FakeTrilobot(max_speed_cm_s=20, track_width_cm=10)
        bot.turn_right(0.5)
        bot.sleep(math.pi / 4)
        self.assertAlmostEqual(bot.heading_deg, 270)

    def test_curve_follows_an_arc(self):
        # Left wheel still, right at 10 cm/s on a 10 cm track: a 5 cm radius circle at 1 rad/s
        bot = FakeTrilobot(max_speed_cm_s=10, track_width_cm=10)
        bot.curve_forward_left(1.0)
        bot.sleep(math.pi / 2)
        self.assertAlmostEqual(bot.heading_deg, 90)
        self.assertAlmostEqual(bot.x_cm, 5)
        self.assertAlmostEqual(bot.y_cm, 5)

    def test_stopped_robot_stays_put(self):
        bot = FakeTrilobot()
        bot.forward()
        bot.stop()
        bot.sleep(10)
        self.assertEqual(bot.distance_travelled_cm, 0)

    def test_coasting_robot_stays_put(self):
        bot = FakeTrilobot()
        bot.forward()
        bot.coast()
        bot.sleep(10)
        self.assertEqual(bot.distance_travelled_cm, 0)

    def test_driving_closes_the_obstacle_distance(self):
        bot = FakeTrilobot(obstacle_cm=100, max_speed_cm_s=50)
        bot.forward(0.5)
        bot.sleep(1)
        self.assertAlmostEqual(bot.read_distance(), 75)
        self.assertFalse(bot.collided)

    def test_driving_into_the_obstacle_stops_there(self):
        bot = FakeTrilobot(obstacle_cm=30, max_speed_cm_s=50)
        bot.forward()
        bot.sleep(5)
        self.assertTrue(bot.collided)
        self.assertEqual(bot.obstacle_cm, 0)
        self.assertAlmostEqual(bot.distance_travelled_cm, 30)
        self.assertAlmostEqual(bot.now(), 5)
        self.assertEqual(bot.read_distance(), NO_READING)  # too close for the sensor

    def test_drive_at_wall_and_stop_without_real_sleeping(self):
        # The shape of a milestone 2 reflex test: poll, and stop when close
        bot = FakeTrilobot(obstacle_cm=200, max_speed_cm_s=50, read_time_s=0.02)
        bot.forward(0.6)
        while bot.now() < 30:
            distance = bot.read_distance()
            if 0 <= distance <= 25:
                bot.stop()
                break
            bot.sleep(0.05)
        self.assertEqual(bot.last_motor_call(), 'stop')
        self.assertFalse(bot.collided)
        self.assertGreater(bot.obstacle_cm, 20)


class TestDistanceSensor(unittest.TestCase):
    def test_nothing_ahead_reads_no_reading(self):
        self.assertEqual(FakeTrilobot().read_distance(), NO_READING)
        self.assertEqual(NO_READING, -999)

    def test_out_of_range_reads_no_reading(self):
        bot = FakeTrilobot(obstacle_cm=500, max_range_cm=400)
        self.assertEqual(bot.read_distance(), NO_READING)
        bot.set_obstacle(399.5)
        self.assertEqual(bot.read_distance(), 399.5)

    def test_inside_the_blind_zone_reads_no_reading(self):
        bot = FakeTrilobot(obstacle_cm=1.5, min_range_cm=2.0)
        self.assertEqual(bot.read_distance(), NO_READING)
        bot.set_obstacle(2.0)
        self.assertEqual(bot.read_distance(), 2.0)

    def test_reading_is_a_float(self):
        self.assertIsInstance(FakeTrilobot(obstacle_cm=42).read_distance(), float)

    def test_queued_readings_come_first_in_order(self):
        bot = FakeTrilobot(obstacle_cm=80)
        bot.queue_distances(10.0, -999, 5000.0)
        self.assertEqual([bot.read_distance() for _ in range(4)], [10.0, -999, 5000.0, 80.0])

    def test_real_keyword_arguments_are_accepted(self):
        bot = FakeTrilobot(obstacle_cm=60)
        self.assertEqual(bot.read_distance(timeout=25, samples=1, offset=0), 60.0)

    def test_zero_samples_or_timeout_takes_no_measurement(self):
        bot = FakeTrilobot(obstacle_cm=60, read_time_s=0.03)
        bot.queue_distances(10.0)
        self.assertEqual(bot.read_distance(samples=0), NO_READING)
        self.assertEqual(bot.read_distance(timeout=0), NO_READING)
        self.assertEqual(bot.read_distance(timeout=-5), NO_READING)
        self.assertEqual(bot.now(), 0.0)
        self.assertEqual(bot.read_distance(), 10.0)  # the queued reading was not used up

    def test_timeout_limits_the_range(self):
        # 17.15 cm of range per ms of timeout
        bot = FakeTrilobot(obstacle_cm=100, max_range_cm=400)
        self.assertEqual(bot.read_distance(timeout=6), 100.0)  # 102.9 cm of range: just enough
        self.assertEqual(bot.read_distance(timeout=5), NO_READING)  # 85.75 cm of range
        self.assertEqual(bot.read_distance(timeout=50), 100.0)

    def test_read_time_advances_the_clock(self):
        bot = FakeTrilobot(read_time_s=0.03)
        bot.read_distance()
        bot.read_distance()
        self.assertAlmostEqual(bot.now(), 0.06)


class TestButtons(unittest.TestCase):
    def test_press_and_release(self):
        bot = FakeTrilobot()
        self.assertFalse(bot.read_button(BUTTON_A))
        bot.press_button(BUTTON_A)
        self.assertTrue(bot.read_button(BUTTON_A))
        self.assertFalse(bot.read_button(BUTTON_Y))
        bot.release_button(BUTTON_A)
        self.assertFalse(bot.read_button(BUTTON_A))

    def test_bad_button_raises(self):
        bot = FakeTrilobot()
        with self.assertRaises(TypeError):
            bot.read_button('A')
        with self.assertRaises(ValueError):
            bot.read_button(4)
        with self.assertRaises(ValueError):
            bot.press_button(-1)

    def test_button_leds(self):
        bot = FakeTrilobot()
        bot.set_button_led(BUTTON_A, True)
        bot.set_button_led(BUTTON_Y, 0.25)
        self.assertEqual(bot.button_leds, [1.0, 0.0, 0.0, 0.25])
        bot.set_button_led(BUTTON_A, False)
        self.assertEqual(bot.button_leds[BUTTON_A], 0.0)

    def test_bad_button_led_raises(self):
        bot = FakeTrilobot()
        with self.assertRaises(ValueError):
            bot.set_button_led(BUTTON_A, 1.5)
        with self.assertRaises(ValueError):
            bot.set_button_led(4, True)
        with self.assertRaises(TypeError):
            bot.set_button_led(None, True)


class TestUnderlights(unittest.TestCase):
    def setUp(self):
        self.bot = FakeTrilobot()

    def test_colour_forms(self):
        self.bot.set_underlight(0, 1, 2, 3)
        self.bot.set_underlight(1, (4, 5, 6))
        self.bot.set_underlight(2, [7, 8, 9])
        self.bot.set_underlight(3, '#ff8000')
        self.bot.set_underlight(4, '00ff10')
        self.assertEqual([self.bot.underlight_color(i) for i in range(5)],
                         [(1, 2, 3), (4, 5, 6), (7, 8, 9), (255, 128, 0), (0, 255, 16)])

    def test_show_false_stages_without_showing(self):
        self.bot.fill_underlighting(255, 0, 0, show=False)
        self.assertEqual(self.bot.underlight_color(LIGHT_FRONT_LEFT), (0, 0, 0))
        self.assertEqual(self.bot.underlight_color(LIGHT_FRONT_LEFT, shown=False), (255, 0, 0))
        self.bot.show_underlighting()
        self.assertEqual(self.bot.underlight_color(LIGHT_FRONT_LEFT), (255, 0, 0))

    def test_groups_and_clearing(self):
        self.bot.fill_underlighting(0, 0, 255)
        self.bot.set_underlights(LIGHTS_LEFT, (0, 255, 0))
        self.bot.clear_underlight(LIGHT_REAR_RIGHT)
        self.assertEqual([self.bot.underlight_color(i) for i in range(6)],
                         [(0, 0, 255), (0, 255, 0), (0, 255, 0), (0, 255, 0), (0, 0, 0), (0, 0, 255)])
        self.bot.clear_underlighting()
        self.assertEqual(self.bot.shown_underlight, [0] * 18)

    def test_hsv(self):
        self.bot.fill_underlighting_hsv(0.0)
        self.assertEqual(self.bot.underlight_color(0), (255, 0, 0))
        self.bot.set_underlight_hsv(1, 1 / 3)
        self.assertEqual(self.bot.underlight_color(1), (0, 255, 0))

    def test_disable_keeps_colours(self):
        self.bot.fill_underlighting(9, 9, 9)
        self.bot.disable_underlighting()
        self.assertFalse(self.bot.underlighting_enabled)
        self.assertEqual(self.bot.underlight_color(0), (9, 9, 9))

    def test_bad_values_raise(self):
        with self.assertRaises(ValueError):
            self.bot.set_underlight(0, 256, 0, 0)
        with self.assertRaises(ValueError):
            self.bot.set_underlight(0, 0, 0)
        with self.assertRaises(ValueError):
            self.bot.set_underlight(0, (1, 2))
        with self.assertRaises(ValueError):
            self.bot.set_underlight(6, 0, 0, 0)
        with self.assertRaises(TypeError):
            self.bot.set_underlight('0', 0, 0, 0)
        with self.assertRaises(ValueError):
            self.bot.set_underlights([], 0, 0, 0)
        with self.assertRaises(TypeError):
            self.bot.set_underlights(0, 0, 0, 0)


def _public_methods(cls):
    return {name: inspect.signature(member)
            for name, member in inspect.getmembers(cls, inspect.isfunction)
            if not name.startswith('_')}


@unittest.skipIf(trilobot is None, "trilobot library not importable")
class TestParityWithRealTrilobot(unittest.TestCase):
    """Compares against the real class without constructing it (construction touches hardware)."""

    # Real methods the fake leaves out on purpose: Scout's robot has no servo
    NOT_FAKED = {
        'initialise_servo', 'disable_servo', 'set_servo_value', 'set_servo_angle',
        'servo_to_center', 'servo_to_min', 'servo_to_max', 'servo_to_percent',
    }

    def setUp(self):
        self.real = _public_methods(trilobot.Trilobot)
        self.fake = _public_methods(FakeTrilobot)

    def test_shared_methods_have_identical_signatures(self):
        for name in self.fake.keys() & self.real.keys():
            with self.subTest(name):
                self.assertEqual(self.fake[name], self.real[name])

    def test_every_real_method_is_faked_or_knowingly_skipped(self):
        self.assertEqual(self.real.keys() - self.fake.keys(), self.NOT_FAKED)

    def test_fake_only_methods_are_the_declared_simulation_helpers(self):
        self.assertEqual(self.fake.keys() - self.real.keys(), FakeTrilobot.SIMULATION_METHODS)

    def test_constants_match(self):
        names = [name for name in dir(trilobot)
                 if name.isupper() and isinstance(getattr(trilobot, name), (int, tuple))]
        self.assertTrue(names)
        for name in names:
            with self.subTest(name):
                self.assertEqual(getattr(fake_robot, name), getattr(trilobot, name))


if __name__ == '__main__':
    unittest.main()
