import unittest

from scout.robot_logic import is_valid_distance, mix_differential, next_color_state, should_avoid


class TestIsValidDistance(unittest.TestCase):
    def test_no_echo_is_invalid(self):
        self.assertFalse(is_valid_distance(-999))

    def test_zero_and_positive_are_valid(self):
        self.assertTrue(is_valid_distance(0))
        self.assertTrue(is_valid_distance(42.5))


class TestMixDifferential(unittest.TestCase):
    def test_centered_stick_is_stopped(self):
        self.assertEqual(mix_differential(0, 0), (0, 0))

    def test_straight_forward(self):
        self.assertEqual(mix_differential(0, 1), (1, 1))

    def test_spin_in_place(self):
        self.assertEqual(mix_differential(1, 0), (1, -1))

    def test_full_diagonal_still_turns(self):
        left, right = mix_differential(1, 1)
        self.assertEqual(left, 1)
        self.assertEqual(right, 0)

    def test_speeds_stay_in_range(self):
        for x in (-1, -0.5, 0, 0.5, 1):
            for y in (-1, -0.5, 0, 0.5, 1):
                left, right = mix_differential(x, y, max_speed=0.8)
                self.assertLessEqual(abs(left), 0.8)
                self.assertLessEqual(abs(right), 0.8)


class TestNextColorState(unittest.TestCase):
    def test_green_holds_inside_hysteresis_band(self):
        self.assertEqual(next_color_state('green', 97), 'green')

    def test_green_to_yellow(self):
        self.assertEqual(next_color_state('green', 95), 'yellow')

    def test_yellow_needs_to_clear_upper_band_for_green(self):
        self.assertEqual(next_color_state('yellow', 104), 'yellow')
        self.assertEqual(next_color_state('yellow', 106), 'green')

    def test_yellow_to_red(self):
        self.assertEqual(next_color_state('yellow', 25), 'red')

    def test_red_holds_inside_hysteresis_band(self):
        self.assertEqual(next_color_state('red', 33), 'red')
        self.assertEqual(next_color_state('red', 36), 'yellow')

    def test_green_jumps_straight_to_red(self):
        self.assertEqual(next_color_state('green', 10), 'red')


class TestShouldAvoid(unittest.TestCase):
    def test_valid_reading_within_threshold(self):
        self.assertTrue(should_avoid(30, None, threshold=30, caution_distance=60))

    def test_valid_reading_beyond_threshold(self):
        self.assertFalse(should_avoid(31, 20, threshold=30, caution_distance=60))

    def test_no_echo_in_open_space_keeps_going(self):
        self.assertFalse(should_avoid(-999, 200, threshold=30, caution_distance=60))
        self.assertFalse(should_avoid(-999, None, threshold=30, caution_distance=60))

    def test_no_echo_after_close_reading_avoids(self):
        self.assertTrue(should_avoid(-999, 45, threshold=30, caution_distance=60))


if __name__ == '__main__':
    unittest.main()
