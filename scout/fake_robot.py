"""
A hardware-free stand-in for trilobot.Trilobot (library version 0.0.2).

FakeTrilobot has the same public methods, signatures, validation and constants
as the real class for everything Scout uses (motors, ultrasonic sensor, buttons,
button LEDs, underlights), so code written against it runs unchanged on the
robot. The servo methods are left out: Scout's robot has no servo.

It also runs a small simulation with its own clock. Nothing moves until
sleep() is called: sleep(seconds) advances the simulated clock and integrates
the motion produced by the current motor speeds, and now() returns the
simulated time. Code that drives the robot should take its sleep and clock
functions as parameters, so tests can pass fake.sleep and fake.now.

The obstacle model is one-dimensional: `obstacle_cm` is the distance to
whatever is straight ahead of the sensor (None for nothing). Driving forward or
backward changes it; turning in place does not. Tests that turn towards or
away from something set it again with set_obstacle(). Driving into the
obstacle stops the robot at 0 cm and sets `collided`; the sensor cannot see
that close, so read_distance() then returns NO_READING, as for a lost echo.
"""

import functools
import heapq
import itertools
import math
from colorsys import hsv_to_rgb

# Constants, with the same names and values as the trilobot module
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
NUM_BUTTONS = 4

LIGHT_FRONT_RIGHT = 0
LIGHT_FRONT_LEFT = 1
LIGHT_MIDDLE_LEFT = 2
LIGHT_REAR_LEFT = 3
LIGHT_REAR_RIGHT = 4
LIGHT_MIDDLE_RIGHT = 5
NUM_UNDERLIGHTS = 6

LIGHTS_LEFT = (LIGHT_FRONT_LEFT, LIGHT_MIDDLE_LEFT, LIGHT_REAR_LEFT)
LIGHTS_RIGHT = (LIGHT_FRONT_RIGHT, LIGHT_MIDDLE_RIGHT, LIGHT_REAR_RIGHT)
LIGHTS_FRONT = (LIGHT_FRONT_LEFT, LIGHT_FRONT_RIGHT)
LIGHTS_MIDDLE = (LIGHT_MIDDLE_LEFT, LIGHT_MIDDLE_RIGHT)
LIGHTS_REAR = (LIGHT_REAR_LEFT, LIGHT_REAR_RIGHT)
LIGHTS_LEFT_DIAGONAL = (LIGHT_FRONT_LEFT, LIGHT_REAR_RIGHT)
LIGHTS_RIGHT_DIAGONAL = (LIGHT_FRONT_RIGHT, LIGHT_REAR_LEFT)

MOTOR_LEFT = 0
MOTOR_RIGHT = 1
NUM_MOTORS = 2

NO_READING = -999  # what read_distance() returns when no echo arrives

# The real read_distance() discards echoes slower than its timeout. Sound
# travels 0.0343 cm/us and the echo covers the distance twice, so each ms of
# timeout allows 17.15 cm of range.
TIMEOUT_RANGE_CM_PER_MS = 343 * 100 / 1000 / 2

MOTOR_METHODS = frozenset({
    'set_motor_speed', 'set_motor_speeds', 'set_left_speed', 'set_right_speed',
    'forward', 'backward', 'turn_left', 'turn_right',
    'curve_forward_left', 'curve_forward_right',
    'curve_backward_left', 'curve_backward_right',
    'stop', 'coast', 'disable_motors',
})


def _logged(method):
    """Record a call in FakeTrilobot.calls once it returns, stamped with the
    simulated time it finished. Calls that raise are not recorded, and only
    the outermost call is, so stop() is logged as 'stop' and not also as the
    set_motor_speeds() it calls."""
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        outermost = self._call_depth == 0
        self._call_depth += 1
        try:
            result = method(self, *args, **kwargs)
        finally:
            self._call_depth -= 1
        if outermost:
            self.calls.append((self._time, method.__name__, args, kwargs))
        return result
    return wrapper


class FakeTrilobot:
    """Simulated Trilobot. Construct it with the simulation settings you need;
    the defaults describe a robot in open space with nothing in range."""

    # Simulation-only methods that the real Trilobot does not have
    SIMULATION_METHODS = frozenset({
        'sleep', 'now', 'at', 'set_obstacle', 'queue_distances',
        'press_button', 'release_button', 'motor_calls', 'last_motor_call',
        'underlight_color',
    })

    def __init__(self, obstacle_cm=None, max_speed_cm_s=50.0, track_width_cm=12.0,
                 min_range_cm=2.0, max_range_cm=400.0, read_time_s=0.0):
        """
        obstacle_cm: distance to the obstacle ahead, or None for nothing ahead
        max_speed_cm_s: ground speed of a wheel at motor speed 1.0
        track_width_cm: distance between the wheels, which sets the turn rate
        min_range_cm: closer than this (the sensor's blind zone, about 2 cm for
            an HC-SR04), read_distance() returns NO_READING, as it does after a collision
        max_range_cm: beyond this, read_distance() returns NO_READING
        read_time_s: simulated time each read_distance() call takes
        """
        self.max_speed_cm_s = max_speed_cm_s
        self.track_width_cm = track_width_cm
        self.min_range_cm = min_range_cm
        self.max_range_cm = max_range_cm
        self.read_time_s = read_time_s

        self._time = 0.0
        self._scheduled = []  # heap of (time, order, action)
        self._schedule_order = itertools.count()
        self._call_depth = 0
        self.calls = []  # (finish time, method name, args, kwargs), outermost calls that returned

        # Motors: speeds as the real class would set them, after clamping
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.motors_enabled = False

        # Pose and travel since construction
        self.x_cm = 0.0
        self.y_cm = 0.0
        self.heading_deg = 0.0  # counterclockwise positive, 0 = starting direction
        self.distance_travelled_cm = 0.0  # net forward travel, negative when reversing
        self.obstacle_cm = obstacle_cm
        self.collided = False

        self._distance_queue = []

        self.buttons_pressed = [False] * NUM_BUTTONS
        self.button_leds = [0.0] * NUM_BUTTONS  # brightness 0.0..1.0

        # Like the real class: colours are staged by set_* and appear on show
        self.underlight = [0] * (NUM_UNDERLIGHTS * 3)
        self.shown_underlight = [0] * (NUM_UNDERLIGHTS * 3)
        self.underlighting_enabled = False

    # ---- Simulation controls (not on the real class) ----

    def now(self):
        """Simulated time in seconds since construction."""
        return self._time

    def sleep(self, seconds):
        """Advance the simulated clock, moving the robot at the current motor speeds.

        Actions scheduled with at() run when the clock reaches their time, in
        time order, and the motion after each one uses the speeds it left.
        """
        if seconds < 0:
            raise ValueError("sleep length must be non-negative")
        end = self._time + seconds
        while self._scheduled and self._scheduled[0][0] <= end:
            when, _, action = heapq.heappop(self._scheduled)
            self._advance_to(when)
            action()
        self._advance_to(end)

    def at(self, time_s, action):
        """Run action() when the simulated clock reaches time_s, for example
        bot.at(0.4, lambda: bot.press_button(BUTTON_A)). A time already passed
        runs on the next sleep()."""
        heapq.heappush(self._scheduled, (time_s, next(self._schedule_order), action))

    def _advance_to(self, time_s):
        if time_s > self._time:
            self._integrate(time_s - self._time)
            self._time = time_s

    def set_obstacle(self, distance_cm):
        """Put an obstacle distance_cm ahead of the sensor, or None for nothing."""
        self.obstacle_cm = distance_cm

    def queue_distances(self, *readings):
        """Make the next read_distance() calls return these values, in order,
        before falling back to the simulated distance."""
        self._distance_queue.extend(readings)

    def press_button(self, button):
        self._check_button(button)
        self.buttons_pressed[button] = True

    def release_button(self, button):
        self._check_button(button)
        self.buttons_pressed[button] = False

    def motor_calls(self):
        """The logged calls to motor methods, as (time, name, args, kwargs)."""
        return [call for call in self.calls if call[1] in MOTOR_METHODS]

    def last_motor_call(self):
        """Name of the most recent motor method called, or None."""
        calls = self.motor_calls()
        return calls[-1][1] if calls else None

    def _integrate(self, dt):
        left = self.left_speed * self.max_speed_cm_s if self.motors_enabled else 0.0
        right = self.right_speed * self.max_speed_cm_s if self.motors_enabled else 0.0
        v = (left + right) / 2  # cm/s along the heading
        omega = (right - left) / self.track_width_cm  # rad/s, counterclockwise

        travel = v * dt
        if travel > 0 and self.obstacle_cm is not None and travel >= self.obstacle_cm:
            # Stop at the obstacle, and only spend the time needed to get there turning
            dt = self.obstacle_cm / v
            travel = self.obstacle_cm
            self.collided = True

        theta = math.radians(self.heading_deg)
        if omega == 0:
            self.x_cm += travel * math.cos(theta)
            self.y_cm += travel * math.sin(theta)
        else:
            new_theta = theta + omega * dt
            radius = v / omega
            self.x_cm += radius * (math.sin(new_theta) - math.sin(theta))
            self.y_cm -= radius * (math.cos(new_theta) - math.cos(theta))
            self.heading_deg = (self.heading_deg + math.degrees(omega * dt)) % 360

        self.distance_travelled_cm += travel
        if self.obstacle_cm is not None:
            self.obstacle_cm = max(0.0, self.obstacle_cm - travel)

    @staticmethod
    def _check_button(button):
        if not isinstance(button, int):
            raise TypeError("button must be an integer")
        if button not in range(NUM_BUTTONS):
            raise ValueError("button must be an integer in the range 0 to 3")

    # ---- Motors (same API as trilobot.Trilobot) ----

    @_logged
    def set_motor_speed(self, motor, speed):
        if not isinstance(motor, int):
            raise TypeError("motor must be an integer")
        if motor not in range(NUM_MOTORS):
            raise ValueError("motor must be an integer in the range 0 to 1")
        speed = max(min(speed, 1.0), -1.0)
        self.motors_enabled = True
        if motor == MOTOR_LEFT:
            self.left_speed = float(speed)
        else:
            self.right_speed = float(speed)

    @_logged
    def set_motor_speeds(self, l_speed, r_speed):
        self.set_motor_speed(MOTOR_LEFT, l_speed)
        self.set_motor_speed(MOTOR_RIGHT, r_speed)

    @_logged
    def set_left_speed(self, speed):
        self.set_motor_speed(MOTOR_LEFT, speed)

    @_logged
    def set_right_speed(self, speed):
        self.set_motor_speed(MOTOR_RIGHT, speed)

    @_logged
    def disable_motors(self):
        self.motors_enabled = False
        self.left_speed = 0.0
        self.right_speed = 0.0

    @_logged
    def forward(self, speed=1.0):
        self.set_motor_speeds(speed, speed)

    @_logged
    def backward(self, speed=1.0):
        self.set_motor_speeds(-speed, -speed)

    @_logged
    def turn_left(self, speed=1.0):
        self.set_motor_speeds(-speed, speed)

    @_logged
    def turn_right(self, speed=1.0):
        self.set_motor_speeds(speed, -speed)

    @_logged
    def curve_forward_left(self, speed=1.0):
        self.set_motor_speeds(0.0, speed)

    @_logged
    def curve_forward_right(self, speed=1.0):
        self.set_motor_speeds(speed, 0.0)

    @_logged
    def curve_backward_left(self, speed=1.0):
        self.set_motor_speeds(0.0, -speed)

    @_logged
    def curve_backward_right(self, speed=1.0):
        self.set_motor_speeds(-speed, 0.0)

    @_logged
    def stop(self):
        self.set_motor_speeds(0.0, 0.0)

    @_logged
    def coast(self):
        # The real robot rolls on briefly; the simulation stops at once
        self.disable_motors()

    # ---- Ultrasonic sensor ----

    @_logged
    def read_distance(self, timeout=50, samples=3, offset=190000):
        """Distance ahead in cm, or NO_READING (-999) when nothing is in range.

        Like the real sensor, a timeout or sample count of zero or less takes
        no measurement, and an echo must return within `timeout` ms, which
        caps the range at timeout * TIMEOUT_RANGE_CM_PER_MS.
        """
        if samples <= 0 or timeout <= 0:
            return NO_READING
        if self.read_time_s:
            self.sleep(self.read_time_s)
        if self._distance_queue:
            return self._distance_queue.pop(0)
        max_range = min(self.max_range_cm, timeout * TIMEOUT_RANGE_CM_PER_MS)
        if self.obstacle_cm is None or self.obstacle_cm >= max_range \
                or self.obstacle_cm < self.min_range_cm:
            return NO_READING
        return float(self.obstacle_cm)

    # ---- Buttons and button LEDs ----

    @_logged
    def read_button(self, button):
        self._check_button(button)
        return self.buttons_pressed[button]

    @_logged
    def set_button_led(self, button_led, value):
        if not isinstance(button_led, int):
            raise TypeError("led must be an integer")
        if button_led not in range(NUM_BUTTONS):
            raise ValueError("button_led must be an integer in the range 0 to 3")
        if isinstance(value, bool):
            self.button_leds[button_led] = 1.0 if value else 0.0
        elif value < 0.0 or value > 1.0:
            raise ValueError("value must be in the range 0.0 to 1.0")
        else:
            self.button_leds[button_led] = float(value)

    # ---- Underlights ----

    def underlight_color(self, light, shown=True):
        """(r, g, b) of one underlight: as shown, or as staged when shown=False."""
        source = self.shown_underlight if shown else self.underlight
        return tuple(source[light * 3:light * 3 + 3])

    @_logged
    def set_underlight(self, light, r_color, g=None, b=None, show=True):
        if not isinstance(light, int):
            raise TypeError("light must be an integer")
        if light not in range(NUM_UNDERLIGHTS):
            raise ValueError("light must be an integer in the range 0 to 5")

        if g is None and b is None:
            if isinstance(r_color, str):
                value = r_color.strip('#')
                r_color = list(int(value[i:i + 2], 16) for i in (0, 2, 4))
            if not isinstance(r_color, (list, tuple)) or len(r_color) != 3 or \
                    any(c < 0 or c > 255 for c in r_color):
                raise ValueError("color must either be a color hex code, or a list/tuple of 3 numbers between 0 and 255")
            color = r_color
        else:
            if r_color < 0 or r_color > 255:
                raise ValueError("r must be in the range 0 to 255")
            if g is None or g < 0 or g > 255:
                raise ValueError("g must be in the range 0 to 255")
            if b is None or b < 0 or b > 255:
                raise ValueError("b must be in the range 0 to 255")
            color = (r_color, g, b)

        self.underlight[light * 3:light * 3 + 3] = [int(c) for c in color]
        if show:
            self.show_underlighting()

    @_logged
    def set_underlights(self, lights, r_color, g=None, b=None, show=True):
        if not isinstance(lights, (list, tuple)):
            raise TypeError("lights must be a list or tuple containing the numbers for the underlights to set (from 0 to 5)")
        if len(lights) > NUM_UNDERLIGHTS:
            raise ValueError("lights contains more values than the number of underlights available")
        if len(lights) == 0:
            raise ValueError("lights cannot be empty")
        for light in lights[:-1]:
            self.set_underlight(light, r_color, g, b, show=False)
        self.set_underlight(lights[-1], r_color, g, b, show=show)

    @_logged
    def fill_underlighting(self, r_color, g=None, b=None, show=True):
        for light in range(NUM_UNDERLIGHTS):
            self.set_underlight(light, r_color, g, b, show=False)
        if show:
            self.show_underlighting()

    @_logged
    def set_underlight_hsv(self, light, h, s=1, v=1, show=True):
        self.set_underlight(light, [c * 255 for c in hsv_to_rgb(h, s, v)], show=show)

    @_logged
    def set_underlights_hsv(self, lights, h, s=1, v=1, show=True):
        self.set_underlights(lights, [c * 255 for c in hsv_to_rgb(h, s, v)], show=show)

    @_logged
    def fill_underlighting_hsv(self, h, s=1, v=1, show=True):
        self.fill_underlighting([c * 255 for c in hsv_to_rgb(h, s, v)], show=show)

    @_logged
    def clear_underlight(self, light, show=True):
        self.set_underlight(light, 0, 0, 0, show=show)

    @_logged
    def clear_underlights(self, lights, show=True):
        self.set_underlights(lights, 0, 0, 0, show=show)

    @_logged
    def clear_underlighting(self, show=True):
        self.fill_underlighting(0, 0, 0, show=show)

    @_logged
    def show_underlighting(self):
        self.shown_underlight = list(self.underlight)
        self.underlighting_enabled = True

    @_logged
    def disable_underlighting(self):
        # Keeps the colours, like the real class; they show again on the next show
        self.underlighting_enabled = False
