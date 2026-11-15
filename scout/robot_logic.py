"""
Hardware-free logic shared by the Trilobot scripts, kept separate so it can be
unit tested without a Raspberry Pi.
"""

# Distance thresholds (in cm) for the controller's LED colors
THRESHOLD_FAR = 100
THRESHOLD_NEAR = 30
HYSTERESIS = 5  # Buffer zone around thresholds to prevent flickering

STATE_COLORS = {
    'green': (0, 255, 0),
    'yellow': (255, 255, 0),
    'red': (255, 0, 0),
}


def is_valid_distance(distance):
    """Trilobot.read_distance() returns -999 when no echo arrives before the timeout
    (object out of range, or a soft/angled surface), so only non-negative values are real."""
    return distance >= 0


def mix_differential(x, y, max_speed=1.0):
    """Convert stick x (turn) and y (forward) in [-1, 1] into (left, right) motor speeds.

    Speeds are scaled down together rather than clipped, so a full diagonal
    push still turns instead of saturating both motors.
    """
    left = y + x
    right = y - x
    scale = max(1.0, abs(left), abs(right))
    return left / scale * max_speed, right / scale * max_speed


def next_color_state(state, distance):
    """Return the new LED color state ('green', 'yellow' or 'red') for a distance in cm.

    To change from green to yellow the distance must go below 95cm, and from
    yellow back to green it must go above 105cm (likewise around 30cm for red).
    """
    if state == 'green':
        if distance <= THRESHOLD_NEAR - HYSTERESIS:
            return 'red'
        if distance <= THRESHOLD_FAR - HYSTERESIS:
            return 'yellow'
    elif state == 'yellow':
        if distance <= THRESHOLD_NEAR - HYSTERESIS:
            return 'red'
        if distance > THRESHOLD_FAR + HYSTERESIS:
            return 'green'
    elif state == 'red':
        if distance > THRESHOLD_FAR + HYSTERESIS:
            return 'green'
        if distance > THRESHOLD_NEAR + HYSTERESIS:
            return 'yellow'
    return state


def should_avoid(distance, last_valid_distance, threshold, caution_distance):
    """Decide whether the navigation script should run its avoidance maneuver.

    A valid reading avoids when within the threshold. A missing echo usually means
    nothing is in range, so it is only treated as an obstacle when the last valid
    reading was already close (the echo may have been lost on a near, angled surface).
    """
    if is_valid_distance(distance):
        return distance <= threshold
    return last_valid_distance is not None and last_valid_distance <= caution_distance
