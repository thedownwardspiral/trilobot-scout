"""
Milestone 2 acceptance check: drive at a wall and stop at or before 30 cm,
every time in 10 tries.

Each try drives forward in 30 cm hops until the reflex layer cuts a hop short
or refuses one, measures how far the wall is once the robot has settled, then
backs away for the next try. Press Button A at any time to halt and end the test.

    ~/.virtualenvs/pimoroni/bin/python scripts/wall_test.py
    ~/.virtualenvs/pimoroni/bin/python scripts/wall_test.py --fake 120   # rehearse in simulation
"""

import argparse
import statistics

from hardware import build, confirm, describe, positive

from scout.motion import Outcome
from scout.robot_logic import is_valid_distance

SETTLE_S = 0.5
IMPLAUSIBLE_CM = 400  # beyond the sensor's rated range: likely a lost echo
REFLEX_STOPS = (Outcome.CUT_SHORT, Outcome.REFUSED)


def plausible(reading):
    return is_valid_distance(reading) and reading <= IMPLAUSIBLE_CM


def settled_distance(mover, samples=5):
    """Median of the plausible readings once the robot has stopped rolling.
    Lost or implausible readings stay in the raw log but are left out here."""
    mover.sleep(SETTLE_S)
    readings = [mover.robot.read_distance() for _ in range(samples)]
    usable = [r for r in readings if plausible(r)]
    return statistics.median(usable) if usable else None


def one_try(mover, max_hops):
    """Hop at the wall until stopped. Returns the last move result."""
    for _ in range(max_hops):
        result = mover.drive(30)
        print(f"    {result.outcome.value:9} {result.used:5.1f} cm in {result.seconds:4.2f} s, "
              f"last reading {describe(result.distance_cm)}")
        if result.outcome is not Outcome.MOVED:
            return result
    return result


def back_off(mover, distance_cm):
    remaining = distance_cm
    while remaining > 0:
        result = mover.drive(-min(30, remaining))
        if result.outcome is not Outcome.MOVED:
            return result
        remaining -= 30
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--tries', type=positive(int), default=10)
    parser.add_argument('--back-off-cm', type=positive(float), default=60,
                        help="how far to reverse between tries (backward moves are unprotected)")
    parser.add_argument('--max-hops', type=positive(int), default=20,
                        help="give up a try after this many hops")
    parser.add_argument('--fake', type=float, metavar='CM',
                        help="run against a simulated robot this far from a wall")
    args = parser.parse_args()

    if args.fake is None and not confirm(
            "Place the robot facing a flat wall, about 1 m away, with at least "
            f"{args.back_off_cm:.0f} cm clear behind it. It will drive at the wall "
            f"{args.tries} times and back off between tries. Button A stops it."):
        return

    robot, mover, log = build(fake_obstacle_cm=args.fake)
    results = []
    try:
        for n in range(1, args.tries + 1):
            print(f"Try {n}:")
            first_reading = len(log.readings)
            last = one_try(mover, args.max_hops)
            if last.outcome is Outcome.STOPPED:
                print(f"  Halted: {last.reason}. Ending the test.")
                break
            distance = settled_distance(mover)
            # A try only counts when the reflex stopped it: reaching the hop
            # limit says nothing about stopping at the wall
            stopped_by_reflex = last.outcome in REFLEX_STOPS
            passed = stopped_by_reflex and distance is not None and distance >= mover.settings.threshold_cm
            results.append((n, last.outcome, distance, passed))
            raw = [r for _, r in log.readings[first_reading:]]
            odd = [r for r in raw if not plausible(r)]
            verdict = 'PASS' if passed else 'FAIL'
            if not stopped_by_reflex:
                verdict += f" (hop limit of {args.max_hops} reached before the reflex stopped a drive)"
            print(f"  Settled at {describe(distance)}: {verdict}. "
                  f"{len(raw)} readings, {len(odd)} lost or implausible{': ' + str(odd) if odd else ''}")
            if n < args.tries:
                back = back_off(mover, args.back_off_cm)
                if back.outcome is Outcome.STOPPED:
                    print(f"  Halted while backing off: {back.reason}. Ending the test.")
                    break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.stop()

    passes = sum(passed for *_, passed in results)
    print(f"\n{passes}/{len(results)} tries stopped at or before {mover.settings.threshold_cm:.0f} cm "
          f"(target: {args.tries}/{args.tries}).")
    for n, outcome, distance, passed in results:
        print(f"  try {n:2}: {outcome.value:9} settled at {describe(distance)}  {'PASS' if passed else 'FAIL'}")


if __name__ == '__main__':
    main()
