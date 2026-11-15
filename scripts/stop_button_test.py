"""
Milestone 2 acceptance check: Button A halts the robot mid-move.

The robot makes a run of moves (30 cm drives, or 90 degree turns with --turn)
and you press Button A while one is under way. The script reports which move
the press stopped, and how far into it.

    ~/.virtualenvs/pimoroni/bin/python scripts/stop_button_test.py
    ~/.virtualenvs/pimoroni/bin/python scripts/stop_button_test.py --turn
    ~/.virtualenvs/pimoroni/bin/python scripts/stop_button_test.py --fake 0.9   # simulate a press at 0.9 s
"""

import argparse

from hardware import build, confirm

from scout.fake_robot import BUTTON_A
from scout.motion import Outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--turn', action='store_true', help="turn on the spot instead of driving forward")
    parser.add_argument('--moves', type=int, default=8, help="most moves to make before giving up")
    parser.add_argument('--fake', type=float, metavar='SECONDS',
                        help="run against a simulated robot that presses Button A at this time")
    args = parser.parse_args()

    kind = "90 degree turns" if args.turn else "30 cm forward drives (the wall veto stays on)"
    if args.fake is None and not confirm(
            f"The robot will make up to {args.moves} {kind}. Press Button A while it is "
            "moving, not between moves. Keep clear space around it."):
        return

    robot, mover, _ = build(fake_obstacle_cm=None if args.fake is None else 500)
    if args.fake is not None:
        robot.at(args.fake, lambda: robot.press_button(BUTTON_A))

    move = (lambda: mover.turn(90)) if args.turn else (lambda: mover.drive(30))
    planned_s = (90 / mover.settings.deg_per_s) if args.turn else (30 / mover.settings.cm_per_s)
    try:
        for n in range(1, args.moves + 1):
            result = move()
            if result.outcome is Outcome.STOPPED:
                if result.seconds > 0:
                    print(f"PASS: move {n} halted after {result.seconds:.2f} s of a {planned_s:.2f} s move "
                          f"({result.reason}). The mover checks every {mover.settings.poll_interval_s:.2f} s.")
                else:
                    print(f"Not mid-move: the press was seen before move {n} started ({result.reason}). "
                          "Run again and press during a move.")
                return
            if result.outcome is not Outcome.MOVED:
                print(f"Move {n} {result.outcome.value}: {result.reason}. Clear the path and run again.")
                return
            print(f"Move {n}: {result.outcome.value} in {result.seconds:.2f} s")
        print(f"No press seen in {args.moves} moves.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.stop()


if __name__ == '__main__':
    main()
