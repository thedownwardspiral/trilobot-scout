# Trilobot Scout

Give a [Pimoroni Trilobot](https://shop.pimoroni.com/products/trilobot) a goal in plain language and let it work its way there. A Claude vision model looks through the robot's camera and picks one small move at a time; local code on the Raspberry Pi decides whether each move is allowed to happen, using the front ultrasonic sensor as a reflex. The robot has no wheel encoders, so moves are timed.

The full brief and build plan are in the [starter handoff doc](https://claude.ai/artifact/UexV6vCbBYpXvMGUFC56x4).

## Status

Milestone 1 (skeleton) is done. Milestone 2 (reflex layer and moves, [issue #3](https://github.com/thedownwardspiral/trilobot-scout/issues/3)) is built and tested off the robot; its hardware checks have not been run yet.

- `scout/motion.py`: `Mover`, the one way to move the robot. `drive(distance_cm)` and `turn(degrees)` run the motors for a timed run at one slow fixed speed, clamped to 30 cm and 90 degrees, and always end with `stop()`. Each returns a `MoveResult`: `moved`, `cut_short`, `refused` or `stopped`. The drive and turn rates in `Settings` are placeholders until calibration (milestone 3).
- `scout/reflex.py`: the safety rules behind the mover. A forward drive is refused, or cut short mid-move, when the sensor reads within 30 cm plus a stopping margin, with missing echoes handled by `robot_logic`. `StopSignal` halts any move; Button A raises it, and it stays raised until cleared.
- `scout/robot_logic.py`: hardware-free helpers (distance validity, differential mixing, LED colour hysteresis, avoidance decision), copied from the earlier [trilobot](https://github.com/thedownwardspiral/trilobot) repo.
- `scout/fake_robot.py`: `FakeTrilobot`, a stand-in for `trilobot.Trilobot` with the same methods, signatures and constants, plus a small simulation with its own clock. `sleep(seconds)` advances simulated time and moves the robot at its current motor speeds, and `now()` reads the clock, so motion code that takes injectable sleep and clock functions can be tested without hardware or real waiting. Tests can also place an obstacle, queue sensor readings, press buttons, read back the LEDs and underlights, and check the log of calls (for example, that `stop()` was the last motor call).

## Hardware checks

Milestone 2 is accepted on the robot with two scripts. They move the robot, so each asks before starting, and Button A stops either one. Run them from the repo root with the Pimoroni virtualenv:

```bash
~/.virtualenvs/pimoroni/bin/python scripts/wall_test.py
~/.virtualenvs/pimoroni/bin/python scripts/stop_button_test.py
```

`wall_test.py` drives at a wall 10 times and passes when every try settles at 30 cm or more; it also reports lost or implausible sensor readings. `stop_button_test.py` makes a run of moves (add `--turn` for turns) and reports whether a Button A press halted one mid-move. To rehearse either one against the simulated robot without moving anything, add `--fake`: `wall_test.py --fake 120` starts 120 cm from a wall, and `stop_button_test.py --fake 0.9` presses Button A at 0.9 s.

## Running the tests

The tests use only the Python standard library. From the repo root:

```bash
python3 -m unittest
```

`tests/test_fake_robot.py` includes a parity check that compares `FakeTrilobot` against the real `trilobot.Trilobot` class. It runs only when the `trilobot` library can be imported, which is true in the Pimoroni virtualenv on the robot, and is skipped elsewhere. It never constructs the real class, because that touches the hardware.

## Repo setup

The security workflows, Dependabot config, branch ruleset and pre-commit secret scan come from [repo-template](https://github.com/thedownwardspiral/repo-template); its README explains what each one does. `.github/setup-repo.sh` has already been run for this repo; run `pre-commit install` in each clone.

## Licence

GPL-3.0, the same as the trilobot repo that `robot_logic.py` comes from. See [LICENSE](LICENSE).
