# Trilobot Scout

Give a [Pimoroni Trilobot](https://shop.pimoroni.com/products/trilobot) a goal in plain language and let it work its way there. A Claude vision model looks through the robot's camera and picks one small move at a time; local code on the Raspberry Pi decides whether each move is allowed to happen, using the front ultrasonic sensor as a reflex. The robot has no wheel encoders, so moves are timed.

The full brief and build plan are in the [starter handoff doc](https://claude.ai/artifact/UexV6vCbBYpXvMGUFC56x4).

## Status

Milestone 1 of the build plan: the project skeleton, with nothing yet driving the robot.

- `scout/robot_logic.py`: hardware-free helpers (distance validity, differential mixing, LED colour hysteresis, avoidance decision), copied from the earlier [trilobot](https://github.com/thedownwardspiral/trilobot) repo.
- `scout/fake_robot.py`: `FakeTrilobot`, a stand-in for `trilobot.Trilobot` with the same methods, signatures and constants, plus a small simulation with its own clock. `sleep(seconds)` advances simulated time and moves the robot at its current motor speeds, and `now()` reads the clock, so motion code that takes injectable sleep and clock functions can be tested without hardware or real waiting. Tests can also place an obstacle, queue sensor readings, press buttons, read back the LEDs and underlights, and check the log of calls (for example, that `stop()` was the last motor call).

Next is milestone 2: the reflex layer and timed motion, tested against the fake first.

## Running the tests

The tests use only the Python standard library. From the repo root:

```bash
python3 -m unittest
```

`tests/test_fake_robot.py` includes a parity check that compares `FakeTrilobot` against the real `trilobot.Trilobot` class. It runs only when the `trilobot` library can be imported, which is true in the Pimoroni virtualenv on the robot, and is skipped elsewhere. It never constructs the real class, because that touches the hardware.

## Repo setup

The security workflows, Dependabot config, branch ruleset and pre-commit secret scan come from [repo-template](https://github.com/thedownwardspiral/repo-template); its README explains what each one does. After creating the GitHub repo, run `.github/setup-repo.sh thedownwardspiral/trilobot-scout` once, and `pre-commit install` in each clone.

## Licence

GPL-3.0, the same as the trilobot repo that `robot_logic.py` comes from. See [LICENSE](LICENSE).
