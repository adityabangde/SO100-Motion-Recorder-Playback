#!/usr/bin/env python
"""
Replay recorded motor poses by interpolating between waypoints.

Usage:
  python replay_poses.py [poses.json] [--steps N] [--hz N] [--loops N]

  --steps   interpolation steps between each waypoint pair (default: 50)
  --hz      control loop frequency in Hz (default: 50)
  --loops   how many times to replay the full sequence (default: 1)
"""

import argparse
import json
import time
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

PORT = "/dev/ttyACM0"
ROBOT_ID = "my_awesome_follower_arm"

MOTORS = {
    "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex":    Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex":    Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll":    Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}


def load_calibration(robot_id: str) -> dict[str, MotorCalibration]:
    import draccus

    fpath = Path(__file__).parent / f"{robot_id}.json"
    if not fpath.is_file():
        raise FileNotFoundError(f"Calibration file not found: {fpath}")
    with open(fpath) as f, draccus.config_type("json"):
        return draccus.load(dict[str, MotorCalibration], f)


def interpolate(start: dict, end: dict, steps: int):
    """Yield `steps` interpolated poses between start and end (inclusive of end)."""
    keys = list(start.keys())
    for i in range(1, steps + 1):
        t = i / steps
        yield {k: start[k] + (end[k] - start[k]) * t for k in keys}


def move_to(bus: FeetechMotorsBus, pose: dict, hz: float):
    bus.sync_write("Goal_Position", pose)
    time.sleep(1.0 / hz)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", nargs="?", default="poses.json")
    parser.add_argument("--steps", type=int, default=50, help="Interpolation steps between waypoints")
    parser.add_argument("--hz", type=float, default=50.0, help="Control frequency in Hz")
    parser.add_argument("--loops", type=int, default=1, help="Number of times to replay")
    args = parser.parse_args()

    waypoints = json.loads(Path(args.file).read_text())
    if len(waypoints) < 2:
        print("Need at least 2 waypoints to replay.")
        return

    print(f"Loaded {len(waypoints)} waypoints from '{args.file}'")
    print(f"Interpolation steps: {args.steps}  |  Hz: {args.hz}  |  Loops: {args.loops}")

    calibration = load_calibration(ROBOT_ID)
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    print(f"Connecting on {PORT}...")
    bus.connect()

    # Enable torque and set position mode
    for motor in bus.motors:
        bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    bus.enable_torque()
    print("Torque enabled. Starting replay...\n")

    try:
        for loop in range(args.loops):
            print(f"Loop {loop + 1}/{args.loops}")

            # First: move smoothly from current position to waypoint[0]
            current = bus.sync_read("Present_Position")
            print(f"  Moving to start waypoint...")
            for pose in interpolate(current, waypoints[0], args.steps):
                move_to(bus, pose, args.hz)

            # Then replay all waypoints in sequence
            for i in range(len(waypoints) - 1):
                print(f"  Waypoint {i + 1} → {i + 2}")
                for pose in interpolate(waypoints[i], waypoints[i + 1], args.steps):
                    move_to(bus, pose, args.hz)

        print("\nReplay complete.")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        bus.disable_torque()
        bus.disconnect()
        print("Torque disabled. Disconnected.")


if __name__ == "__main__":
    main()
