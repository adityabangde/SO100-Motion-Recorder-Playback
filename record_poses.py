#!/usr/bin/env python
"""
Record motor poses interactively.

Controls:
  a  - record current motor position as a waypoint
  s  - save all recorded waypoints to JSON and exit
  q  - quit without saving
"""

import json
import sys
import termios
import time
import tty
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

PORT = "/dev/ttyACM0"
ROBOT_ID = "my_awesome_follower_arm"
OUTPUT_FILE = "poses.json"

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
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS

    fpath = HF_LEROBOT_CALIBRATION / ROBOTS / "so_follower" / f"{robot_id}.json"
    if not fpath.is_file():
        raise FileNotFoundError(f"Calibration file not found: {fpath}\nRun lerobot-calibrate first.")
    with open(fpath) as f, draccus.config_type("json"):
        return draccus.load(dict[str, MotorCalibration], f)


def get_key() -> str:
    """Read a single keypress without waiting for ENTER."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_positions(bus: FeetechMotorsBus) -> dict:
    return bus.sync_read("Present_Position")


def main():
    print("Loading calibration...")
    calibration = load_calibration(ROBOT_ID)

    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    print(f"Connecting to motors on {PORT}...")
    bus.connect()
    bus.disable_torque()
    print("Torque disabled — move arm freely.\n")

    waypoints = []

    print("Controls:  [a] record pose   [s] save & exit   [q] quit\n")

    try:
        while True:
            key = get_key()

            if key == "a":
                pos = read_positions(bus)
                waypoints.append(pos)
                print(f"[{len(waypoints)}] Recorded: { {k: round(v, 2) for k, v in pos.items()} }")

            elif key == "s":
                out = Path(OUTPUT_FILE)
                out.write_text(json.dumps(waypoints, indent=2))
                print(f"\nSaved {len(waypoints)} waypoints to {out.resolve()}")
                break

            elif key == "q":
                print("\nQuit without saving.")
                break

    finally:
        bus.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
