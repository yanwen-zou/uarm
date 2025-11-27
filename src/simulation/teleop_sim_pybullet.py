"""
Minimal PyBullet teleop for openarm_bi.

Reads 16 joint angles from two serial ports (8 per arm), subtracts calibration
offsets, maps to 18 DoF (fingers duplicated), and sends POSITION_CONTROL
commands in the PyBullet GUI.
"""

from __future__ import annotations

import argparse
import queue
import re
import time
from pathlib import Path
from typing import Dict, List

import numpy as np
import pybullet as p
import pybullet_data
import serial


def pwm_to_angle(response_str: str, pwm_min: int = 500, pwm_max: int = 2500, angle_range: float = 270):
    """Parse servo response and convert PWM to degrees."""
    match = re.search(r"P(\d{4})", response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    pwm_span = pwm_max - pwm_min
    angle = (pwm_val - pwm_min) / pwm_span * angle_range
    return angle


def send_command(ser_obj: serial.Serial, cmd: str, sleep_s: float = 0.01) -> str:
    """Send a command to a serial device and read all available bytes."""
    ser_obj.write(cmd.encode("ascii"))
    time.sleep(sleep_s)
    response = ser_obj.read_all()
    return response.decode("ascii", errors="ignore") if response else ""


class PyBulletOpenArmBiTeleop:
    """PyBullet-based dual-arm OpenArm teleop using two serial ports."""

    JOINT_NAMES = [
        # Left arm 7
        "openarm_left_joint1",
        "openarm_left_joint2",
        "openarm_left_joint3",
        "openarm_left_joint4",
        "openarm_left_joint5",
        "openarm_left_joint6",
        "openarm_left_joint7",
        # Left fingers (2)
        "openarm_left_finger_joint1",
        "openarm_left_finger_joint2",
        # Right arm 7
        "openarm_right_joint1",
        "openarm_right_joint2",
        "openarm_right_joint3",
        "openarm_right_joint4",
        "openarm_right_joint5",
        "openarm_right_joint6",
        "openarm_right_joint7",
        # Right fingers (2)
        "openarm_right_finger_joint1",
        "openarm_right_finger_joint2",
    ]

    def __init__(
        self,
        urdf_path: Path,
        serial_left: str = "/dev/ttyUSB0",
        serial_right: str = "/dev/ttyUSB1",
        control_freq: float = 50.0,
        use_serial: bool = True,
    ):
        self.urdf_path = urdf_path
        self.control_freq = control_freq
        self.use_serial = use_serial

        # PyBullet setup
        p.connect(p.GUI)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(
            str(self.urdf_path),
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION | p.URDF_MAINTAIN_LINK_ORDER,
        )
        self.joint_name_to_idx: Dict[str, int] = {}
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            name = info[1].decode("utf-8")
            self.joint_name_to_idx[name] = i
        missing = [n for n in self.JOINT_NAMES if n not in self.joint_name_to_idx]
        if missing:
            raise RuntimeError(f"Missing joints in URDF: {missing}")

        # Serial setup
        self.ser_left = None
        self.ser_right = None
        if use_serial:
            self.ser_left = serial.Serial(serial_left, 115200, timeout=0.01)
            self.ser_right = serial.Serial(serial_right, 115200, timeout=0.01)

        # State
        self.zero_angles = [0.0] * 16
        self.arm_pos_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
        self.stop = False

        if use_serial:
            self._init_servos(self.ser_left, 0)
            self._init_servos(self.ser_right, 8)

    def _init_servos(self, ser_obj: serial.Serial, start_idx: int):
        """Release torque and record zero positions for one arm (8 servos)."""
        send_command(ser_obj, "#000PVER!")
        for i in range(8):
            send_command(ser_obj, "#000PCSK!")
            send_command(ser_obj, f"#{i:03d}PULK!")
            response = send_command(ser_obj, f"#{i:03d}PRAD!")
            angle = pwm_to_angle(response.strip())
            self.zero_angles[start_idx + i] = angle if angle is not None else 0.0

    def _angle_stream_loop(self):
        """Read angles from both serial ports and publish the latest 16-D array."""
        left_count = right_count = 8
        period = max(1.0 / self.control_freq, 1e-6)
        next_time = time.monotonic()

        while not self.stop:
            arm_pos_deg = [0.0] * 16
            # Left arm
            for i in range(left_count):
                response = send_command(self.ser_left, f"#{i:03d}PRAD!", sleep_s=0.008)
                angle = pwm_to_angle(response.strip())
                if angle is None:
                    continue
                arm_pos_deg[i] = angle - self.zero_angles[i]
            # Right arm
            for i in range(right_count):
                response = send_command(self.ser_right, f"#{i:03d}PRAD!", sleep_s=0.008)
                angle = pwm_to_angle(response.strip())
                if angle is None:
                    continue
                arm_pos_deg[left_count + i] = angle - self.zero_angles[left_count + i]

            arm_pos_rad = np.radians(arm_pos_deg).astype(np.float32)

            # Overwrite queue with latest
            try:
                while True:
                    self.arm_pos_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.arm_pos_queue.put_nowait(arm_pos_rad)
            except Exception:
                pass

            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()

    @staticmethod
    def _map_to_ctrl(raw: np.ndarray) -> np.ndarray:
        """Map 16-DOF (left/right 8 joints) to 18 controls (2 gripper joints duplicated)."""
        if raw.shape[0] != 16:
            raise ValueError(f"Expected 16-length raw array, got {raw.shape[0]}")
        action = np.zeros(18, dtype=np.float32)
        # Left arm joints
        action[0:7] = raw[0:7]
        action[7] = raw[7]
        action[8] = raw[7]
        # Right arm joints
        action[9:16] = raw[8:15]
        action[16] = raw[15]
        action[17] = raw[15]
        return action

    def _set_joint_positions(self, targets: np.ndarray):
        """Send position commands to all joints."""
        for name, target in zip(self.JOINT_NAMES, targets):
            idx = self.joint_name_to_idx[name]
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(target),
                positionGain=0.2,
                velocityGain=1.0,
                force=200.0,
            )

    def run(self):
        """Main control loop. Runs reader thread (if serial) and drives PyBullet."""
        if self.use_serial:
            import threading

            reader = threading.Thread(target=self._angle_stream_loop, daemon=True)
            reader.start()

        dt = 1.0 / self.control_freq
        try:
            while p.isConnected():
                if self.use_serial:
                    try:
                        raw = self.arm_pos_queue.get(timeout=0.001)
                    except queue.Empty:
                        raw = None
                else:
                    # Demo: gentle oscillation for quick visual check
                    t = time.time()
                    raw = np.array([0.2 * np.sin(t)] * 16, dtype=np.float32)

                if raw is not None:
                    ctrl = self._map_to_ctrl(raw)
                    self._set_joint_positions(ctrl)

                p.stepSimulation()
                time.sleep(dt)
        finally:
            self.stop = True
            try:
                if self.use_serial and self.ser_left:
                    self.ser_left.close()
                if self.use_serial and self.ser_right:
                    self.ser_right.close()
            except Exception:
                pass
            p.disconnect()


def main():
    parser = argparse.ArgumentParser(description="PyBullet teleop for OpenArm bimanual")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("src/simulation/mani_skill/assets/robots/openarm_bi/openarm_bi.urdf"),
        help="Path to URDF model file",
    )
    parser.add_argument("--serial-left", type=str, default="/dev/ttyUSB0", help="Left arm serial port")
    parser.add_argument("--serial-right", type=str, default="/dev/ttyUSB1", help="Right arm serial port")
    parser.add_argument("--rate", type=float, default=50.0, help="Control frequency (Hz)")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run without serial. Plays a simple oscillation for quick visualization.",
    )
    args = parser.parse_args()

    teleop = PyBulletOpenArmBiTeleop(
        urdf_path=args.urdf,
        serial_left=args.serial_left,
        serial_right=args.serial_right,
        control_freq=args.rate,
        use_serial=not args.demo,
    )
    teleop.run()


if __name__ == "__main__":
    main()
