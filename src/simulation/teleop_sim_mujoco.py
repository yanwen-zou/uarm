import argparse
import queue
import re
import threading
import time
from pathlib import Path

import mujoco
from mujoco import viewer
import numpy as np
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


class MujocoOpenArmBiTeleop:
    """Mujoco-based dual-arm OpenArm teleoperation using two serial ports."""

    def __init__(
        self,
        model_xml_path: Path,
        serial_left: str = "/dev/ttyUSB0",
        serial_right: str = "/dev/ttyUSB1",
        control_freq: float = 50.0,
    ):
        self.model = mujoco.MjModel.from_xml_path(str(model_xml_path))
        self.data = mujoco.MjData(self.model)

        # Serial setup
        self.ser_left = serial.Serial(serial_left, 115200, timeout=0.01)
        self.ser_right = serial.Serial(serial_right, 115200, timeout=0.01)

        self.control_freq = control_freq
        self.stop_event = threading.Event()
        self.arm_pos_queue: "queue.Queue[list[float]]" = queue.Queue(maxsize=1)

        # Calibration
        self.zero_angles = [0.0] * 16
        self._init_servos(self.ser_left, 0)
        self._init_servos(self.ser_right, 8)

        # Threads
        self.reader_thread = threading.Thread(target=self._angle_stream_loop, daemon=True)
        self.reader_thread.start()

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

        while not self.stop_event.is_set():
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
        """Map 16-DOF (left/right 8 joints) to 18 Mujoco controls (2 gripper joints duplicated)."""
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

    def run(self):
        """Launch Mujoco viewer and control loop."""
        dt = 1.0 / self.control_freq
        with viewer.launch_passive(self.model, self.data) as v:
            v.cam.distance = 3.0
            while v.is_running() and not self.stop_event.is_set():
                try:
                    raw = self.arm_pos_queue.get(timeout=0.001)
                except queue.Empty:
                    raw = None
                if raw is not None:
                    ctrl = self._map_to_ctrl(raw)
                    self.data.ctrl[: ctrl.shape[0]] = ctrl

                mujoco.mj_step(self.model, self.data)
                v.sync()
                time.sleep(dt)

    def close(self):
        self.stop_event.set()
        try:
            self.ser_left.close()
        except Exception:
            pass
        try:
            self.ser_right.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Mujoco teleop for OpenArm bimanual")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("src/simulation/mani_skill/assets/robots/openarm_bi/openarm_bi.urdf"),
        help="Path to MJCF/URDF model file for Mujoco",
    )
    parser.add_argument("--serial-left", type=str, default="/dev/ttyUSB0", help="Left arm serial port")
    parser.add_argument("--serial-right", type=str, default="/dev/ttyUSB1", help="Right arm serial port")
    parser.add_argument("--rate", type=float, default=50.0, help="Control frequency (Hz)")
    args = parser.parse_args()

    teleop = MujocoOpenArmBiTeleop(
        model_xml_path=args.model,
        serial_left=args.serial_left,
        serial_right=args.serial_right,
        control_freq=args.rate,
    )
    try:
        teleop.run()
    finally:
        teleop.close()


if __name__ == "__main__":
    main()
