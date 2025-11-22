import serial
import time 
import numpy as np
import re

# Initial positions for dual-arm (left 8 + right 8); extend with zeros if not used
init_qpos = np.array([14.1, -8, -24.7, 196.9, 62.3, -8.8, 0.0, 0.0] + [0.0] * 8)
init_qpos = np.radians(init_qpos)


# Set serial port parameters
SERIAL_PORT_LEFT = '/dev/ttyUSB0'
SERIAL_PORT_RIGHT = '/dev/ttyUSB1'
BAUDRATE = 115200

def send_command(ser, cmd):
    ser.write(cmd.encode('ascii'))
    time.sleep(0.01)
    response = ser.read_all()
    # print(f"Raw response (binary): {response}")  # Print raw binary response
    return response.decode('ascii', errors='ignore')

def pwm_to_angle(response_str, pwm_min=500, pwm_max=2500, angle_range=270):
    match = re.search(r'P(\d{4})', response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    pwm_span = pwm_max - pwm_min
    angle = (pwm_val - pwm_min) / pwm_span * angle_range
    return angle

def angle_to_gripper(angle_deg, angle_range=270, pos_min=50, pos_max=730):
    """
    Map servo angle (degrees) to gripper position.

    Parameters:
    - angle_deg: Servo angle in degrees
    - angle_range: Maximum servo angle (default 270°)
    - pos_min: Gripper closed position (default 50)
    - pos_max: Gripper open position (default 730)

    Returns:
    - gripper position (integer)
    """
    ratio = (angle_deg / angle_range) * 3
    position = pos_min + (pos_max - pos_min) * ratio
    return int(np.clip(position, pos_min, pos_max))

def main():
    index = 2
    left_count = 8
    right_count = 8
    arm_pos = [0.0] * (left_count + right_count)
    angle_pos = [0.0] * (left_count + right_count)
    zero_angles = [0.0] * (left_count + right_count)

    with serial.Serial(SERIAL_PORT_LEFT, BAUDRATE, timeout=0.01) as ser_left, \
         serial.Serial(SERIAL_PORT_RIGHT, BAUDRATE, timeout=0.01) as ser_right:
        print("Serial ports opened")

        # Version queries (one per port)
        response_left = send_command(ser_left, f'#00{index}PVER!')
        response_right = send_command(ser_right, f'#00{index}PVER!')

        # Release torque on all servos for both arms
        for i in range(left_count):
            cmd = f'#00{i}PULK!'
            resp = send_command(ser_left, cmd)
            print(f"[Left] Servo {i} torque released: {resp.strip()}")
        for i in range(right_count):
            cmd = f'#00{i}PULK!'
            resp = send_command(ser_right, cmd)
            print(f"[Right] Servo {i} torque released: {resp.strip()}")

        print(f"Version response left: {response_left.strip()}")
        print(f"Version response right: {response_right.strip()}")

        while True:
            # Left arm
            for i in range(left_count):
                cmd = f'#00{i}PRAD!'
                response = send_command(ser_left, cmd)
                angle = pwm_to_angle(response.strip())
                angle_pos[i] = angle
                if angle is not None:
                    angle_offset = angle - zero_angles[i] + np.degrees(init_qpos[i])
                    arm_pos[i] = np.radians(angle_offset)
            # Right arm
            for i in range(right_count):
                cmd = f'#00{i}PRAD!'
                response = send_command(ser_right, cmd)
                angle = pwm_to_angle(response.strip())
                angle_pos[left_count + i] = angle
                if angle is not None:
                    angle_offset = angle - zero_angles[left_count + i] + np.degrees(init_qpos[left_count + i])
                    arm_pos[left_count + i] = np.radians(angle_offset)

            # Print angles with one decimal place
            print([f"{a:.1f}" if a is not None else "nan" for a in angle_pos])
    
if __name__ == "__main__":
    main()
