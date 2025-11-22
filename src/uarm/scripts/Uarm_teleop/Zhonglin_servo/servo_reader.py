#!/usr/bin/env python3
import rospy
import serial
import time
import re
from std_msgs.msg import Float64MultiArray

class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")

        self.arm_topic = rospy.get_param("~arm_topic", "/arm_left")
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        servo_count = rospy.get_param("~servo_count", 8)  # 7 DOF arm + gripper
        self.servo_ids = rospy.get_param("~servo_ids", list(range(servo_count)))

        self.pub = rospy.Publisher(self.arm_topic, Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(rospy.get_param("~publish_rate", 50))

        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
        rospy.loginfo(f"Serial port {self.serial_port} opened, publishing {len(self.servo_ids)} servos to {self.arm_topic}")

        self.zero_angles = {sid: 0.0 for sid in self.servo_ids}
        self._init_servos()

    def send_command(self, cmd):
        self.ser.write(cmd.encode('ascii'))
        time.sleep(0.008)
        return self.ser.read_all().decode('ascii', errors='ignore')

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def _init_servos(self):
        self.send_command('#000PVER!')
        for servo_id in self.servo_ids:
            self.send_command("#000PCSK!")
            self.send_command(f'#{servo_id:03d}PULK!')
            response = self.send_command(f'#{servo_id:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            self.zero_angles[servo_id] = angle if angle is not None else 0.0
        rospy.loginfo(f"Servo initial angle calibration completed for {len(self.servo_ids)} servos")

    def run(self):
        angle_offset = {sid: 0.0 for sid in self.servo_ids}  # Current published angles
        target_angle_offset = {sid: 0.0 for sid in self.servo_ids}  # Target angle for each servo
        num_interp = 5  # Interpolation steps
        step_size = 1  # Minimum change threshold

        while not rospy.is_shutdown():
            for servo_id in self.servo_ids:
                response = self.send_command(f'#{servo_id:03d}PRAD!')
                angle = self.pwm_to_angle(response.strip())
                if angle is not None:
                    new_angle = angle - self.zero_angles[servo_id]
                    if abs(new_angle - target_angle_offset[servo_id]) > step_size:
                        target_angle_offset[servo_id] = new_angle
                else:
                    rospy.logwarn(f"Servo {servo_id} response error: {response.strip()}")

            # Interpolate to approach target angle
            for step in range(num_interp):
                for servo_id in self.servo_ids:
                    delta = target_angle_offset[servo_id] - angle_offset[servo_id]
                    angle_offset[servo_id] += delta * 0.2  # Lazy interpolation, coefficient < 1 for adjustable smoothness
                ordered_angles = [angle_offset[sid] for sid in self.servo_ids]
                self.pub.publish(Float64MultiArray(data=ordered_angles))
                self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
