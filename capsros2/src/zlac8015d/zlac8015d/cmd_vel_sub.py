import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from zltech import ZLAC8015D

class DifferentialDriveKinematics(Node):
    def __init__(self):
        super().__init__('zlac8015d')
        self.declare_parameter('thread', 0.55)  # m
        self.declare_parameter('max_speed', 25.0)   # m/s
        self.declare_parameter('max_angular_velocity', 25.0)  # rad/s
        self.declare_parameter('wheel_radius', 0.1)  # m
        self.declare_parameter('motor_max_rpm', 35)
        self.declare_parameter('motor_min_rpm', -35)

        self.thread = self.get_parameter('thread').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.MOTOR_MAX_RPM = self.get_parameter('motor_max_rpm').get_parameter_value().integer_value
        self.MOTOR_MIN_RPM = self.get_parameter('motor_min_rpm').get_parameter_value().integer_value

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 타이머 설정: 0.1초마다 cmd_vel 메시지 처리
        self.timer_period = 0.01  # 주기 (초)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.latest_cmd_vel = None  # 마지막으로 수신한 cmd_vel 메시지 저장

#################################################################
# left id = 1 , right id = 2
#################################################################
        self.motor = ZLAC8015D.MotorController(port="/dev/ttyZLAC8015D")
        self.initialize_motors()
        self.get_logger().info('zlac8015d node started')
        
    def initialize_motors(self):
        self.motor.disable_motor(1)
        self.motor.enable_motor(1)
        self.motor.set_mode(4,1)
        self.motor.set_max_rpm(200,1)
        self.motor.set_max_L_current(15,1)
        self.motor.set_max_R_current(15,1)
        self.motor.set_rated_L_current(6,1)
        self.motor.set_rated_R_current(6,1)

        self.motor.disable_motor(2)
        self.motor.enable_motor(2)
        self.motor.set_mode(4,2)
        self.motor.set_max_rpm(200,2)
        self.motor.set_max_L_current(15,2)
        self.motor.set_max_R_current(15,2)
        self.motor.set_rated_L_current(6,2)
        self.motor.set_rated_R_current(6,2)
            
    def cmd_vel_callback(self, msg):
        # 최신 cmd_vel 메시지를 저장
        self.latest_cmd_vel = msg

    def timer_callback(self):
        if self.latest_cmd_vel is None:
            return  # 수신된 메시지가 없으면 처리하지 않음

        linear_velocity_x = self.latest_cmd_vel.linear.x  # m/s
        yaw_rate = self.latest_cmd_vel.angular.z  # rad/s

        wheel_speeds = self.compute_inverse_kinematics(linear_velocity_x, yaw_rate)
        wheel_speeds_rpm = self.convert_to_rpm(wheel_speeds)

        # Apply max/min RPM limits
        wheel_speeds_rpm[0] = np.clip(wheel_speeds_rpm[0], self.MOTOR_MIN_RPM, self.MOTOR_MAX_RPM)
        wheel_speeds_rpm[1] = np.clip(wheel_speeds_rpm[1], self.MOTOR_MIN_RPM, self.MOTOR_MAX_RPM)

        self.motor.Lok = True
        self.motor.Rok = True
        self.get_logger().info(f'L : {wheel_speeds_rpm[0]} RPM, R : {wheel_speeds_rpm[1]} RPM')
        self.motor.set_rpm_w_toq(int(wheel_speeds_rpm[0]), -int(wheel_speeds_rpm[1]))

    def compute_inverse_kinematics(self, linear_velocity_x, yaw_rate):
        linear_velocity_x = np.clip(linear_velocity_x, -self.max_speed, self.max_speed)
        yaw_rate = np.clip(yaw_rate, -self.max_angular_velocity, self.max_angular_velocity)

        rotational_velocity = (self.thread / 2.0) * yaw_rate
        combined_velocity = abs(linear_velocity_x) + abs(rotational_velocity)

        # initial p gain
        gain = 1.0

        if combined_velocity > self.max_speed:
            excess_velocity = abs(combined_velocity - self.max_speed)
            adjusted_linear_velocity = abs(linear_velocity_x) - excess_velocity
            gain = adjusted_linear_velocity / abs(linear_velocity_x)

        # gain apply
        linear_velocity_x *= gain

        # left, right wheel speed calc (m/s)
        left_wheel_speed = (linear_velocity_x - rotational_velocity) / self.max_speed
        right_wheel_speed = (linear_velocity_x + rotational_velocity) / self.max_speed

        return np.array([left_wheel_speed, right_wheel_speed]) * self.max_speed

    def convert_to_rpm(self, wheel_speeds):
        # m/s to rpm
        wheel_speeds_rpm = (wheel_speeds / (2 * np.pi * self.wheel_radius)) * 60
        wheel_speeds_rpm_int16 = wheel_speeds_rpm.astype(np.int16)
        return wheel_speeds_rpm_int16

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
