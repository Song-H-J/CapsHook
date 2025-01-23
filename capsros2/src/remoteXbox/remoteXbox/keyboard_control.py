#!/usr/bin/env python

import os
import sys
import rclpy
import select
import termios
import tty

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

MAX_LIN_VEL = 10.0
MAX_ANG_VEL = 10.0

LIN_VEL_STEP_SIZE = 1.0
ANG_VEL_STEP_SIZE = 1.0

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
Press x to stop
CTRL-C to quit
"""

e = """
Communications Failed
"""

def print_vels(target_linear_velocity, target_angular_velocity):
    print(f'currently:\tlinear velocity {target_linear_velocity:.2f}\t angular velocity {target_angular_velocity:.2f}')

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def constrain(input_vel, low_bound, high_bound):
    return max(min(input_vel, high_bound), low_bound)

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while rclpy.ok():
            key = get_key(settings)
            if key:
                if key == 'w':
                    target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                elif key == 's':
                    target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                elif key == 'a':
                    target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                elif key == 'd':
                    target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                elif key == 'x':
                    target_linear_velocity = 0.0
                    control_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    control_angular_velocity = 0.0
                elif key == '\x03':
                    break

                print_vels(target_linear_velocity, target_angular_velocity)

            twist = Twist()
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(control_angular_velocity)

            pub.publish(twist)

    except Exception as e:
        print(f"Error in main loop: {e}")

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
