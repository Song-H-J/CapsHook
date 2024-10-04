#!/usr/bin/env python3

import os
import sys
import rclpy
import evdev
import time

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from evdev import InputDevice, ecodes, categorize

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 10
MAX_ANG_VEL = 10

LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 1

msg = """
Control Your Robot!
---------------------------
CTRL-C to quit
"""

e = """
Communications Failed
"""

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity, target_angular_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)
 

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)

def main():
    # ===================== 컨트롤러 찾는 코드 ===================== #
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    controller = None

    for device in devices:
        print(f"Detected device: {device.path}, {device.name}, {device.phys}")
        if "Microsoft Xbox Controller" in device.name:
            controller = evdev.InputDevice(device.path)
            print(f"Using controller: {controller.path}, {controller.name}, {controller.phys}")
            break

    if controller is None:
        print("No Microsoft Xbox Controller found. Please ensure the controller is connected and recognized by the system.")
        sys.exit(1)
    # ============================================================ #

    settings = None
    if os.name != 'nt':
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
        motorState = "Stop"
        # ===================== 컨트롤러 입력 받아서 motorState 설정하는 코드 ===================== #
        while True:
            try:
                event = controller.read_one()
                if event:
                    if event.type == ecodes.EV_ABS:
                        absEvent = categorize(event)
                        if absEvent.event.code == ecodes.ABS_HAT0X:
                            if absEvent.event.value == -1:
                                motorState = "Left"
                                print("Left")
                            elif absEvent.event.value == 1:
                                motorState = "Right"
                                print("Right")
                            elif absEvent.event.value == 0:
                                motorState = "Stop"
                                print("Stop")
                        elif absEvent.event.code == ecodes.ABS_HAT0Y:
                            if absEvent.event.value == -1:
                                motorState = "Go"
                                print("Go")
                            elif absEvent.event.value == 1:
                                motorState = "Back"
                                print("Back")
                            elif absEvent.event.value == 0:
                                motorState = "Stop"
                                print("Stop")
                    # ======================================================================================== #
                    if motorState == 'Go':
                        target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                        status = status + 1
                        print_vels(target_linear_velocity, target_angular_velocity)
                    elif motorState == 'Back':
                        target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                        status = status + 1
                        print_vels(target_linear_velocity, target_angular_velocity)
                    elif motorState == 'Left':
                        target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                        status = status + 1
                        print_vels(target_linear_velocity, target_angular_velocity)
                    elif motorState == 'Right':
                        target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                        status = status + 1
                        print_vels(target_linear_velocity, target_angular_velocity)
                    elif motorState == 'Stop':
                        target_linear_velocity = 0.0
                        control_linear_velocity = 0.0
                        target_angular_velocity = 0.0
                        control_angular_velocity = 0.0
                        print_vels(target_linear_velocity, target_angular_velocity)

                    if status == 20:
                        print(msg)
                        status = 0

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
                    twist.angular.z = control_angular_velocity

                    pub.publish(twist)

            except Exception as e:
                print(f"Error reading event: {e}")

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

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
