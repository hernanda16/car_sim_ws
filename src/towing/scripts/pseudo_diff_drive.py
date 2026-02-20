#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class PseudoAckermann(Node):
    def __init__(self):
        super().__init__('pseudo_ackermann_level3')

        # ===== ROBOT PARAM =====
        self.wheel_radius = 0.23
        self.wheel_base = 1.01086
        self.wheel_separation = 0.86

        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            '/drive_controller/commands',
            10)

        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.get_logger().info("Pseudo Ackermann LEVEL 3 READY 🚀")

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # =========================
        # DIFFERENTIAL REAR
        # =========================
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        wl = v_left / self.wheel_radius
        wr = v_right / self.wheel_radius

        drive_msg = Float64MultiArray()
        drive_msg.data = [wl, wr]
        self.drive_pub.publish(drive_msg)

        # =========================
        # STEERING ANGLE
        # =========================
        steer_angle = 0.0
        if abs(v) > 0.001:
            steer_angle = math.atan((w * self.wheel_base) / v)

        steer_angle = max(min(steer_angle, 1.8), -1.8)

        steer_msg = Float64MultiArray()
        steer_msg.data = [steer_angle]
        self.steer_pub.publish(steer_msg)


def main():
    rclpy.init()
    node = PseudoAckermann()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()