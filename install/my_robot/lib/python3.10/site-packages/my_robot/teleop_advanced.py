#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class AdvancedTeleop(Node):
    def __init__(self):
        super().__init__("advanced_teleop")

        # ---------- MAPEADO REAL GAMESIR ----------
        self.BUTTON_LB = 6   # - angular
        self.BUTTON_LT = 8   # + angular

        self.BUTTON_RB = 7   # - lineal
        self.BUTTON_RT = 9   # + lineal
        # ------------------------------------------

        # Ejes joystick izquierdo
        self.AXIS_LINEAR_X = 1   # adelante / atrás
        self.AXIS_ANGULAR_Z = 0  # izquierda / derecha

        # Velocidades iniciales
        self.linear_speed = 0.30
        self.angular_speed = 0.80

        self.linear_min = 0.05
        self.linear_max = 1.00

        self.angular_min = 0.10
        self.angular_max = 2.50

        self.DELTA = 0.05

        # ROS
        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.last_buttons = []

        self.get_logger().info("? GameSir teleop listo: joystick + control RH/LH")

    def joy_callback(self, msg: Joy):

        if not self.last_buttons:
            self.last_buttons = msg.buttons[:]

        self.process_speed_buttons(msg.buttons)

        # Joystick izquierdo como UIOJKLM
        linear = msg.axes[self.AXIS_LINEAR_X] * self.linear_speed
        angular = msg.axes[self.AXIS_ANGULAR_Z] * self.angular_speed

        # Enviar cmd_vel
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        self.last_buttons = msg.buttons[:]

    def process_speed_buttons(self, buttons):

        # + LINEAL (RT)
        if buttons[self.BUTTON_RT] == 1 and self.last_buttons[self.BUTTON_RT] == 0:
            self.linear_speed = min(self.linear_speed + self.DELTA, self.linear_max)
            self.get_logger().info(f"?? Velocidad lineal = {self.linear_speed:.2f}")

        # - LINEAL (RB)
        if buttons[self.BUTTON_RB] == 1 and self.last_buttons[self.BUTTON_RB] == 0:
            self.linear_speed = max(self.linear_speed - self.DELTA, self.linear_min)
            self.get_logger().info(f"?? Velocidad lineal = {self.linear_speed:.2f}")

        # + ANGULAR (LT)
        if buttons[self.BUTTON_LT] == 1 and self.last_buttons[self.BUTTON_LT] == 0:
            self.angular_speed = min(self.angular_speed + self.DELTA, self.angular_max)
            self.get_logger().info(f"?? Velocidad angular = {self.angular_speed:.2f}")

        # - ANGULAR (LB)
        if buttons[self.BUTTON_LB] == 1 and self.last_buttons[self.BUTTON_LB] == 0:
            self.angular_speed = max(self.angular_speed - self.DELTA, self.angular_min)
            self.get_logger().info(f"?? Velocidad angular = {self.angular_speed:.2f}")


def main():
    rclpy.init()
    node = AdvancedTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
