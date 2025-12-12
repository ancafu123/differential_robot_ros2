#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class AdvancedTeleop(Node):
    def __init__(self):
        super().__init__("advanced_teleop")

        # Botones (AJUSTA para tu control)
        self.BUTTON_RB = 5   # RB
        self.BUTTON_RT = 7   # RT (algunos joysticks lo dan como botón)
        self.BUTTON_LB = 4   # LB
        self.BUTTON_LT = 6   # LT

        # Ejes del joystick (normalmente)
        self.AXIS_LINEAR_X = 1   # joystick izquierdo vertical
        self.AXIS_ANGULAR_Z = 0  # joystick izquierdo horizontal

        # Parámetros de velocidad (ajustables)
        self.linear_speed = 0.3
        self.angular_speed = 0.8

        self.linear_min = 0.05
        self.linear_max = 1.0
        self.angular_min = 0.1
        self.angular_max = 2.5

        # Suscriptores y publicadores
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10)

        # Para evitar múltiples incrementos por pulsación
        self.last_buttons = []

        self.get_logger().info("Advanced teleop listo con control de 4 botones!")

    def joy_callback(self, msg: Joy):
        if not self.last_buttons:
            self.last_buttons = msg.buttons

        # Procesar botones para cambiar velocidades
        self.process_speed_buttons(msg.buttons)

        # Lectura joysticks
        linear = msg.axes[self.AXIS_LINEAR_X] * self.linear_speed
        angular = msg.axes[self.AXIS_ANGULAR_Z] * self.angular_speed

        # Publicar comando
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        # Guardar botones anteriores
        self.last_buttons = msg.buttons

    def process_speed_buttons(self, buttons):

        # ------ AUMENTAR VELOCIDAD LINEAL ------
        if buttons[self.BUTTON_RT] == 1 and self.last_buttons[self.BUTTON_RT] == 0:
            self.linear_speed = min(self.linear_speed + 0.05, self.linear_max)
            self.get_logger().info(f"Velocidad lineal: {self.linear_speed:.2f}")

        # ------ DISMINUIR VELOCIDAD LINEAL ------
        if buttons[self.BUTTON_RB] == 1 and self.last_buttons[self.BUTTON_RB] == 0:
            self.linear_speed = max(self.linear_speed - 0.05, self.linear_min)
            self.get_logger().info(f"Velocidad lineal: {self.linear_speed:.2f}")

        # ------ AUMENTAR VELOCIDAD ANGULAR ------
        if buttons[self.BUTTON_LT] == 1 and self.last_buttons[self.BUTTON_LT] == 0:
            self.angular_speed = min(self.angular_speed + 0.05, self.angular_max)
            self.get_logger().info(f"Velocidad angular: {self.angular_speed:.2f}")

        # ------ DISMINUIR VELOCIDAD ANGULAR ------
        if buttons[self.BUTTON_LB] == 1 and self.last_buttons[self.BUTTON_LB] == 0:
            self.angular_speed = max(self.angular_speed - 0.05, self.angular_min)
            self.get_logger().info(f"Velocidad angular: {self.angular_speed:.2f}")


def main():
    rclpy.init()
    node = AdvancedTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
