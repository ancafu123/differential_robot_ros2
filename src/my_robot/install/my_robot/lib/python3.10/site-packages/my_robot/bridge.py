#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import json
import math
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Parametros
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('wheel_radius', 0.0325)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Conexion serial
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=0.1)
            time.sleep(2)  # Esperar a que la Pico se inicialice
            self.get_logger().info(f'Conectado a {serial_port} @ {baud_rate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error abriendo puerto serial: {e}')
            raise
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Variables de odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Timer para leer serial
        self.create_timer(0.01, self.read_serial_callback)  # 100Hz
        
        self.get_logger().info('Serial Bridge Node iniciado')
    
    def cmd_vel_callback(self, msg):
        """
        Recibe comandos de velocidad y los envia a la Pico
        """
        cmd_data = {
            "type": "cmd_vel",
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z
        }
        
        try:
            json_str = json.dumps(cmd_data) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Error escribiendo en serial: {e}')
    
    def read_serial_callback(self):
        """
        Lee datos de odometria de la Pico
        """
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                if line:
                    try:
                        data = json.loads(line)
                        
                        if data.get("type") == "odom":
                            self.process_odometry(
                                data.get("left", 0.0),
                                data.get("right", 0.0),
                                data.get("dt", 0.0)
                            )
                    except json.JSONDecodeError:
                        # Puede ser un mensaje de debug de la Pico
                        pass
                        
        except serial.SerialException as e:
            self.get_logger().error(f'Error leyendo serial: {e}')
    
    def process_odometry(self, dist_left, dist_right, dt):
        """
        Procesa datos de odometria y publica
        """
        if dt <= 0.0:
            return
        
        # Calcular desplazamiento
        dist_center = (dist_left + dist_right) / 2.0
        d_theta = (dist_right - dist_left) / self.wheel_separation
        
        # Actualizar posicion
        if abs(d_theta) < 1e-6:
            # Movimiento recto
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
        else:
            # Movimiento curvo
            radius = dist_center / d_theta
            self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + d_theta) - math.cos(self.theta))
        
        self.theta += d_theta
        
        # Normalizar ángulo
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calcular velocidades
        v = dist_center / dt
        omega = d_theta / dt
        
        # Publicar odometria
        self.publish_odometry(v, omega)
    
    def publish_odometry(self, v, omega):
        """
        Publica mensaje de odometria y TF
        """
        current_time = self.get_clock().now()
        
        # Crear quaternion desde yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        # Publicar TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publicar Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Posicion
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Velocidad
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        # Covarianzas
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[14] = 1000000.0
        odom.pose.covariance[21] = 1000000.0
        odom.pose.covariance[28] = 1000000.0
        odom.pose.covariance[35] = 0.01
        
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[7] = 0.001
        odom.twist.covariance[14] = 1000000.0
        odom.twist.covariance[21] = 1000000.0
        odom.twist.covariance[28] = 1000000.0
        odom.twist.covariance[35] = 0.01
        
        self.odom_pub.publish(odom)
    
    def destroy_node(self):
        """
        Cleanup al cerrar
        """
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            # Enviar comando de parada
            stop_cmd = {"type": "cmd_vel", "linear_x": 0.0, "angular_z": 0.0}
            self.serial_conn.write((json.dumps(stop_cmd) + '\n').encode('utf-8'))
            self.serial_conn.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SerialBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()