#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import serial
import threading
import json
import time
import math

# TF2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200
READ_TIMEOUT = 0.1

WHEEL_RADIUS = 0.0325
WHEEL_BASE = 0.17

class PicoSerialBridge(Node):
    def __init__(self):
        super().__init__('pico_serial_bridge')
        self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=READ_TIMEOUT)
        self.get_logger().info(f"Serial abierto en {SERIAL_PORT} @ {BAUD}")
        
        # Subs y Pubs
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF Broadcaster - ¡ESTO ES LO QUE FALTA!
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.lock = threading.Lock()
        self.last_wl = 0.0
        self.last_wr = 0.0

        # odom pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # start thread to read serial
        t = threading.Thread(target=self.serial_read_loop, daemon=True)
        t.start()

        # timer to publish joint_states and odom at 20 Hz
        self.timer = self.create_timer(0.05, self.pub_states)

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        j = {"v": v, "w": w}
        line = json.dumps(j) + "\n"
        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error("Error escribiendo serial: " + str(e))

    def serial_read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                try:
                    j = json.loads(line)
                    if 'wl' in j and 'wr' in j:
                        with self.lock:
                            self.last_wl = float(j['wl'])
                            self.last_wr = float(j['wr'])
                except Exception as e:
                    self.get_logger().debug(f"JSON parse error: {e} - {line}")
            except Exception as e:
                self.get_logger().error("Error leyendo serial: " + str(e))
                time.sleep(0.1)

    def pub_states(self):
        tnow = self.get_clock().now()
        dt = (tnow - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        with self.lock:
            wl = self.last_wl
            wr = self.last_wr

        # Publicar joint_states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "base_link"  # Importante: frame_id para joint_states
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        
        if not hasattr(self, 'pos_l'):
            self.pos_l = 0.0
            self.pos_r = 0.0
            
        self.pos_l += wl * dt
        self.pos_r += wr * dt
        js.position = [self.pos_l, self.pos_r]
        js.velocity = [wl, wr]
        self.joint_pub.publish(js)

        # Odom: compute robot linear+angular velocities and integrate pose
        v = WHEEL_RADIUS * (wl + wr) / 2.0
        omega = WHEEL_RADIUS * (wr - wl) / WHEEL_BASE

        # integrate pose (simple euler)
        delta_x = v * math.cos(self.yaw) * dt
        delta_y = v * math.sin(self.yaw) * dt
        delta_yaw = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # PUBLICAR ODOMETRÍA
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'  # Frame padre
        odom.child_frame_id = 'base_link'  # Frame hijo
        
        # Pose en odom frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        qz = math.sin(self.yaw/2.0)
        qw = math.cos(self.yaw/2.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        # Covarianza (importante para navegación)
        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        # Twist en base_link frame
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega
        
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

        # ¡PUBLICAR TRANSFORM ODOM ? BASE_LINK!
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        self.last_time = tnow

def main(args=None):
    rclpy.init(args=args)
    node = PicoSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Cerrando puente serial...")
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()