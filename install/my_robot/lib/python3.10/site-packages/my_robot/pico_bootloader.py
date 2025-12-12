#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import os
import threading
import time
import serial
import serial.tools.list_ports

class PicoBootloader(Node):
    def __init__(self):
        super().__init__("pico_bootloader")
        
        self.declare_parameter('main_path', '/home/andres/robot_diff/src/my_robot/my_robot/main.py')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('timeout', 8.0)
        self.declare_parameter('retry_count', 3)
        
        self.main_path = self.get_parameter('main_path').value
        self.port = self.get_parameter('port').value
        self.timeout = self.get_parameter('timeout').value
        self.retry_count = self.get_parameter('retry_count').value
        
        self.get_logger().info("Iniciando bootloader del Pico W...")
        
        # Ejecutar en hilo separado
        self.load_thread = threading.Thread(target=self.load_to_pico)
        self.load_thread.start()

    def is_pico_connected(self, port=None):
        """Verifica si el Pico esta conectado y disponible"""
        test_port = port or self.port
        
        if not os.path.exists(test_port):
            self.get_logger().warning(f"Puerto {test_port} no existe")
            return False
            
        try:
            # Intentar abrir el puerto
            with serial.Serial(test_port, 115200, timeout=1) as ser:
                time.sleep(0.5)
                ser.write(b'\x03')  # Ctrl+C para limpiar REPL
                time.sleep(0.5)
                ser.write(b'\x04')  # Ctrl+D para soft reset
                time.sleep(1)
            return True
        except Exception as e:
            self.get_logger().warning(f"Puerto {test_port} no accesible: {e}")
            return False

    def find_pico_port(self):
        """Busca automaticamente el puerto del Pico"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'Pico' in port.description or 'ACM' in port.device:
                self.get_logger().info(f"Pico encontrado en: {port.device}")
                return port.device
        return None

    def run_mpremote_cmd(self, args, timeout=8.0):
        """Ejecuta comando mpremote con mejor manejo"""
        cmd = ["mpremote", "connect", self.port] + args
        self.get_logger().info(f"Ejecutando: {' '.join(cmd)}")
        
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            stdout, stderr = process.communicate(timeout=timeout)
            
            if stdout:
                self.get_logger().info(f"Output: {stdout.strip()}")
            if stderr:
                self.get_logger().warning(f"Stderr: {stderr.strip()}")
                
            return process.returncode == 0
            
        except subprocess.TimeoutExpired:
            self.get_logger().error("Timeout - matando proceso...")
            process.kill()
            stdout, stderr = process.communicate()
            return False
        except Exception as e:
            self.get_logger().error(f"Error ejecutando comando: {e}")
            return False

    def cleanup_previous_sessions(self):
        """Limpia sesiones previas de mpremote"""
        self.get_logger().info("Limpiando sesiones previas...")
        try:
            subprocess.run(['pkill', '-f', 'mpremote'], 
                         capture_output=True, timeout=5)
            time.sleep(5)
            # También limpiar procesos de serial
            subprocess.run(['sudo', 'fuser', '-k', self.port], 
                         capture_output=True, timeout=5)
            time.sleep(5)
        except Exception as e:
            self.get_logger().warning(f"Error en limpieza: {e}")

    def load_to_pico(self):
        """Carga el archivo al Pico con reintentos"""
        time.sleep(2)  # Esperar inicialización de ROS
        
        # Verificar que el archivo existe
        if not os.path.exists(self.main_path):
            self.get_logger().error(f"Archivo {self.main_path} no encontrado!")
            return False
        
        # Buscar Pico automaticamente si el puerto no funciona
        if not self.is_pico_connected():
            self.get_logger().warning("Puerto configurado no disponible, buscando...")
            found_port = self.find_pico_port()
            if found_port:
                self.port = found_port
                self.get_logger().info(f"Usando puerto: {self.port}")
            else:
                self.get_logger().error("No se encontro ningun Pico conectado")
                return False
        
        # Limpiar sesiones previas
        self.cleanup_previous_sessions()
        
        # Esperar a que el Pico este listo
        self.get_logger().info("Esperando a que el Pico este listo...")
        time.sleep(3)
        
        # Intentar copiar con reintentos
        for attempt in range(self.retry_count):
            self.get_logger().info(f"Intento {attempt + 1} de {self.retry_count}")
            
            # 1. Copiar main.py
            if self.run_mpremote_cmd(["cp", self.main_path, ":main.py"], timeout=self.timeout):
                self.get_logger().info("Archivo copiado exitosamente")
                
                # 2. Verificar que se copio
                time.sleep(1)
                if self.run_mpremote_cmd(["ls", ":"], timeout=5.0):
                    self.get_logger().info("Archivos listados correctamente")
                    
                    # 3. Reiniciar Pico
                    if self.run_mpremote_cmd(["reset"], timeout=5.0):
                        self.get_logger().info("Pico reiniciado exitosamente")
                        return True
                    else:
                        self.get_logger().warning("Reinicio fallo, pero archivo puede estar cargado")
                        return True
                
            self.get_logger().warning(f"Intento {attempt + 1} fallado, reintentando...")
            time.sleep(2)
        
        self.get_logger().error("? Todos los intentos fallaron")
        return False

def main(args=None):
    rclpy.init(args=args)
    
    node = PicoBootloader()
    
    # Esperar maximo 45 segundos
    max_wait = 45.0
    start_time = time.time()
    
    while time.time() - start_time < max_wait and node.load_thread.is_alive():
        rclpy.spin_once(node, timeout_sec=1.0)
        time.sleep(0.5)
    
    if node.load_thread.is_alive():
        node.get_logger().warning("Timeout esperando por la carga")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()