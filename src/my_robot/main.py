from machine import Pin, PWM
import time
import json
import sys
import select

# ========== CONFIGURACION DE PINES ==========
# Motor A (Izquierdo)
AIN1 = PWM(Pin(26))
AIN2 = PWM(Pin(27))
AIN1.freq(1000)
AIN2.freq(1000)

# Motor B (Derecho)
BIN1 = PWM(Pin(16))
BIN2 = PWM(Pin(17))
BIN1.freq(1000)
BIN2.freq(1000)

# Encoders
encA_A = Pin(18, Pin.IN, Pin.PULL_UP)
encA_B = Pin(19, Pin.IN, Pin.PULL_UP)
encB_A = Pin(20, Pin.IN, Pin.PULL_UP)
encB_B = Pin(21, Pin.IN, Pin.PULL_UP)

# ========== PARAMETROS DEL ROBOT ==========
PULSES_PER_REV = 2472  # Cuadratura x4
WHEEL_RADIUS = 0.0325  # metros
WHEEL_SEPARATION = 0.17  # metros
MAX_PWM = 65535

# Variables globales
encoder_left = 0
encoder_right = 0
last_time = time.ticks_ms()

# ========== FUNCIONES DE ENCODERS ==========
def encoder_left_callback(pin):
    global encoder_left
    if encA_B.value():
        encoder_left += 1
    else:
        encoder_left -= 1

def encoder_right_callback(pin):
    global encoder_right
    if encB_B.value():
        encoder_right += 1
    else:
        encoder_right -= 1

# Interrupciones de encoders
encA_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_left_callback)
encB_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_right_callback)

# ========== CONTROL DE MOTORES ==========
def set_motor_left(speed):
    """
    speed: -1.0 a 1.0
    Negativo = reversa, Positivo = adelante
    """
    pwm_value = int(abs(speed) * MAX_PWM)
    pwm_value = min(pwm_value, MAX_PWM)
    
    if speed > 0:
        AIN1.duty_u16(pwm_value)
        AIN2.duty_u16(0)
    elif speed < 0:
        AIN1.duty_u16(0)
        AIN2.duty_u16(pwm_value)
    else:
        AIN1.duty_u16(0)
        AIN2.duty_u16(0)

def set_motor_right(speed):
    """
    speed: -1.0 a 1.0
    """
    pwm_value = int(abs(speed) * MAX_PWM)
    pwm_value = min(pwm_value, MAX_PWM)
    
    if speed > 0:
        BIN1.duty_u16(pwm_value)
        BIN2.duty_u16(0)
    elif speed < 0:
        BIN1.duty_u16(0)
        BIN2.duty_u16(pwm_value)
    else:
        BIN1.duty_u16(0)
        BIN2.duty_u16(0)

def stop_motors():
    set_motor_left(0)
    set_motor_right(0)

# ========== CINEMATICA DIFERENCIAL ==========
def cmd_vel_to_wheel_speeds(linear_x, angular_z):
    """
    Convierte cmd_vel a velocidades de ruedas
    linear_x: m/s
    angular_z: rad/s
    Retorna: (velocidad_izq, velocidad_der) en m/s
    """
    # Cinematica diferencial
    vel_left = linear_x - (angular_z * WHEEL_SEPARATION / 2.0)
    vel_right = linear_x + (angular_z * WHEEL_SEPARATION / 2.0)
    
    return vel_left, vel_right

def wheel_speed_to_pwm(vel_mps):
    """
    Convierte velocidad en m/s a PWM (-1.0 a 1.0)
    Velocidad maxima estimada: ~0.5 m/s
    """
    MAX_SPEED_MPS = 0.5  # Ajustar segun tu robot
    pwm = vel_mps / MAX_SPEED_MPS
    pwm = max(-1.0, min(1.0, pwm))  # Limitar
    return pwm

# ========== ODOMETRIA ==========
def calculate_odometry():
    """
    Calcula distancia recorrida por cada rueda
    Retorna: (distancia_izq, distancia_der, dt) en metros y segundos
    """
    global encoder_left, encoder_right, last_time
    
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_time) / 1000.0  # segundos
    last_time = current_time
    
    # Distancia = (pulsos / pulsos_por_rev) * circunferencia
    circumference = 2 * 3.14159 * WHEEL_RADIUS
    
    dist_left = (encoder_left / PULSES_PER_REV) * circumference
    dist_right = (encoder_right / PULSES_PER_REV) * circumference
    
    # Reset encoders
    encoder_left = 0
    encoder_right = 0
    
    return dist_left, dist_right, dt

# ========== COMUNICACION SERIAL ==========
def read_serial_non_blocking():
    """
    Lee linea completa de serial sin bloquear
    """
    poll_obj = select.poll()
    poll_obj.register(sys.stdin, select.POLLIN)
    
    if poll_obj.poll(0):  # 0 = non-blocking
        line = sys.stdin.readline().strip()
        return line
    return None

def send_odometry(dist_left, dist_right, dt):
    """
    Envia odometria por serial en formato JSON
    """
    data = {
        "type": "odom",
        "left": dist_left,
        "right": dist_right,
        "dt": dt
    }
    print(json.dumps(data))

# ========== LOOP PRINCIPAL ==========
def main():
    print("Pico W Motor Controller Started")
    print("Waiting for commands...")
    
    last_odom_time = time.ticks_ms()
    ODOM_INTERVAL_MS = 50  # Enviar odometria cada 50ms (20Hz)
    
    watchdog_timer = time.ticks_ms()
    WATCHDOG_TIMEOUT_MS = 500  # Detener motores si no hay comando en 500ms
    
    try:
        while True:
            # Leer comando serial
            line = read_serial_non_blocking()
            
            if line:
                try:
                    cmd = json.loads(line)
                    
                    if cmd.get("type") == "cmd_vel":
                        linear_x = cmd.get("linear_x", 0.0)
                        angular_z = cmd.get("angular_z", 0.0)
                        
                        # Convertir a velocidades de ruedas
                        vel_left, vel_right = cmd_vel_to_wheel_speeds(linear_x, angular_z)
                        
                        # Convertir a PWM
                        pwm_left = wheel_speed_to_pwm(vel_left)
                        pwm_right = wheel_speed_to_pwm(vel_right)
                        
                        # Aplicar a motores
                        set_motor_left(pwm_left)
                        set_motor_right(pwm_right)
                        
                        # Reset watchdog
                        watchdog_timer = time.ticks_ms()
                        
                except ValueError:
                    print("Error parsing JSON")
            
            # Watchdog - detener motores si no hay comandos
            if time.ticks_diff(time.ticks_ms(), watchdog_timer) > WATCHDOG_TIMEOUT_MS:
                stop_motors()
            
            # Enviar odometria periodicamente
            if time.ticks_diff(time.ticks_ms(), last_odom_time) >= ODOM_INTERVAL_MS:
                dist_left, dist_right, dt = calculate_odometry()
                send_odometry(dist_left, dist_right, dt)
                last_odom_time = time.ticks_ms()
            
            time.sleep_ms(5)  # Small delay
            
    except KeyboardInterrupt:
        print("Stopping...")
        stop_motors()

if __name__ == "__main__":
    main()