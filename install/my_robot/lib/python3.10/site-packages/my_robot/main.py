# ================= MicroPython – FIRMWARE PARA PICO W ==================

import machine
import utime
import sys
import select
import json
from math import pi

# ===== CONFIGURACION DE MOTORES =====
IN1 = machine.PWM(machine.Pin(26))  # Motor izquierdo adelante
IN2 = machine.PWM(machine.Pin(27))  # Motor izquierdo atrás
IN3 = machine.PWM(machine.Pin(16))  # Motor derecho adelante
IN4 = machine.PWM(machine.Pin(17))  # Motor derecho atrás

for m in (IN1, IN2, IN3, IN4):
    m.freq(1000)
    m.duty_u16(0)

# ===== CONFIGURACION ENCODERS =====
encA_A = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
encA_B = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)
encB_A = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)
encB_B = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)

# ===== CONSTANTES DEL ROBOT =====
WHEEL_RADIUS = 0.0325
WHEEL_DISTANCE = 0.15
MAX_LINEAR_SPEED = 0.5  # m/s
MAX_ANGULAR_SPEED = 2.0  # rad/s

# Configuracion PWM
PWM_MIN = 20000
PWM_MAX = 65535

# Control PID (valores aproximados, ajustar según necesidad)
KP = 0.8
KI = 0.1
KD = 0.05

# Variables de control
target_left_speed = 0.0
target_right_speed = 0.0
current_left_speed = 0.0
current_right_speed = 0.0

# Variables para encoders
encoder_left_count = 0
encoder_right_count = 0
last_encoder_left_A = 0
last_encoder_right_A = 0
last_encoder_time = utime.ticks_ms()

# Velocidades medidas (rad/s)
measured_left_speed = 0.0
measured_right_speed = 0.0

# ===== FUNCIONES DE ENCODERS =====
def encoder_left_handler(pin):
    global encoder_left_count, last_encoder_left_A
    current_A = encA_A.value()
    current_B = encA_B.value()
    
    if current_A != last_encoder_left_A:
        if current_A == current_B:
            encoder_left_count += 1
        else:
            encoder_left_count -= 1
        last_encoder_left_A = current_A

def encoder_right_handler(pin):
    global encoder_right_count, last_encoder_right_A
    current_A = encB_A.value()
    current_B = encB_B.value()
    
    if current_A != last_encoder_right_A:
        if current_A == current_B:
            encoder_right_count += 1
        else:
            encoder_right_count -= 1
        last_encoder_right_A = current_A

# Configurar interrupciones para encoders
encA_A.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=encoder_left_handler)
encB_A.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=encoder_right_handler)

# ===== FUNCIONES DE CONTROL DE MOTORES =====
def speed_to_pwm(speed):
    """Convierte velocidad lineal (m/s) a valor PWM"""
    # Normalizar velocidad
    norm_speed = speed / MAX_LINEAR_SPEED
    norm_speed = max(-1.0, min(1.0, norm_speed))
    
    if abs(norm_speed) < 0.05:  # Deadzone
        return 0, 0
    
    # Mapear a PWM
    pwm_value = int(PWM_MIN + abs(norm_speed) * (PWM_MAX - PWM_MIN))
    direction = 1 if norm_speed > 0 else -1
    
    return pwm_value, direction

def set_motor_speed(left_speed, right_speed):
    """Controla ambos motores con las velocidades especificadas"""
    # Motor izquierdo
    pwm_left, dir_left = speed_to_pwm(left_speed)
    if dir_left == 1:
        IN1.duty_u16(pwm_left)
        IN2.duty_u16(0)
    elif dir_left == -1:
        IN1.duty_u16(0)
        IN2.duty_u16(pwm_left)
    else:
        IN1.duty_u16(0)
        IN2.duty_u16(0)
    
    # Motor derecho
    pwm_right, dir_right = speed_to_pwm(right_speed)
    if dir_right == 1:
        IN3.duty_u16(pwm_right)
        IN4.duty_u16(0)
    elif dir_right == -1:
        IN3.duty_u16(0)
        IN4.duty_u16(pwm_right)
    else:
        IN3.duty_u16(0)
        IN4.duty_u16(0)

# ===== FUNCIONES DE CALCULO DE VELOCIDAD =====
def update_measured_speeds():
    """Calcula las velocidades actuales basadas en los encoders"""
    global encoder_left_count, encoder_right_count, last_encoder_time
    global measured_left_speed, measured_right_speed
    
    current_time = utime.ticks_ms()
    dt = utime.ticks_diff(current_time, last_encoder_time) / 1000.0  # segundos
    
    if dt > 0.01:  # Actualizar solo si ha pasado tiempo suficiente
        # Asumimos 20 ticks por revolución (ajustar según tu encoder)
        TICKS_PER_REVOLUTION = 20
        
        # Calcular velocidades angulares (rad/s)
        left_revs = encoder_left_count / TICKS_PER_REVOLUTION
        right_revs = encoder_right_count / TICKS_PER_REVOLUTION
        
        measured_left_speed = (left_revs * 2 * pi / dt) if dt > 0 else 0
        measured_right_speed = (right_revs * 2 * pi / dt) if dt > 0 else 0
        
        # Reset contadores
        encoder_left_count = 0
        encoder_right_count = 0
        last_encoder_time = current_time

# ===== CONTROL PID SIMPLIFICADO =====
def compute_motor_output(target_speed, measured_speed, last_error, integral):
    """Control PID simplificado para cada motor"""
    error = target_speed - measured_speed
    
    # Termino proporcional
    P = KP * error
    
    # Termino integral
    integral += error
    I = KI * integral
    I = max(-0.5, min(0.5, I))  # Limitar integral
    
    # Termino derivativo
    D = KD * (error - last_error)
    
    output = P + I + D
    output = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, output))
    
    return output, error, integral

# Variables para control PID
left_integral = 0
right_integral = 0
last_left_error = 0
last_right_error = 0

# ===== LOOP PRINCIPAL =====
last_cmd_time = utime.ticks_ms()
last_report_time = utime.ticks_ms()

def main_loop():
    global target_left_speed, target_right_speed
    global current_left_speed, current_right_speed
    global left_integral, right_integral, last_left_error, last_right_error
    global last_cmd_time, last_report_time
    
    while True:
        # ---------- LECTURA DE COMANDOS SERIAL ----------
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        
        if rlist:
            try:
                line = sys.stdin.readline().strip()
                if line:
                    data = json.loads(line)
                    v = float(data.get("v", 0))  # Velocidad lineal (m/s)
                    w = float(data.get("w", 0))  # Velocidad angular (rad/s)
                    
                    # Cinematica diferencial inversa
                    target_left_speed = v - (w * WHEEL_DISTANCE / 2)
                    target_right_speed = v + (w * WHEEL_DISTANCE / 2)
                    
                    # Limitar velocidades
                    target_left_speed = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, target_left_speed))
                    target_right_speed = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, target_right_speed))
                    
                    last_cmd_time = utime.ticks_ms()
                    
            except Exception as e:
                sys.stdout.write(json.dumps({"error": str(e)}) + "\n")
        
        # ---------- TIMEOUT DE SEGURIDAD ----------
        # Si no recibimos comandos por 1 segundo, paramos los motores
        if utime.ticks_diff(utime.ticks_ms(), last_cmd_time) > 1000:
            target_left_speed = 0.0
            target_right_speed = 0.0
        
        # ---------- ACTUALIZAR VELOCIDADES MEDIDAS ----------
        update_measured_speeds()
        
        # ---------- CONTROL PID ----------
        left_output, last_left_error, left_integral = compute_motor_output(
            target_left_speed, measured_left_speed, last_left_error, left_integral
        )
        
        right_output, last_right_error, right_integral = compute_motor_output(
            target_right_speed, measured_right_speed, last_right_error, right_integral
        )
        
        # ---------- APLICAR VELOCIDADES A MOTORES ----------
        set_motor_speed(left_output, right_output)
        
        # ---------- REPORTAR ESTADO ----------
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_report_time) >= 100:  # 10 Hz
            try:
                # Convertir a velocidades angulares para el bridge
                wl = measured_left_speed / WHEEL_RADIUS  # rad/s
                wr = measured_right_speed / WHEEL_RADIUS  # rad/s
                
                report = {
                    "wl": wl,
                    "wr": wr,
                    "target_left": target_left_speed,
                    "target_right": target_right_speed,
                    "measured_left": measured_left_speed,
                    "measured_right": measured_right_speed
                }
                sys.stdout.write(json.dumps(report) + "\n")
                last_report_time = current_time
            except Exception as e:
                pass
        
        utime.sleep_ms(10)

# ===== INICIALIZACION =====
if __name__ == "__main__":
    # Mensaje de inicio
    sys.stdout.write("Pico W Robot Controller iniciado\n")
    sys.stdout.write("Esperando comandos en formato JSON: {'v': 0.0, 'w': 0.0}\n")
    
    try:
        main_loop()
    except KeyboardInterrupt:
        # Apagar motores al salir
        set_motor_speed(0, 0)
        sys.stdout.write("Controller terminado. Motores apagados.\n")
    except Exception as e:
        set_motor_speed(0, 0)
        sys.stdout.write(f"Error: {e}\n")