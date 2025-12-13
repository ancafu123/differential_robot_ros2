# differential_robot_ros2
Robot autónomo ROS2 Humble, SLAM + Nav2 

[![Lenguaje C++](https://img.shields.io/badge/C++-17-blue)](#)
[![Lenguaje Python](https://img.shields.io/badge/Python-3.8+-yellow?logo=python)](#)
[![Sistema Operativo](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange?logo=gazebo)](#)
[![CMake](https://img.shields.io/badge/CMake-3.16+-064F8C?logo=cmake)](#)
[![Colcon](https://img.shields.io/badge/Build-Colcon-22314E)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![Git](https://img.shields.io/badge/Git-2.34+-F05032?logo=git)](#)
[![VS Code](https://img.shields.io/badge/IDE-VS%20Code-007ACC?logo=visualstudiocode)](#)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25?logo=gnubash)](#)
<!-- [![Docker](https://img.shields.io/badge/Container-Docker-2496ED?logo=docker)](#) -->
<!-- [![Docker Compose](https://img.shields.io/badge/Docker--Compose-Blue?logo=docker)](#) -->
[![Arquitectura](https://img.shields.io/badge/CPU-x86_64%20%7C%20ARM64-lightgrey?logo=amd)](#)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)
[![Versión Actual](https://img.shields.io/badge/Versión-v1.0.0-blue)](#)


# Proyecto de Robot Móvil con RPLIDAR y ROS2

Este repositorio documenta el desarrollo completo de un **robot móvil diferencial** basado en **ROS 2 Humble**, equipado con un **RPLIDAR A1**. Incluye la integración de hardware, configuración de drivers, pruebas en RViz2 y todo el proceso de puesta en marcha.

---

## 📌 1. Descripción general del proyecto

El proyecto consiste en la implementación de un robot móvil que utiliza:

* Raspberry Pi 4
* RPLIDAR A1
* Motores DC con encoder
* ROS2 Humble + SLAM Toolbox + Nav2
* Nodo personalizado para movimiento
* Configuración completa de sensores

El objetivo es lograr navegación básica, lectura de LIDAR, visualización en RViz2, y pruebas iniciales de SLAM.

---

## 📦 2. Estructura del repositorio

```
├── launch/                 # Archivos .launch.py
├── config/                 # Archivos YAML de parámetros
├── src/                    # Nodos personalizados
├── urdf/                   # Modelos del robot
├── scripts/                # Scripts auxiliares
├── README.md               # Este documento
└── package.xml / CMakeLists.txt
```

---

## 🔧 3. Instalación de dependencias

### ROS 2 Humble

Instalar ROS2 Humble:

```
sudo apt install ros-humble-desktop
```

### RPLIDAR ROS2

```
sudo apt install ros-humble-rplidar-ros
```

### Dependencias adicionales

```
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-xacro
```

---

## 🛠️ 4. Configuración del RPLIDAR

### Verificar puerto:

```
ls /dev/ttyUSB*
```

### Dar permisos

```
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
```

(Reiniciar sesión)

### Archivo YAML

```
serial_port: /dev/ttyUSB0
serial_baudrate: 115200   # 256000 si es A2/A3/S2
frame_id: laser
```

---

## 🚀 5. Ejecución del nodo RPLIDAR

```
ros2 launch rplidar_ros rplidar.launch.py
```

Comprobación directa:

```
ros2 run rplidar_ros rplidar_node
```

---

## 🛰️ 6. Visualización en RViz2

```
rviz2
```

Agregar:

* LaserScan
* TF
* Odometry
* RobotModel

### Errores comunes (y soluciones)

Se incluyen en el apartado final del documento.

---

## 🤖 7. URDF del robot

El robot usa un modelo URDF que define:

* Base del robot
* Ruedas
* Chasis
* Sensor LIDAR
* Cámara
* Coordenadas TF

Ejemplo de ejecución del URDF:

```
ros2 launch robot_description display.launch.py
```

---

## 🧭 8. Navegación y SLAM (Nav2)

Para usar SLAM Toolbox:

```
sudo apt install ros-humble-slam-toolbox
```

Para ejecutar navegación:

```
ros2 launch nav2_bringup navigation_launch.py
```

---

## 🧰 9. Solución de errores frecuentes

### ❌ RPLIDAR: exit code -6

Causas:

* Puerto incorrecto
* Permisos insuficientes
* Baudrate incorrecto
* Driver USB ocupado

### ❌ RViz: `QXcbConnection XCB error`

Causas:

* Problemas con GPU
* Ejecutar en Wayland en vez de Xorg
* SSH sin aceleración gráfica

Solución recomendada: usar **Ubuntu on Xorg**.

---

## 🧪 10. Pruebas realizadas

* Prueba de lectura continua del LIDAR
* Publicación de /scan en ROS2
* Generación de mapa inicial con SLAM
* Visualización completa en RViz2
* Prueba de movimiento básico del robot

---

## 📄 11. Licencia

MIT License.

---

## 👤 Autor

**Andrés Castellanos**
