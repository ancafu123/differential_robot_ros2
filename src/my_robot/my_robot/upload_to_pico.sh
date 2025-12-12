#!/bin/bash
PORT="/dev/ttyACM0"
FILE="main.py"

echo "Subiendo $FILE al Pico W..."

# Matar procesos que puedan estar usando el puerto
sudo pkill -f "python.*$PORT" 2>/dev/null
sleep 1

# Subir archivo usando rshell
rshell -p $PORT "cp $FILE /pyboard/$FILE"

echo "Archivo transferido. Ejecutando..."

# Ejecutar el archivo
rshell -p $PORT "repl" << EOF
import $FILE
$FILE.main()
EOF

echo "Listo!"