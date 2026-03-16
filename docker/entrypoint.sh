#!/bin/bash
set -e

# 1. Cargar el entorno base de ROS Noetic
source /opt/ros/noetic/setup.bash

# 2. Cargar tu workspace local (si ya lo has compilado con catkin build)
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source "/root/catkin_ws/devel/setup.bash"
fi

# 3. Ejecutar el comando que le pase Docker (por defecto será abrir 'bash')
exec "$@"
