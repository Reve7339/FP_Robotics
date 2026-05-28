#!/bin/bash
# Inicia el servidor de la interfaz web y el puente de ROS 2 dentro del contenedor
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "Iniciando servidor web y puente ROS 2 para KiraOne en http://localhost:8080..."

# Intentar abrir el navegador por defecto
if command -v xdg-open &> /dev/null; then
  (sleep 1.5 && xdg-open "http://localhost:8080") &
elif command -v sensible-browser &> /dev/null; then
  (sleep 1.5 && sensible-browser "http://localhost:8080") &
fi

distrobox enter ros2-humble -- bash -c "
  export PATH=\$(echo \$PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/ros2_ws/install/setup.bash'
  python3 '$SCRIPT_DIR/ui/web_bridge.py'
"
