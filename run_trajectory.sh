#!/bin/bash
# Ejecuta el generador de trayectoria
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "Ejecutando generador de trayectoria (KiraOne)..."
distrobox enter ros2-humble -- bash -c "
  export PATH=\$(echo \$PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/ros2_ws/install/setup.bash'
  /usr/bin/python3 '$SCRIPT_DIR/ros2_ws/src/arm_simulation/scripts/trajectory_generator.py'
"
