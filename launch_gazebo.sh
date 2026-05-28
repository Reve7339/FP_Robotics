#!/bin/bash
# Ejecuta la simulación en Gazebo Classic
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "Iniciando Gazebo Classic (KiraOne)..."
distrobox enter ros2-humble -- bash -c "
  export PATH=\$(echo \$PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/ros2_ws/install/setup.bash'
  ros2 launch arm_simulation gazebo.launch.py
"
