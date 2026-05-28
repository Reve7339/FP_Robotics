#!/bin/bash

# KiraOne Setup Automation Script
# Sets up the ROS2 Workspace inside distrobox, configures permissions, and creates easy-to-use runners.

set -e

# Color codes for clean interface
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}===============================================${NC}"
echo -e "${GREEN}      KiraOne - SETUP & COMPILATION SYSTEM     ${NC}"
echo -e "${BLUE}===============================================${NC}"

# 1. Verify distrobox is installed
if ! command -v distrobox &> /dev/null; then
    echo -e "${RED}Error: distrobox no está instalado en el sistema host.${NC}"
    exit 1
fi

# 2. Check if ros2-humble container exists
echo -e "${BLUE}[1/4] Verificando contenedor Distrobox...${NC}"
if ! distrobox list | grep -q "ros2-humble"; then
    echo -e "${YELLOW}Advertencia: El contenedor 'ros2-humble' no existe o no está corriendo.${NC}"
    echo -e "Intentando crear el contenedor con Ubuntu 22.04..."
    distrobox create -n ros2-humble -i ubuntu:22.04 -y
fi
echo -e "${GREEN}✔ Contenedor 'ros2-humble' detectado.${NC}"

# 3. Compile ROS2 workspace inside distrobox
echo -e "${BLUE}[2/4] Compilando Workspace de ROS2 dentro de 'ros2-humble'...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

distrobox enter ros2-humble -- bash -c "
  cd '$SCRIPT_DIR/ros2_ws' && \
  source /opt/ros/humble/setup.bash && \
  colcon build --packages-select arm_simulation
"

echo -e "${GREEN}✔ Workspace compilado exitosamente.${NC}"

# 4. Make Python scripts executable
echo -e "${BLUE}[3/4] Configurando permisos de ejecución...${NC}"
chmod +x "$SCRIPT_DIR/ros2_ws/src/arm_simulation/scripts/trajectory_generator.py"
echo -e "${GREEN}✔ Permisos configurados.${NC}"

# 5. Generate Runner scripts for a premium experience
echo -e "${BLUE}[4/4] Generando scripts de ejecución rápida...${NC}"

# Create launch_gazebo.sh
cat << 'EOF' > "$SCRIPT_DIR/launch_gazebo.sh"
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
EOF
chmod +x "$SCRIPT_DIR/launch_gazebo.sh"

# Create run_trajectory.sh
cat << 'EOF' > "$SCRIPT_DIR/run_trajectory.sh"
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
EOF
chmod +x "$SCRIPT_DIR/run_trajectory.sh"
# Create start_ui.sh
cat << 'EOF' > "$SCRIPT_DIR/start_ui.sh"
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
EOF
chmod +x "$SCRIPT_DIR/start_ui.sh"

echo -e "${GREEN}✔ Scripts de ejecución rápida generados.${NC}"
echo -e "${BLUE}===============================================${NC}"
echo -e "${GREEN}        KiraOne LISTO PARA USAR                ${NC}"
echo -e "${BLUE}===============================================${NC}"
echo -e "Puedes iniciar la simulación y control usando:"
echo -e "  1. ${YELLOW}./launch_gazebo.sh${NC}  - Para lanzar Gazebo"
echo -e "  2. ${YELLOW}./run_trajectory.sh${NC} - Para ejecutar la trayectoria programada"
echo -e "  3. ${YELLOW}./start_ui.sh${NC}       - Para iniciar la UI web e interactuar"
echo -e "${BLUE}===============================================${NC}"
