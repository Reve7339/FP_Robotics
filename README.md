# Brazo Robótico Industrial de 3 GDL - Simulación y Control

Este proyecto consiste en el diseño, implementación de firmware y simulación tridimensional de un brazo robótico articulado industrial de **3 GDL** (Grados de Libertad) equipado con un cabezal de corte láser como efector final. La simulación está desarrollada sobre **ROS 2 Humble** y **Gazebo Classic** corriendo bajo un contenedor **Ubuntu 22.04 LTS (Distrobox)** en un sistema operativo base Manjaro Linux.

---

## 📂 Estructura del Proyecto

El repositorio está organizado de la siguiente manera:

*   **`ros2_ws/`**: Workspace de ROS 2 Humble.
    *   `src/arm_simulation/`: Paquete de simulación principal.
        *   `urdf/arm.urdf.xacro`: Descriptor cinemático del robot basado en el ABB IRB 120 (con articulaciones 4, 5 y 6 fijas).
        *   `meshes/`: Mallas STL oficiales de visualización y colisión del brazo robótico.
        *   `launch/`: Launch file para arrancar Gazebo Classic (`gazebo.launch.py`).
        *   `config/`: Archivo de configuración de los controladores posicionales (`controllers.yaml`).
        *   `scripts/trajectory_generator.py`: Generador de movimiento programado con interpolación suave (curva S).
*   **`esp32_firmware/`**: Firmware desarrollado en C (Arduino) para el microcontrolador ESP32 físico. Implementa la cinemática inversa, comunicación serial, Watchdog de seguridad y control de servomotores.
*   **`ui/`**: Estructura de archivos inicial (HTML, CSS, JS) para el panel web de control de usuario.
*   **`articulo.tex` / `articulo.pdf`**: Documento de investigación y reporte técnico del proyecto en formato IEEE LaTeX.

---

## 🛠️ Requisitos e Instalación

La simulación se ejecuta de manera aislada en un contenedor de Distrobox con Ubuntu 22.04 LTS para garantizar la compatibilidad con ROS 2 Humble.

### 1. Iniciar el Contenedor
Asegúrate de estar dentro del contenedor de Distrobox:
```bash
distrobox enter ros2-humble
```

### 2. Compilar el Workspace
Ingresa a la carpeta del workspace y compila el paquete de simulación:
```bash
cd ros2_ws
colcon build --packages-select arm_simulation
```

---

## 🚀 Ejecución de la Simulación

### 1. Lanzamiento en Gazebo (Simulación Física)
Para inicializar el servidor de física y ver el robot industrial naranja con su cabezal láser en 3D:
```bash
# Entrar al directorio del workspace
cd ros2_ws

# Limpiar el path de Miniconda para evitar conflictos de Python y ejecutar
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_simulation gazebo.launch.py
```

### 2. Ejecutar la Trayectoria Programada
Con Gazebo en ejecución, abre **otra terminal**, entra al contenedor, navega al workspace y ejecuta el script:
```bash
cd ros2_ws
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/arm_simulation/scripts/trajectory_generator.py
```



---

## ⚠️ Nota sobre Miniconda
Si utilizas **Miniconda** o **Anaconda** en tu sistema operativo base, sus rutas de Python interferirán dentro de tu contenedor Distrobox causando fallos al importar librerías de ROS 2 (`rclpy`). Los comandos anteriores incluyen un filtro temporal (`export PATH=...`) para remover la ruta de Miniconda antes de ejecutar los procesos de ROS 2.
