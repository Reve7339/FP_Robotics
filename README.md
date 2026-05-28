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

## 🛠️ Instalación y Configuración Automática

Para compilar el workspace de ROS 2, configurar los permisos y generar los accesos directos, solo necesitas ejecutar el script de configuración en la raíz del repositorio:

```bash
./setup.sh
```

---

## 🚀 Ejecución de la Simulación

Una vez completado el setup, puedes arrancar los diferentes componentes usando los scripts autogenerados:

### 1. Simulación en Gazebo Classic (Modelado 3D)
Para lanzar el simulador físico en 3D:
```bash
./launch_gazebo.sh
```

### 2. Generador de Trayectoria (Controlador de Movimiento)
Para ejecutar la trayectoria programada de corte por keyframes (abre en otra terminal):
```bash
./run_trajectory.sh
```

### 3. Panel de Control Web (KiraOne UI)
Para abrir la interfaz interactiva de usuario en tu navegador:
```bash
./start_ui.sh
```

---

## ⚠️ Notas de Integración
*   **Miniconda/Anaconda:** Los scripts de ejecución limpian automáticamente tu variable `PATH` de rutas de Conda en tiempo de ejecución para evitar conflictos de Python con ROS 2 (`rclpy`).
*   **Distrobox:** Todo el entorno de ROS 2 corre de forma aislada dentro del contenedor `ros2-humble` basado en Ubuntu 22.04 LTS.
