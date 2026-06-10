import http.server
import json
import threading
import sys
import os
import math
import time
import io

# Importación opcional de pyserial para comunicación con el ESP32 físico
HAS_SERIAL = False
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    pass

# Intento de importación dinámica para OpenCV o Pillow
HAS_OPENCV = False
HAS_PILLOW = False

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    try:
        from PIL import Image as PILImage
        HAS_PILLOW = True
    except ImportError:
        pass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity
from gazebo_msgs.msg import LinkStates


def log_message(msg):
    try:
        with open("/home/reve/Documents/UC/robotica/finalProject/web_bridge.log", "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except Exception:
        pass


class SerialManager:
    """
    Gestiona la conexión serial con el ESP32 físico de forma segura y thread-safe.
    Auto-descubre el puerto del ESP32 buscando por VID/PID comunes de CP210x y CH340.
    """
    ESP32_VIDS = {0x10C4, 0x1A86, 0x0403, 0x2341}  # CP210x, CH340, FTDI, Arduino

    def __init__(self):
        self._lock = threading.Lock()
        self._ser = None
        self._port = None
        self._connected = False
        self._last_response = None
        self._last_error = None

    def _find_esp32_port(self):
        """Detecta automáticamente el puerto serie del ESP32."""
        if not HAS_SERIAL:
            return None
        for port in serial.tools.list_ports.comports():
            if port.vid in self.ESP32_VIDS:
                return port.device
        # Fallback: buscar por nombre de dispositivo típico
        for port in serial.tools.list_ports.comports():
            dev = port.device.lower()
            if 'ttyusb' in dev or 'ttyacm' in dev or 'cu.usbserial' in dev or 'cu.wchusbserial' in dev:
                return port.device
        return None

    def connect(self, port=None, baudrate=115200):
        """Abre la conexión serial. Si port es None, auto-detecta el ESP32."""
        if not HAS_SERIAL:
            self._last_error = "pyserial no instalado. Ejecuta: pip install pyserial"
            return False
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
                target_port = port or self._find_esp32_port()
                if not target_port:
                    self._last_error = "ESP32 no encontrado. Conecta el USB y verifica los permisos (sudo usermod -a -G dialout $USER)."
                    self._connected = False
                    return False
                self._ser = serial.Serial(target_port, baudrate, timeout=1.0)
                self._port = target_port
                self._connected = True
                self._last_error = None
                log_message(f"[Serial] Conectado al ESP32 en {target_port} a {baudrate} bps")
                print(f"[Serial] Conectado al ESP32 en {target_port} a {baudrate} bps")
                sys.stdout.flush()
                return True
            except Exception as e:
                self._last_error = str(e)
                self._connected = False
                log_message(f"[Serial] Error de conexión: {e}")
                return False

    def disconnect(self):
        """Cierra la conexión serial."""
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self._connected = False
            self._port = None

    def send_cartesian(self, x_mm, y_mm, z_mm):
        """
        Envía un comando cartesiano al ESP32 en el formato: X:val,Y:val,Z:val\n
        El ESP32 resuelve la IK internamente y mueve los servos.
        Retorna (ok: bool, response: str).
        """
        if not HAS_SERIAL:
            return False, "pyserial no disponible"
        with self._lock:
            if not self._connected or not self._ser or not self._ser.is_open:
                return False, "No conectado al ESP32"
            try:
                cmd = f"X:{x_mm:.2f},Y:{y_mm:.2f},Z:{z_mm:.2f}\n"
                self._ser.write(cmd.encode('utf-8'))
                self._ser.flush()
                # Leer respuesta con timeout
                resp = self._ser.readline().decode('utf-8', errors='replace').strip()
                self._last_response = resp
                ok = resp.startswith('OK')
                if not ok:
                    log_message(f"[Serial] Respuesta ESP32: {resp}")
                return ok, resp
            except Exception as e:
                self._last_error = str(e)
                self._connected = False
                log_message(f"[Serial] Error al enviar comando: {e}")
                return False, str(e)

    @property
    def status(self):
        """Retorna el estado de conexión como diccionario para la API."""
        return {
            "connected": self._connected,
            "port": self._port,
            "has_serial": HAS_SERIAL,
            "last_error": self._last_error,
            "last_response": self._last_response,
        }


# Instancia global del gestor serial
serial_manager = SerialManager()


def zhang_suen_thinning(img):
    """
    Algoritmo de esqueletización de Zhang-Suen para reducir una imagen binaria
    a líneas de 1 píxel de espesor de forma rápida y vectorizada con NumPy.
    """
    import numpy as np
    im = (img > 0).astype(np.uint8)
    im = np.pad(im, 1, mode='constant', constant_values=0)
    
    while True:
        # Paso 1
        p2 = im[:-2, 1:-1]
        p3 = im[:-2, 2:]
        p4 = im[1:-1, 2:]
        p5 = im[2:, 2:]
        p6 = im[2:, 1:-1]
        p7 = im[2:, :-2]
        p8 = im[1:-1, :-2]
        p9 = im[:-2, :-2]
        
        B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
        
        A = ((p2 == 0) & (p3 == 1)).astype(np.uint8) + \
            ((p3 == 0) & (p4 == 1)).astype(np.uint8) + \
            ((p4 == 0) & (p5 == 1)).astype(np.uint8) + \
            ((p5 == 0) & (p6 == 1)).astype(np.uint8) + \
            ((p6 == 0) & (p7 == 1)).astype(np.uint8) + \
            ((p7 == 0) & (p8 == 1)).astype(np.uint8) + \
            ((p8 == 0) & (p9 == 1)).astype(np.uint8) + \
            ((p9 == 0) & (p2 == 1)).astype(np.uint8)
            
        cond1 = (B >= 2) & (B <= 6) & (A == 1) & (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
        inner = im[1:-1, 1:-1]
        to_delete_1 = (inner == 1) & cond1
        
        im[1:-1, 1:-1] = np.where(to_delete_1, 0, inner)
        
        # Paso 2
        p2 = im[:-2, 1:-1]
        p3 = im[:-2, 2:]
        p4 = im[1:-1, 2:]
        p5 = im[2:, 2:]
        p6 = im[2:, 1:-1]
        p7 = im[2:, :-2]
        p8 = im[1:-1, :-2]
        p9 = im[:-2, :-2]
        
        B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
        A = ((p2 == 0) & (p3 == 1)).astype(np.uint8) + \
            ((p3 == 0) & (p4 == 1)).astype(np.uint8) + \
            ((p4 == 0) & (p5 == 1)).astype(np.uint8) + \
            ((p5 == 0) & (p6 == 1)).astype(np.uint8) + \
            ((p6 == 0) & (p7 == 1)).astype(np.uint8) + \
            ((p7 == 0) & (p8 == 1)).astype(np.uint8) + \
            ((p8 == 0) & (p9 == 1)).astype(np.uint8) + \
            ((p9 == 0) & (p2 == 1)).astype(np.uint8)
            
        cond2 = (B >= 2) & (B <= 6) & (A == 1) & (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
        inner = im[1:-1, 1:-1]
        to_delete_2 = (inner == 1) & cond2
        
        im[1:-1, 1:-1] = np.where(to_delete_2, 0, inner)
        
        if not np.any(to_delete_1) and not np.any(to_delete_2):
            break
            
    return (im[1:-1, 1:-1] * 255).astype(np.uint8)


class WebBridgeNode(Node):
    """
    Nodo de ROS 2 que expone una interfaz de comunicación para recibir consignas angulares
    desde clientes externos y publicarlas en el simulador.
    """
    def __init__(self):
        """
        Inicializa el nodo, configura el publicador en el tópico de control del brazo
        y el cliente de servicio para interactuar con entidades de Gazebo.
        """
        super().__init__('web_bridge_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.latest_jpeg = None
        self.jpeg_lock = threading.Lock()
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.conveyor_active = False
        self.conveyor_target_move = 0.0
        self.cylinder_positions = {
            'cylinder_A1': 0.0,
            'cylinder_D': -0.375,
            'cylinder_R': -0.75,
            'cylinder_I': -1.125,
            'cylinder_A2': -1.50,
            'cylinder_N': -1.875,
            'cylinder_A3': -2.25
        }
        self.belt_center_y = 0.0
        self.conveyor_lock = threading.Lock()
        self.start_conveyor_thread()
        self.flange_pose = None
        self.laser_active = False
        self.laser_was_active = False
        self.laser_lock = threading.Lock()
        self.link_states_subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_states_callback,
            10
        )
        self.start_laser_thread()
        self.get_logger().info('WebBridge ROS 2 Node iniciado. Suscrito a /camera/image_raw')

    def image_callback(self, msg):
        """
        Recibe fotogramas de la cámara virtual de Gazebo, los convierte a formato
        JPEG de forma asíncrona y los almacena en caché.
        """
        global HAS_OPENCV, HAS_PILLOW
        try:
            channels = 4 if 'a' in msg.encoding.lower() or '4' in msg.encoding else 3
            if HAS_OPENCV:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
                if channels == 4:
                    if 'bgr' in msg.encoding.lower():
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                    else:
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                else:
                    if 'bgr' in msg.encoding.lower():
                        img_bgr = img
                    else:
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                ret, jpeg = cv2.imencode('.jpg', img_bgr)
                if ret:
                    with self.jpeg_lock:
                        self.latest_jpeg = jpeg.tobytes()
            elif HAS_PILLOW:
                mode = 'RGBA' if channels == 4 else 'RGB'
                # En caso de que sea BGR/BGRA, Pillow lee los canales directamente,
                # por lo que los colores podrían verse invertidos (azul/rojo), pero
                # al menos mostrará la imagen. La mayoría de cámaras ROS son RGB.
                img = PILImage.frombytes(mode, (msg.width, msg.height), msg.data)
                if channels == 4:
                    img = img.convert('RGB')
                fp = io.BytesIO()
                img.save(fp, format='JPEG')
                with self.jpeg_lock:
                    self.latest_jpeg = fp.getvalue()
        except Exception as e:
            self.get_logger().error(f'Error en image_callback: {e}')

    def start_conveyor_thread(self):
        self.conveyor_thread = threading.Thread(target=self.conveyor_loop, daemon=True)
        self.conveyor_thread.start()

    def conveyor_loop(self):
        rate = 20.0  # 20 Hz
        dt = 1.0 / rate
        velocity = 0.04  # 4 cm/s
        was_moving = False
        
        while rclpy.ok():
            try:
                with self.conveyor_lock:
                    active = self.conveyor_active
                    
                if active:
                    if not was_moving:
                        log_message("[conveyor_loop] Iniciando movimiento continuo de la cinta.")
                        was_moving = True
                    
                    step = velocity * dt
                    
                    with self.conveyor_lock:
                        for name in self.cylinder_positions:
                            y = self.cylinder_positions[name]
                            y += step
                            if y > 0.75:
                                y -= 2.625
                            self.cylinder_positions[name] = y
                            
                        positions_copy = dict(self.cylinder_positions)
                        
                    # Realizar llamadas de servicio fuera del lock para evitar lock contention y deadlocks
                    for name in positions_copy:
                        y = positions_copy[name]
                        # Update in Gazebo asynchronously for fluid movement during progress
                        if self.set_entity_state_client.service_is_ready():
                            req = SetEntityState.Request()
                            req.state.name = name
                            req.state.reference_frame = 'world'
                            req.state.pose.position.x = 0.50
                            req.state.pose.position.y = y
                            req.state.pose.position.z = 0.055
                            req.state.pose.orientation.w = 1.0
                            self.set_entity_state_client.call_async(req)
                else:
                    if was_moving:
                        log_message("[conveyor_loop] Deteniendo movimiento continuo de la cinta. Realizando sincronización final.")
                        was_moving = False
                        
                        with self.conveyor_lock:
                            positions_copy = dict(self.cylinder_positions)
                            
                        # Esperar un momento para que se procesen las llamadas asíncronas intermedias pendientes
                        time.sleep(0.1)
                        futures = []
                        for name in positions_copy:
                            if self.set_entity_state_client.service_is_ready():
                                req = SetEntityState.Request()
                                req.state.name = name
                                req.state.reference_frame = 'world'
                                req.state.pose.position.x = 0.50
                                req.state.pose.position.y = positions_copy[name]
                                req.state.pose.position.z = 0.055
                                req.state.pose.orientation.w = 1.0
                                try:
                                    log_message(f"[conveyor_loop] Enviando pose final para {name} a y={positions_copy[name]:.6f}")
                                    future = self.set_entity_state_client.call_async(req)
                                    futures.append((name, future))
                                except Exception as ex:
                                    log_message(f"[conveyor_loop] Error en llamada final para {name}: {ex}")
                                    
                        # Espera no bloqueante en paralelo con timeout para todas las llamadas finales
                        start_wait = time.time()
                        while futures and rclpy.ok():
                            futures = [(name, fut) for name, fut in futures if not fut.done()]
                            if not futures:
                                break
                            time.sleep(0.01)
                            if time.time() - start_wait > 0.5:
                                pending_names = [name for name, _ in futures]
                                log_message(f"[conveyor_loop] Timeout esperando respuesta de Gazebo para: {pending_names}")
                                break
            except Exception as e:
                log_message(f"[conveyor_loop] EXCEPCIÓN: {e}")
            time.sleep(dt)

    def link_states_callback(self, msg):
        try:
            if 'arm_3gdl::flange' in msg.name:
                idx = msg.name.index('arm_3gdl::flange')
                with self.laser_lock:
                    self.flange_pose = msg.pose[idx]
            
            # Sincronizar posición del centro de la cinta
            if 'conveyor_belt::belt_center_x' in msg.name:
                idx = msg.name.index('conveyor_belt::belt_center_x')
                with self.conveyor_lock:
                    self.belt_center_y = msg.pose[idx].position.y
            
            # Sincronizar posiciones de los cilindros desde Gazebo cuando no nos estamos moviendo
            with self.conveyor_lock:
                if not self.conveyor_active:
                    for name in self.cylinder_positions:
                        # Buscamos la posición del sistema de referencia de la base: target_base_x
                        link_name = f"{name}::target_base_x"
                        if link_name in msg.name:
                            idx = msg.name.index(link_name)
                            self.cylinder_positions[name] = msg.pose[idx].position.y
        except Exception as e:
            pass

    def start_laser_thread(self):
        self.laser_thread = threading.Thread(target=self.laser_loop, daemon=True)
        self.laser_thread.start()

    def laser_loop(self):
        rate = 20.0  # 20 Hz
        dt = 1.0 / rate
        last_active = None
        last_pose_is_none = None
        
        while rclpy.ok():
            try:
                with self.laser_lock:
                    active = self.laser_active
                    pose = self.flange_pose
                
                # Registrar transiciones en el archivo de log
                if active != last_active or (pose is None) != last_pose_is_none:
                    log_message(f"[laser_loop] Cambio de estado: active={active}, pose_is_None={pose is None}, laser_was_active={self.laser_was_active}")
                    last_active = active
                    last_pose_is_none = (pose is None)
                
                if active and pose is not None:
                    # Posición exacta de la brida (el offset de la longitud del haz ya está en el URDF)
                    px = pose.position.x
                    py = pose.position.y
                    pz = pose.position.z
                    q = pose.orientation
                    
                    # Set entity state for laser_beam
                    if self.set_entity_state_client.service_is_ready():
                        s_req = SetEntityState.Request()
                        s_req.state.name = 'laser_beam'
                        s_req.state.reference_frame = 'world'
                        s_req.state.pose.position.x = px
                        s_req.state.pose.position.y = py
                        s_req.state.pose.position.z = pz
                        s_req.state.pose.orientation = q
                        self.set_entity_state_client.call_async(s_req)
                    else:
                        log_message("[laser_loop] ADVERTENCIA: set_entity_state_client NO está listo para MOSTRAR el láser")
                else:
                    if self.laser_was_active:
                        if self.set_entity_state_client.service_is_ready():
                            s_req = SetEntityState.Request()
                            s_req.state.name = 'laser_beam'
                            s_req.state.reference_frame = 'world'
                            s_req.state.pose.position.z = -10.0
                            s_req.state.pose.orientation.w = 1.0
                            self.set_entity_state_client.call_async(s_req)
                            self.laser_was_active = False
                            log_message("[laser_loop] Enviada solicitud para OCULTAR el láser en Gazebo.")
                        else:
                            log_message("[laser_loop] ADVERTENCIA: set_entity_state_client NO está listo para OCULTAR el láser")
                        
                if active and pose is not None:
                    self.laser_was_active = True
            except Exception as e:
                log_message(f"[laser_loop] EXCEPCIÓN: {e}")
                
            time.sleep(dt)

    def publish_joints(self, j1_deg, j2_deg, j3_deg):
        """
        Convierte los ángulos recibidos de grados a radianes y los publica en el tópico de ROS 2.
        """
        j1_rad = float(j1_deg) * math.pi / 180.0
        j2_rad = float(j2_deg) * math.pi / 180.0
        j3_rad = float(j3_deg) * math.pi / 180.0

        msg = Float64MultiArray()
        msg.data = [j1_rad, j2_rad, j3_rad]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado en ROS -> J1: {j1_rad:.4f} rad, J2: {j2_rad:.4f} rad, J3: {j3_rad:.4f} rad')

    def toggle_workspace_sphere(self, visible):
        """
        Mueve la esfera de espacio de trabajo en Gazebo Classic a Z=0.096173 (visible) o Z=-10.0 (oculto).
        """
        print(f"[WebBridge] Petición para mover esfera recibida: visible={visible}")
        sys.stdout.flush()

        if not self.set_entity_state_client.service_is_ready():
            print("[WebBridge] ADVERTENCIA: El servicio /gazebo/set_entity_state NO está disponible.")
            sys.stdout.flush()
            self.get_logger().warning('El servicio /gazebo/set_entity_state no está disponible en Gazebo.')

        req = SetEntityState.Request()
        req.state.name = 'workspace_sphere'
        req.state.reference_frame = 'world'

        z_pos = 0.096173 if visible else -10.0

        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = z_pos
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0

        # Llamada no bloqueante
        future = self.set_entity_state_client.call_async(req)
        
        def service_callback(fut):
            try:
                response = fut.result()
                print(f"[WebBridge] RESPUESTA DE GAZEBO: success={response.success}, message='{response.status_message}'")
                sys.stdout.flush()
            except Exception as e:
                print(f"[WebBridge] ERROR al obtener resultado del servicio: {e}")
                sys.stdout.flush()

        future.add_done_callback(service_callback)
        print(f"[WebBridge] Llamada enviada al servicio /gazebo/set_entity_state con Z={z_pos}")
        sys.stdout.flush()
        self.get_logger().info(f'Solicitud enviada para reubicar la esfera: visible={visible} (Z={z_pos} m)')

    def reload_target_entity(self, xml_content):
        """
        Elimina y vuelve a spawnear el cilindro en Gazebo para forzar la recarga de la textura.
        """
        if not self.delete_entity_client.service_is_ready():
            print("[WebBridge] ERROR: Servicio /delete_entity no disponible.")
            sys.stdout.flush()
            return False
            
        del_req = DeleteEntity.Request()
        del_req.name = 'cutting_target'
        
        future_del = self.delete_entity_client.call_async(del_req)
        start_time = time.time()
        while not future_del.done():
            time.sleep(0.02)
            if time.time() - start_time > 2.0:
                print("[WebBridge] Timeout al borrar entidad.")
                sys.stdout.flush()
                break
                
        time.sleep(0.1) # Breve pausa antes de spawnear
        
        if not self.spawn_entity_client.service_is_ready():
            print("[WebBridge] ERROR: Servicio /spawn_entity no disponible.")
            sys.stdout.flush()
            return False
            
        spawn_req = SpawnEntity.Request()
        spawn_req.name = 'cutting_target'
        spawn_req.xml = xml_content
        spawn_req.robot_namespace = ''
        spawn_req.initial_pose.position.x = 0.50
        spawn_req.initial_pose.position.y = 0.0
        spawn_req.initial_pose.position.z = 0.015
        spawn_req.initial_pose.orientation.x = 0.0
        spawn_req.initial_pose.orientation.y = 0.0
        spawn_req.initial_pose.orientation.z = 0.0
        spawn_req.initial_pose.orientation.w = 1.0
        spawn_req.reference_frame = 'world'
        
        future_spawn = self.spawn_entity_client.call_async(spawn_req)
        start_time = time.time()
        while not future_spawn.done():
            time.sleep(0.02)
            if time.time() - start_time > 3.0:
                print("[WebBridge] Timeout al spawnear entidad.")
                sys.stdout.flush()
                break
        return True


ros_node = None


class ROSHTTPServerHandler(http.server.SimpleHTTPRequestHandler):
    """
    Manejador del servidor HTTP que procesa peticiones REST del frontend y las traduce a ROS 2.
    """
    def do_POST(self):
        """
        Procesa peticiones POST enviadas a la API, decodifica y ejecuta la lógica correspondiente en el nodo ROS 2.
        """
        if self.path == '/api/move':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                j1 = data.get('j1', 0.0)
                j2 = data.get('j2', 0.0)
                j3 = data.get('j3', 0.0)
                
                if ros_node is not None:
                    ros_node.publish_joints(j1, j2, j3)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path == '/api/toggle_workspace':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                visible = data.get('visible', True)
                print(f"[WebBridge] POST /api/toggle_workspace: visible={visible}")
                sys.stdout.flush()
                
                if ros_node is not None:
                    ros_node.toggle_workspace_sphere(visible)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
            except Exception as e:
                print(f"[WebBridge] ERROR en POST /api/toggle_workspace: {e}")
                sys.stdout.flush()
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path in ('/api/advance_conveyor', '/api/toggle_conveyor'):
            try:
                content_length = 0
                if 'Content-Length' in self.headers:
                    try:
                        content_length = int(self.headers['Content-Length'])
                    except ValueError:
                        pass
                
                post_data = b''
                if content_length > 0:
                    post_data = self.rfile.read(content_length)
                
                log_message(f"[HTTP] POST {self.path} recibido. Conmutando estado de la cinta...")
                print(f"[WebBridge] POST {self.path}")
                sys.stdout.flush()
                
                if ros_node is not None:
                    active_req = None
                    if post_data:
                        try:
                            data = json.loads(post_data.decode('utf-8'))
                            if 'active' in data:
                                active_req = bool(data['active'])
                        except Exception:
                            pass
                    
                    with ros_node.conveyor_lock:
                        if active_req is not None:
                            ros_node.conveyor_active = active_req
                        else:
                            ros_node.conveyor_active = not ros_node.conveyor_active
                        
                        log_message(f"[HTTP] Estado de la cinta establecido en: {ros_node.conveyor_active}")
                        current_active = ros_node.conveyor_active
                else:
                    current_active = False
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success", "active": current_active}).encode('utf-8'))
            except Exception as e:
                log_message(f"[HTTP] ERROR en POST {self.path}: {e}")
                print(f"[WebBridge] ERROR en POST {self.path}: {e}")
                sys.stdout.flush()
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path == '/api/toggle_laser':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                active = data.get('active', False)
                print(f"[WebBridge] POST /api/toggle_laser: active={active}")
                sys.stdout.flush()
                log_message(f"[HTTP] POST /api/toggle_laser: active={active}")
                
                if ros_node is not None:
                    with ros_node.laser_lock:
                        ros_node.laser_active = active
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success", "active": active}).encode('utf-8'))
            except Exception as e:
                print(f"[WebBridge] ERROR en POST /api/toggle_laser: {e}")
                sys.stdout.flush()
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path == '/api/process_drawing':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                image_b64 = data.get('image', '')
                
                num_pixels = 0
                bbox_info = "Ninguno"
                
                import time
                ts = str(int(time.time()))
                filename = f"target_draw_{ts}.png"
                material_name = f"CuttingTarget/Drawing_{ts}"
                
                # Guardar imagen como textura target_draw.png en src e install
                import base64
                if image_b64.startswith("data:image/png;base64,"):
                    header, base64_data = image_b64.split(',', 1)
                    img_bytes = base64.b64decode(base64_data)
                    
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    
                    src_tex_dir = os.path.abspath(os.path.join(script_dir, "../ros2_ws/src/arm_simulation/materials/textures"))
                    install_tex_dir = os.path.abspath(os.path.join(script_dir, "../ros2_ws/install/arm_simulation/share/arm_simulation/materials/textures"))
                    
                    # 1. Limpiar texturas antiguas
                    for d in [src_tex_dir, install_tex_dir]:
                        if os.path.exists(d):
                            for f in os.listdir(d):
                                if f.startswith("target_draw_") and f.endswith(".png"):
                                    try:
                                        os.remove(os.path.join(d, f))
                                    except Exception:
                                        pass
                    
                    # 2. Guardar la nueva textura
                    src_path = os.path.join(src_tex_dir, filename)
                    os.makedirs(src_tex_dir, exist_ok=True)
                    with open(src_path, "wb") as f:
                        f.write(img_bytes)
                        
                    install_path = os.path.join(install_tex_dir, filename)
                    os.makedirs(install_tex_dir, exist_ok=True)
                    with open(install_path, "wb") as f:
                        f.write(img_bytes)
                        
                    # 3. Escribir el nuevo archivo .material dinámico
                    material_content = f"""material {material_name}
{{
  technique
  {{
    pass
    {{
      ambient 1.0 1.0 1.0 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.2 0.2 0.2 1.0 12.5
      emissive 0.1 0.1 0.1 1.0

      texture_unit
      {{
        texture {filename}
        filtering bilinear
      }}
    }}
  }}
}}
"""
                    src_mat_dir = os.path.abspath(os.path.join(script_dir, "../ros2_ws/src/arm_simulation/materials/scripts"))
                    install_mat_dir = os.path.abspath(os.path.join(script_dir, "../ros2_ws/install/arm_simulation/share/arm_simulation/materials/scripts"))
                    
                    for d in [src_mat_dir, install_mat_dir]:
                        os.makedirs(d, exist_ok=True)
                        with open(os.path.join(d, "target.material"), "w") as f:
                            f.write(material_content)
                    
                    global HAS_OPENCV, HAS_PILLOW
                    if HAS_OPENCV:
                        import cv2
                        import numpy as np
                        nparr = np.frombuffer(img_bytes, np.uint8)
                        img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                        if img is not None:
                            # Contar píxeles que no son del fondo cian (r=0, g=204, b=204)
                            if img.shape[2] >= 3:
                                diff = np.sum(np.abs(img[:, :, :3].astype(int) - [204, 204, 0]), axis=2)
                                draw_mask = diff > 30
                                num_pixels = int(np.sum(draw_mask))
                                pts = np.argwhere(draw_mask)
                                if len(pts) > 0:
                                    min_y, min_x = pts.min(axis=0)
                                    max_y, max_x = pts.max(axis=0)
                                    bbox_info = f"[{min_x}, {min_y}] a [{max_x}, {max_y}]"
                    elif HAS_PILLOW:
                        from PIL import Image as PILImage
                        import io
                        img = PILImage.open(io.BytesIO(img_bytes))
                        rgb_img = img.convert('RGB')
                        pixels = list(rgb_img.getdata())
                        diff_pixels = [p for p in pixels if abs(p[0]-0) + abs(p[1]-204) + abs(p[2]-204) > 30]
                        num_pixels = len(diff_pixels)
                        width, height = rgb_img.size
                        drawn_indices = [i for i, p in enumerate(pixels) if abs(p[0]-0) + abs(p[1]-204) + abs(p[2]-204) > 30]
                        if drawn_indices:
                            min_x = min(idx % width for idx in drawn_indices)
                            max_x = max(idx % width for idx in drawn_indices)
                            min_y = min(idx // width for idx in drawn_indices)
                            max_y = max(idx // width for idx in drawn_indices)
                            bbox_info = f"[{min_x}, {min_y}] a [{max_x}, {max_y}]"
                
                # Forzar recarga del modelo en Gazebo
                reload_status = "No recargado"
                if ros_node is not None:
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    urdf_path = os.path.abspath(os.path.join(script_dir, "../ros2_ws/src/arm_simulation/urdf/cutting_target.urdf"))
                    if os.path.exists(urdf_path):
                        with open(urdf_path, "r") as f:
                            urdf_xml = f.read()
                        
                        # Reemplazar dinámicamente el nombre del material en el URDF
                        import re
                        urdf_xml_modified = re.sub(
                            r'<name>CuttingTarget/Drawing(_\d+)?</name>',
                            f'<name>{material_name}</name>',
                            urdf_xml
                        )
                        
                        success = ros_node.reload_target_entity(urdf_xml_modified)
                        reload_status = "Exitoso" if success else "Fallo"
                
                log_msg = f"Imagen de textura recibida. Pixeles dibujados: {num_pixels}. BBox: {bbox_info}. Recarga Gazebo: {reload_status}"
                print(f"[WebBridge] {log_msg}")
                sys.stdout.flush()
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "success", 
                    "pixels": num_pixels,
                    "bbox": bbox_info,
                    "message": f"Cámara detectó el trazo en Gazebo. Recarga: {reload_status}."
                }).encode('utf-8'))
            except Exception as e:
                print(f"[WebBridge] ERROR en POST /api/process_drawing: {e}")
                sys.stdout.flush()
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path == '/api/calculate_path':
            try:
                if ros_node is None:
                    raise Exception("ROS Node not initialized")
                
                # Obtener la imagen más reciente de la cámara
                with ros_node.jpeg_lock:
                    jpeg_data = ros_node.latest_jpeg
                
                if jpeg_data is None:
                    raise Exception("No image available from camera yet. Make sure the camera tab is active or the camera is streaming.")
                
                import cv2
                import numpy as np
                import base64
                
                # Decodificar la imagen JPEG
                nparr = np.frombuffer(jpeg_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is None:
                    raise Exception("Failed to decode camera image")
                
                # 1. Detectar el cilindro cian en la imagen
                # Cian en BGR: [204, 204, 0] aprox. (HSV: H entre 80 y 105, S/V entre 50 y 255)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower_cyan = np.array([80, 50, 50])
                upper_cyan = np.array([105, 255, 255])
                cyan_mask = cv2.inRange(hsv, lower_cyan, upper_cyan)
                
                # Encontrar contornos de la máscara cian
                contours, _ = cv2.findContours(cyan_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    raise Exception("No se detectó el cilindro cian en la imagen de la cámara. Verifica que un cilindro esté alineado en la faja.")
                
                # Tomar el contorno de mayor área
                c = max(contours, key=cv2.contourArea)
                x_box, y_box, w_box, h_box = cv2.boundingRect(c)
                
                # Validar dimensiones mínimas para evitar falsos positivos
                if w_box < 10 or h_box < 10:
                    raise Exception("El objeto cian detectado es demasiado pequeño")
                
                # Extraer la ROI del cilindro
                cylinder_roi = img[y_box:y_box+h_box, x_box:x_box+w_box]
                
                # 2. Desenrollado Cilíndrico
                # Curvatura vertical (eje Y de la ROI). El radio en px es h_box / 2
                R_p = h_box / 2.0
                y_c = h_box / 2.0
                
                # Altura desenrollada = R_p * pi
                h_unrolled = int(R_p * np.pi)
                w_unrolled = w_box
                
                # Mapeo de píxeles vectorizado usando np.meshgrid
                theta_max = 1.25  # Limitar a unos +/- 71.6 grados para evitar los bordes extremos del cilindro
                y_indices = np.arange(h_unrolled)
                x_indices = np.arange(w_box)
                
                map_x, map_y_indices = np.meshgrid(x_indices, y_indices)
                
                theta = -theta_max + (map_y_indices / (h_unrolled - 1.0)) * (2.0 * theta_max)
                dy = R_p * np.sin(theta)
                map_y = (y_c + dy).astype(np.float32)
                map_x = map_x.astype(np.float32)
                
                unrolled = cv2.remap(cylinder_roi, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(204, 204, 0))
                
                # 3. Aislar el dibujo/trazo de forma robusta usando Umbralización Adaptativa
                # La umbralización adaptativa calcula un umbral local por píxel, lo que la hace
                # totalmente inmune a gradientes de luz, reflejos especulares y sombras globales.
                gray = cv2.cvtColor(unrolled, cv2.COLOR_BGR2GRAY)
                stroke_mask = cv2.adaptiveThreshold(
                    gray, 
                    255, 
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                    cv2.THRESH_BINARY_INV, 
                    21,  # Tamaño de bloque (debe ser impar y mayor que el ancho de línea)
                    6    # Constante C restada de la media local (más sensible para trazos tenues)
                )
                
                # Limpiar márgenes de la ROI desenrollada para ignorar bordes de corte artificiales y fondo
                margin_x = int(w_unrolled * 0.08)
                margin_y = int(h_unrolled * 0.08)
                stroke_mask[:margin_y, :] = 0
                stroke_mask[-margin_y:, :] = 0
                stroke_mask[:, :margin_x] = 0
                stroke_mask[:, -margin_x:] = 0
                
                # 4. Limpieza Morfológica y eliminación de ruido en la máscara del trazo
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                cleaned_mask = cv2.morphologyEx(stroke_mask, cv2.MORPH_CLOSE, kernel)
                cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_OPEN, kernel)
                
                # Filtrar componentes conectados muy pequeños (como ruido de umbralizado)
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(cleaned_mask, connectivity=8)
                for i in range(1, num_labels):
                    if stats[i, cv2.CC_STAT_AREA] < 8:
                        cleaned_mask[labels == i] = 0
                
                # 5. Esqueletización del trazo utilizando Zhang-Suen (línea central real de 1px)
                skel = zhang_suen_thinning(cleaned_mask)
                
                # Extraer todas las coordenadas de píxeles del esqueleto (en formato Y, X)
                pts_y, pts_x = np.where(skel > 0)
                pts_2d_raw = [(int(x), int(y)) for x, y in zip(pts_x, pts_y)]
                
                # Ordenamiento de Trayectoria por Vecino Más Cercano y separación de trazos (strokes)
                strokes = []
                if pts_2d_raw:
                    unvisited = set(pts_2d_raw)
                    # Comenzar por el punto que esté más arriba a la izquierda
                    current = min(unvisited, key=lambda p: p[0] + p[1])
                    current_stroke = [current]
                    unvisited.remove(current)
                    
                    while unvisited:
                        cx, cy = current
                        # Encontrar el punto no visitado más cercano
                        nearest = min(unvisited, key=lambda p: (p[0] - cx)**2 + (p[1] - cy)**2)
                        dist_sq = (nearest[0] - cx)**2 + (nearest[1] - cy)**2
                        
                        # Si la distancia al vecino más cercano es pequeña (<= 3 px), continúa el mismo trazo.
                        # De lo contrario, se trata de un salto de trayectoria (levantar láser / fin de rama).
                        if dist_sq <= 9.0:
                            current_stroke.append(nearest)
                        else:
                            strokes.append(current_stroke)
                            current_stroke = [nearest]
                            
                        unvisited.remove(nearest)
                        current = nearest
                    strokes.append(current_stroke)
                
                # Simplificar cada trazo individual y descartar trazos diminutos (ruido residual o espuelas)
                simplified_strokes = []
                for stroke in strokes:
                    if len(stroke) < 3: # Filtrar trazos extremadamente cortos (< 3px)
                        continue
                    stroke_arr = np.array(stroke, dtype=np.int32).reshape(-1, 1, 2)
                    epsilon = 1.2  # Tolerancia para suavizar las líneas y eliminar el efecto pixelado/jagged
                    approx = cv2.approxPolyDP(stroke_arr, epsilon, False)
                    simplified_stroke = [tuple(pt[0]) for pt in approx]
                    if len(simplified_stroke) >= 2:
                        simplified_strokes.append(simplified_stroke)
                
                # Compilar los puntos 2D finales asignándoles la bandera de láser activo
                points_2d_with_laser = []
                for stroke in simplified_strokes:
                    for idx, pt in enumerate(stroke):
                        # El primer punto de cada trazo es un posicionamiento (láser apagado)
                        # Los siguientes puntos son cortes activos (láser encendido)
                        laser_on = (idx > 0)
                        points_2d_with_laser.append((pt, laser_on))
                
                if not points_2d_with_laser:
                    raise Exception("No se identificó ningún trazo o dibujo oscuro sobre la superficie del cilindro")
                
                # Redimensionar la imagen de visualización a una escala de 4x para que se vea nítida en la UI
                scale_factor = 4
                h_vis = h_unrolled * scale_factor
                w_vis = w_unrolled * scale_factor
                result_vis = cv2.resize(unrolled, (w_vis, h_vis), interpolation=cv2.INTER_CUBIC)
                
                # Dibujar el recorrido en rojo solo para los segmentos donde el láser está encendido
                for i in range(1, len(points_2d_with_laser)):
                    pt_prev, _ = points_2d_with_laser[i-1]
                    pt_curr, laser_on = points_2d_with_laser[i]
                    if laser_on:
                        p1 = (pt_prev[0] * scale_factor, pt_prev[1] * scale_factor)
                        p2 = (pt_curr[0] * scale_factor, pt_curr[1] * scale_factor)
                        cv2.line(result_vis, p1, p2, (0, 0, 255), 2)
                
                # Codificar la imagen visualizada a Base64
                _, res_jpeg = cv2.imencode('.jpg', result_vis)
                res_b64 = base64.b64encode(res_jpeg.tobytes()).decode('utf-8')
                
                # 5. Convertir a coordenadas 3D en mm
                # Localizar la posición Y actual del cilindro más cercano al centro de corte
                cylinder_y = 0.0
                with ros_node.conveyor_lock:
                    target_y = ros_node.belt_center_y
                    min_dist = 999.0
                    for name, y in ros_node.cylinder_positions.items():
                        dist = abs(y - target_y)
                        if dist < min_dist:
                            min_dist = dist
                            cylinder_y = y
                
                # Parámetros físicos: largo = 60mm, radio = 15mm. Eje central del cilindro en X=500mm
                R_m = 15.0
                points_3d = []
                
                for (px, py), laser_on in points_2d_with_laser:
                    # Mapear el eje vertical (py) a un ángulo theta (de -theta_max a theta_max)
                    theta_max = 1.25
                    theta_val = -theta_max + (py / (h_unrolled - 1.0)) * (2.0 * theta_max)
                    
                    dx = R_m * np.sin(theta_val)
                    dz = R_m * np.cos(theta_val)
                    
                    # Proyectar
                    x_r = 500.0 + dx
                    y_r = (cylinder_y + ((px / w_box) - 0.5) * 0.06) * 1000.0
                    z_r = 55.0 + dz # El centro geométrico está en Z=55mm (pose.position.z = 0.055)
                    
                    points_3d.append({"x": x_r, "y": y_r, "z": z_r, "laser": laser_on})
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "success",
                    "image": f"data:image/jpeg;base64,{res_b64}",
                    "points": points_3d
                }).encode('utf-8'))
                
            except Exception as e:
                log_message(f"[HTTP] ERROR en POST /api/calculate_path: {e}")
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path == '/api/physical/move':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                x = float(data.get('x', 0.0))
                y = float(data.get('y', 0.0))
                z = float(data.get('z', 0.0))
                ok, resp = serial_manager.send_cartesian(x, y, z)
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({
                    "status": "ok" if ok else "error",
                    "response": resp,
                    "connected": serial_manager.status["connected"]
                }).encode('utf-8'))
            except Exception as e:
                log_message(f"[HTTP] ERROR en POST /api/physical/move: {e}")
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        elif self.path == '/api/physical/connect':
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length) if content_length > 0 else b'{}'
            try:
                data = json.loads(post_data.decode('utf-8')) if post_data else {}
                if data.get('disconnect', False):
                    serial_manager.disconnect()
                else:
                    port = data.get('port', None)
                    serial_manager.connect(port=port)
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(serial_manager.status).encode('utf-8'))
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        """
        Maneja las peticiones GET, incluyendo el flujo continuo de video de la cámara
        virtual de Gazebo (/api/camera_stream) y la entrega de archivos estáticos del frontend.
        """
        if self.path == '/api/physical/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(serial_manager.status).encode('utf-8'))
            return
        elif self.path == '/api/conveyor_state':
            if ros_node is None:
                self.send_response(503)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "ROS Node not initialized"}).encode('utf-8'))
                return
                
            with ros_node.conveyor_lock:
                state_data = {
                    "active": ros_node.conveyor_active,
                    "positions": ros_node.cylinder_positions
                }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(state_data).encode('utf-8'))
            return
            
        elif self.path == '/api/camera_stream':
            if ros_node is None:
                self.send_response(503)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b"Servidor ROS 2 no inicializado")
                return

            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            try:
                while True:
                    with ros_node.jpeg_lock:
                        jpeg_data = ros_node.latest_jpeg

                    if jpeg_data is not None:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpeg_data)}\r\n\r\n'.encode('utf-8'))
                        self.wfile.write(jpeg_data)
                        self.wfile.write(b'\r\n')
                    else:
                        time.sleep(0.1)
                        continue

                    # Controlar tasa de fotogramas (~16 FPS)
                    time.sleep(0.06)
            except Exception as e:
                pass
        else:
            super().do_GET()


def run_http_server():
    """
    Inicia y mantiene en ejecución el servidor HTTP en el puerto 8080.
    """
    server_address = ('', 8080)
    try:
        from http.server import ThreadingHTTPServer
        httpd = ThreadingHTTPServer(server_address, ROSHTTPServerHandler)
        print("Servidor HTTP multihilo (ThreadingHTTPServer) corriendo en http://localhost:8080 ...")
    except ImportError:
        import socketserver
        class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
            daemon_threads = True
        httpd = ThreadingHTTPServer(server_address, ROSHTTPServerHandler)
        print("Servidor HTTP multihilo (socketserver fallback) corriendo en http://localhost:8080 ...")
        
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


def main():
    """
    Punto de entrada principal que inicializa el entorno de ROS 2, inicia el servidor HTTP
    en un hilo secundario y mantiene activo el nodo en el hilo principal.
    """
    global ros_node
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    global HAS_OPENCV, HAS_PILLOW
    if not HAS_OPENCV and not HAS_PILLOW:
        print("[WebBridge] ADVERTENCIA: OpenCV y Pillow no están instalados en el entorno.")
        print("[WebBridge] La transmisión de la cámara de Gazebo no funcionará.")
        print("[WebBridge] Para habilitarla, instala OpenCV o Pillow en distrobox: pip install opencv-python pillow")
        sys.stdout.flush()
    else:
        lib_name = "OpenCV" if HAS_OPENCV else "Pillow"
        print(f"[WebBridge] Codificador de imagen inicializado usando {lib_name}.")
        sys.stdout.flush()

    rclpy.init(args=None)
    ros_node = WebBridgeNode()
    
    server_thread = threading.Thread(target=run_http_server, daemon=True)
    server_thread.start()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        print("\nDeteniendo puente WebBridge...")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
