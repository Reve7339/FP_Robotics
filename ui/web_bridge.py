import http.server
import json
import threading
import sys
import os
import math

# Importar cliente de ROS 2 en Python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        # Publicador en el tópico de comandos de control del brazo en simulación
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.get_logger().info('WebBridge ROS 2 Node iniciado. Publicando en /arm_controller/commands')

    def publish_joints(self, j1_deg, j2_deg, j3_deg):
        # Convertir ángulos de grados a radianes (ROS espera radianes)
        j1_rad = float(j1_deg) * math.pi / 180.0
        j2_rad = float(j2_deg) * math.pi / 180.0
        j3_rad = float(j3_deg) * math.pi / 180.0

        msg = Float64MultiArray()
        msg.data = [j1_rad, j2_rad, j3_rad]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado en ROS -> J1: {j1_rad:.4f} rad, J2: {j2_rad:.4f} rad, J3: {j3_rad:.4f} rad')

# Instancia global del nodo de ROS 2
ros_node = None

class ROSHTTPServerHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/api/move':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                j1 = data.get('j1', 0.0)
                j2 = data.get('j2', 0.0)
                j3 = data.get('j3', 0.0)
                
                # Publicar articulaciones a ROS
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
        else:
            self.send_response(404)
            self.end_headers()

def run_http_server():
    server_address = ('', 8080)
    httpd = http.server.HTTPServer(server_address, ROSHTTPServerHandler)
    print("Servidor HTTP corriendo en http://localhost:8080 ...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()

def main():
    global ros_node
    
    # Cambiar de directorio para servir correctamente la carpeta ui/ como estática
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    # Inicializar rclpy
    rclpy.init(args=None)
    ros_node = WebBridgeNode()
    
    # Arrancar el servidor HTTP en un hilo separado
    server_thread = threading.Thread(target=run_http_server, daemon=True)
    server_thread.start()
    
    # Mantener el nodo de ROS corriendo en el hilo principal
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        print("\nDeteniendo puente WebBridge...")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
