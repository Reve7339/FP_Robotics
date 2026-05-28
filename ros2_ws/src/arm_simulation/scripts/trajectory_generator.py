#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)

        # Frecuencia de actualización para movimiento suave (50 Hz)
        self.rate = 50.0
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        # Definición de la trayectoria en pose de articulaciones [joint1, joint2, joint3] en radianes
        # Diseñado para simular una secuencia de corte industrial:
        # 1. Home -> 2. Posicionar sobre pieza -> 3. Corte barrido horizontal -> 4. Retorno suave
        self.keyframes = [
            {"pose": [0.0, 0.0, 0.0], "time": 3.0},       # Home (reposo vertical)
            {"pose": [-0.6, 0.4, -0.4], "time": 3.0},     # Inicio de trayectoria (esquina inferior izquierda)
            {"pose": [0.6, 0.4, -0.4], "time": 4.0},      # Trayectoria de corte 1 (barrido a la derecha)
            {"pose": [0.6, 0.6, -0.7], "time": 2.0},      # Avance longitudinal
            {"pose": [-0.6, 0.6, -0.7], "time": 4.0},     # Trayectoria de corte 2 (barrido a la izquierda)
            {"pose": [0.0, 0.0, 0.0], "time": 3.0}        # Retorno a Home
        ]

        self.current_keyframe_idx = 0
        self.step_counter = 0
        self.total_steps_for_transition = 0
        self.start_pose = [0.0, 0.0, 0.0]
        self.target_pose = self.keyframes[0]["pose"]

        self.get_logger().info("Nodo de trayectoria inicializado. Iniciando ciclo de corte...")
        self.prepare_next_transition()

    def prepare_next_transition(self):
        self.start_pose = list(self.target_pose)
        keyframe = self.keyframes[self.current_keyframe_idx]
        self.target_pose = keyframe["pose"]
        duration = keyframe["time"]

        self.total_steps_for_transition = int(duration * self.rate)
        self.step_counter = 0

        self.get_logger().info(
            f"Transicionando a Keyframe {self.current_keyframe_idx}: {self.target_pose} en {duration}s"
        )

        # Avanzar al siguiente keyframe para la próxima vez
        self.current_keyframe_idx = (self.current_keyframe_idx + 1) % len(self.keyframes)

    def timer_callback(self):
        if self.step_counter >= self.total_steps_for_transition:
            self.prepare_next_transition()

        # Interpolación cosenoidal (S-curve / smoothstep) para aceleración/desaceleración suave
        t = float(self.step_counter) / float(self.total_steps_for_transition)
        smooth_t = (1.0 - math.cos(t * math.pi)) / 2.0  # Mapea de 0.0-1.0 de forma suave

        current_command = []
        for start, target in zip(self.start_pose, self.target_pose):
            val = start + (target - start) * smooth_t
            current_command.append(val)

        msg = Float64MultiArray()
        msg.data = current_command
        self.publisher_.publish(msg)

        self.step_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Generador de trayectoria detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
