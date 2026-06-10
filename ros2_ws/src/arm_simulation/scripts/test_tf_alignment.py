#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import math
import numpy as np

class TFAlignmentValidator(Node):
    def __init__(self):
        super().__init__('tf_alignment_validator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Esperar un poco a que se llene el buffer de TFs
        self.timer = self.create_timer(1.0, self.validate_alignment)
        self.get_logger().info("Validator node started. Waiting for TF data...")

    def validate_alignment(self):
        self.timer.cancel()
        
        frames = ['system1', 'system2', 'system3', 'flange']
        base_frame = 'link1'
        
        print("\n" + "="*70)
        print(" VALIDACIÓN DE ALINEACIÓN DE EJES X (VISTA LATERAL / PLANO X-Z de link1)")
        print("="*70)
        
        all_parallel = True
        
        for frame in frames:
            try:
                # Obtener la transformación de link1 al sistema
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(base_frame, frame, now, timeout=rclpy.duration.Duration(seconds=2.0))
                
                # Obtener cuaternión
                q = trans.transform.rotation
                
                # Convertir cuaternión a matriz de rotación
                # R = R(q)
                R = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)
                
                # El eje X del frame en coordenadas de link1 es la primera columna de R
                x_axis = R[:, 0]
                
                # Calcular ángulo en el plano vertical X-Z de link1
                # En link1, el eje Y es perpendicular al plano de movimiento,
                # por lo que el eje X del frame proyectado en el plano de movimiento de link1 (X-Z) es:
                angle_rad = math.atan2(x_axis[2], x_axis[0])
                angle_deg = math.degrees(angle_rad)
                
                print(f"\nFrame: {frame}")
                print(f"  Vector eje X (en frame link1): [{x_axis[0]:.6f}, {x_axis[1]:.6f}, {x_axis[2]:.6f}]")
                print(f"  Desviación lateral (Y): {x_axis[1]:.6f} (debe ser aprox 0)")
                print(f"  Ángulo en plano X-Z (Pitch): {angle_deg:.4f}°")
                
                if abs(x_axis[1]) > 0.005:
                    print("  ⚠️ ADVERTENCIA: El eje X no está en el mismo plano de movimiento (Y != 0)")
                    all_parallel = False
                elif abs(angle_deg) > 0.1:
                    print(f"  ⚠️ ADVERTENCIA: El eje X no es paralelo al eje X horizontal (Ángulo = {angle_deg:.4f}°)")
                    all_parallel = False
                else:
                    print("  ✅ ALINEACIÓN PERFECTA (eje X paralelo y horizontal)")
                    
            except Exception as e:
                self.get_logger().error(f"No se pudo obtener la transformación para {frame}: {str(e)}")
                all_parallel = False
        
        print("\n" + "="*70)
        if all_parallel:
            print(" RESULTADO FINAL: ✅ TODOS LOS EJES X SON PARALELOS Y HORIZONTALES!")
            print(" El brazo está perfectamente recto y alineado en la pose cero de la simulación.")
        else:
            print(" RESULTADO FINAL: ❌ EXISTEN ALINEACIONES O DESVIACIONES FUERA DE LÍMITES.")
        print("="*70 + "\n")
        
        rclpy.shutdown()

    def quaternion_to_matrix(self, x, y, z, w):
        # R = [ [1-2y^2-2z^2,   2xy-2wz,     2xz+2wy],
        #       [  2xy+2wz,   1-2x^2-2z^2,   2yz-2wx],
        #       [  2xz-2wy,     2yz+2wx,   1-2x^2-2y^2] ]
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = TFAlignmentValidator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()
