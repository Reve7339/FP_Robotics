#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import sys

class TFEchoAll(Node):
    def __init__(self):
        super().__init__('tf_echo_all')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a timer to run the TF query once
        self.timer = self.create_timer(1.0, self.query_all_tfs)
        self.run_once = False

    def query_all_tfs(self):
        if self.run_once:
            return
        
        frames = ['link1', 'link2', 'link3', 'flange']
        base = 'base_link'
        
        print("\n=== COORDENADAS DE LAS ARTICULACIONES DESDE TF EN GAZEBO ===")
        print(f"Sistema de referencia base: {base}\n")
        
        success = True
        for frame in frames:
            try:
                # Query transform
                trans = self.tf_buffer.lookup_transform(base, frame, rclpy.time.Time())
                
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                
                # Convert quaternion to RPY
                # Simplified conversion for visual representation
                x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
                
                print(f"-> {frame} respecto a {base}:")
                print(f"   Traslación: [{translation.x:.6f}, {translation.y:.6f}, {translation.z:.6f}] m")
                print(f"               [{translation.x*1000:.3f}, {translation.y*1000:.3f}, {translation.z*1000:.3f}] mm")
                print(f"   Rotación (Quaternion): xyzw = [{x:.6f}, {y:.6f}, {z:.6f}, {w:.6f}]\n")
                
            except Exception as e:
                print(f"Error buscando transformada de {base} a {frame}: {str(e)}")
                success = False
                
        if success:
            self.run_once = True
            # Shutdown node cleanly after printing
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TFEchoAll()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
