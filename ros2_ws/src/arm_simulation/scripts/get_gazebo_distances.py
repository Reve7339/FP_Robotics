import xml.etree.ElementTree as ET
import math
import os

def parse_urdf():
    # Path to the URDF/Xacro file
    urdf_path = "/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/urdf/arm.urdf.xacro"
    if not os.path.exists(urdf_path):
        print(f"Error: No se encontró el archivo URDF en {urdf_path}")
        return

    # Parse XML
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    print("=== DISTANCIAS Y LOGITUDES EXTRAÍDAS DEL MODELO URDF EN GAZEBO ===\n")

    # 1. World to base_link
    joint_world = root.find(".//joint[@name='world_to_base']")
    if joint_world is not None:
        origin = joint_world.find("origin")
        print(f"world_to_base:")
        print(f"  xyz: {origin.attrib.get('xyz')}")
        print(f"  rpy: {origin.attrib.get('rpy')}\n")

    # 2. Joint 1 (Base Rotation)
    joint1 = root.find(".//joint[@name='joint1']")
    if joint1 is not None:
        origin = joint1.find("origin")
        print(f"joint1 (Base a Link1):")
        print(f"  xyz: {origin.attrib.get('xyz')} (Desfase vertical base height d1 = 60 mm)")
        print(f"  rpy: {origin.attrib.get('rpy')}\n")

    # 3. Joint 2 (Shoulder)
    joint2 = root.find(".//joint[@name='joint2']")
    if joint2 is not None:
        origin = joint2.find("origin")
        xyz = [float(x) for x in origin.attrib.get('xyz').split()]
        print(f"joint2 (Link1 a Hombro):")
        print(f"  xyz: {origin.attrib.get('xyz')} (Desfase radial a2 = {xyz[0]*1000:.3f} mm, Desfase vertical d2 = {xyz[2]*1000:.3f} mm)")
        print(f"  rpy: {origin.attrib.get('rpy')}\n")

    # 4. Joint 3 (Elbow)
    joint3 = root.find(".//joint[@name='joint3']")
    if joint3 is not None:
        origin = joint3.find("origin")
        xyz = [float(x) for x in origin.attrib.get('xyz').split()]
        length = math.sqrt(xyz[0]**2 + xyz[1]**2) * 1000
        angle = math.atan2(xyz[1], xyz[0]) * 180 / math.pi
        print(f"joint3 (Hombro a Codo):")
        print(f"  xyz: {origin.attrib.get('xyz')}")
        print(f"  Longitud del brazo (Link2) a3: {length:.3f} mm")
        print(f"  Ángulo de inclinación phi2: {angle:.6f}° ({math.atan2(xyz[1], xyz[0]):.6f} rad)\n")

    # 5. Joint 3 to Flange
    joint_flange = root.find(".//joint[@name='joint_3-flange']")
    if joint_flange is not None:
        origin = joint_flange.find("origin")
        xyz = [float(x) for x in origin.attrib.get('xyz').split()]
        length = math.sqrt(xyz[0]**2 + xyz[1]**2) * 1000
        angle = math.atan2(xyz[1], xyz[0]) * 180 / math.pi
        print(f"joint_3-flange (Codo a Brida):")
        print(f"  xyz: {origin.attrib.get('xyz')}")
        print(f"  Longitud del antebrazo (Link3) a4: {length:.3f} mm")
        print(f"  Ángulo de inclinación phi3: {angle:.6f}° ({math.atan2(xyz[1], xyz[0]):.6f} rad)")
        print(f"  Desfase transversal d4: {xyz[2]*1000:.3f} mm\n")

if __name__ == "__main__":
    parse_urdf()
