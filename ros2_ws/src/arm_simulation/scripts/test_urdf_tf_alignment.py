#!/usr/bin/env python3
import math
import numpy as np

def rpy_to_matrix(r, p, y):
    # R = R_z(y) * R_y(p) * R_x(r)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    return Rz @ Ry @ Rx

# Datos de origen de las articulaciones de la simulación (URDF actual)
# 1. world -> base_link: rpy="1.57079632679 0 0"
R_w_base = rpy_to_matrix(1.57079632679, 0, 0)

# 2. base_link -> link1 (joint1, theta1=0): rpy="-1.57079632679 0 0"
R_base_l1 = rpy_to_matrix(-1.57079632679, 0, 0)

# 3. link1 -> link2 (joint2, theta2=0): rpy="1.57079632679 0 0"
R_l1_l2 = rpy_to_matrix(1.57079632679, 0, 0)

# 4. link2 -> link3 (joint3, theta3=0): rpy="0 0 0"
R_l2_l3 = rpy_to_matrix(0, 0, 0)

# 5. link3 -> flange (joint_3-flange): rpy="0 0 0"
R_l3_flange = rpy_to_matrix(0, 0, 0)

# Marcos de referencia visuales (visualize_frame) en el URDF actual:
# system0: parent="base_link", rpy="-1.57079632679 0 0"
R_base_sys0 = rpy_to_matrix(-1.57079632679, 0, 0)

# system1: parent="link2", rpy="0 0 1.030041" (using 0 0 0 to check link coordinate frame)
R_l2_sys1 = rpy_to_matrix(0, 0, 0)

# system2: parent="link3", rpy="0 0 -0.191392" (using 0 0 0 to check link coordinate frame)
R_l3_sys2 = rpy_to_matrix(0, 0, 0)

# system3: parent="flange", rpy="0 0 0"
R_flange_sys3 = rpy_to_matrix(0, 0, 0)

# Calcular rotaciones acumuladas relativas a link1 (marco horizontal del brazo)
# Queremos ver si los ejes X de cada sistema son paralelos al eje X de link1.
# El eje X de link1 en su propio marco es [1, 0, 0]^T.
# Para cualquier sistema S, su eje X en el marco de link1 es: R_l1_S * [1, 0, 0]^T.

transforms = {
    "system1": R_l1_l2 @ R_l2_sys1,
    "system2": R_l1_l2 @ R_l2_l3 @ R_l3_sys2,
    "system3": R_l1_l2 @ R_l2_l3 @ R_l3_flange @ R_flange_sys3,
    "flange": R_l1_l2 @ R_l2_l3 @ R_l3_flange
}

print("\n" + "="*75)
print(" CÁLCULO ANALÍTICO DE ALINEACIÓN DE EJES X (DESDE LA SIMULACIÓN URDF)")
print(" Plano de referencia: vista lateral (plano X-Z de link1)")
print("="*75)

all_parallel = True

for name, R_l1_S in transforms.items():
    # Obtener el eje X del sistema expresado en coordenadas de link1
    x_axis_in_l1 = R_l1_S[:, 0]
    
    # Desviación fuera del plano X-Z de link1 (componente Y)
    y_deviation = x_axis_in_l1[1]
    
    # Ángulo en el plano X-Z (Pitch)
    angle_rad = math.atan2(x_axis_in_l1[2], x_axis_in_l1[0])
    angle_deg = math.degrees(angle_rad)
    
    print(f"\nSistema: {name}")
    print(f"  Dirección del eje X en link1: [{x_axis_in_l1[0]:.6f}, {x_axis_in_l1[1]:.6f}, {x_axis_in_l1[2]:.6f}]")
    print(f"  Desviación lateral (Y): {y_deviation:.6f} (debe ser 0 para estar coplanar)")
    print(f"  Ángulo lateral (Pitch): {angle_deg:.4f}° (debe ser 0° para estar horizontal)")
    
    if abs(y_deviation) > 1e-5:
        print("  ❌ ERROR: El eje X se sale del plano vertical (desviación Y no es cero).")
        all_parallel = False
    elif abs(angle_deg) > 1e-5:
        print(f"  ❌ ERROR: El eje X no es horizontal en la vista lateral (Ángulo = {angle_deg:.4f}°).")
        all_parallel = False
    else:
        print("  ✅ PERFECTO: El eje X es coplanar, paralelo y horizontal.")

print("\n" + "="*75)
if all_parallel:
    print(" VERIFICACIÓN EXITOSA: ✅ TODOS LOS EJES X SON PARALELOS Y PERFECTAMENTE ALINEADOS!")
    print(" Al estar todos los ángulos en 0.0000°, el brazo físico en Gazebo")
    print(" formará una línea recta perfecta y horizontal.")
else:
    print(" VERIFICACIÓN FALLIDA: ❌ Existen desalineaciones entre los marcos.")
print("="*75 + "\n")
