import numpy as np

# Constantes del robot
d1 = 60.0
d2 = 36.0
a2 = 15.0
a3 = 146.0
a4 = 161.0
d4 = -7.0

phi2 = 1.030041
phi3 = -0.191392

def dh_matrix(theta, d, a, alpha):
    alpha_rad = np.radians(alpha) if isinstance(alpha, (int, float)) else alpha
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha_rad),  np.sin(theta)*np.sin(alpha_rad), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha_rad), -np.cos(theta)*np.sin(alpha_rad), a*np.sin(theta)],
        [0,              np.sin(alpha_rad),               np.cos(alpha_rad),              d],
        [0,              0,                               0,                              1]
    ])

def fk_dh(theta1, theta2, theta3):
    theta2_star = theta2 + phi2
    theta3_star = theta3 + phi3 - phi2
    
    T1 = dh_matrix(theta1, d1 + d2, a2, 90)
    T2 = dh_matrix(theta2_star, 0, a3, 0)
    T3 = dh_matrix(theta3_star, d4, a4, 0)
    
    T = T1 @ T2 @ T3
    return T

def fk_analytical(theta1, theta2, theta3):
    theta2_star = theta2 + phi2
    theta3_star = theta3 + phi3 - phi2
    
    x_plane = a2 + a3 * np.cos(theta2_star) + a4 * np.cos(theta2_star + theta3_star)
    z_plane = d1 + d2 + a3 * np.sin(theta2_star) + a4 * np.sin(theta2_star + theta3_star)
    
    x = x_plane * np.cos(theta1) + d4 * np.sin(theta1)
    y = x_plane * np.sin(theta1) - d4 * np.cos(theta1)
    z = z_plane
    return np.array([x, y, z])

# Probar con angulos cero
t1, t2, t3 = 0.0, 0.0, 0.0
T_dh = fk_dh(t1, t2, t3)
pos_dh = T_dh[:3, 3]
pos_analyt = fk_analytical(t1, t2, t3)

print("D-H End Effector Position (3 matrices):")
print(pos_dh)
print("\nAnalytical End Effector Position:")
print(pos_analyt)

# Probar con otros angulos
t1, t2, t3 = 0.5, -0.3, 0.8
T_dh = fk_dh(t1, t2, t3)
pos_dh = T_dh[:3, 3]
pos_analyt = fk_analytical(t1, t2, t3)

print("\n--- Con t1=0.5, t2=-0.3, t3=0.8 ---")
print("D-H End Effector Position (3 matrices):")
print(pos_dh)
print("\nAnalytical End Effector Position:")
print(pos_analyt)
