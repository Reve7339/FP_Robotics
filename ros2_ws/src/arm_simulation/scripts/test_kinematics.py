import numpy as np

# Constantes del robot
d1 = 60.0
d2 = 36.173
a2 = 14.915
a3 = 146.190
a4 = 160.823
d4 = -4.0

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

def fk_original(theta1, theta2, theta3):
    theta2_star = theta2 + phi2
    theta3_star = theta3 + phi3 - phi2
    
    T1 = dh_matrix(theta1, d1, 0, 0)
    T2 = dh_matrix(0, d2, a2, 90)
    T3 = dh_matrix(theta2_star, 0, a3, 0)
    T4 = dh_matrix(theta3_star, d4, a4, 0)
    
    T = T1 @ T2 @ T3 @ T4
    return T

def fk_shifted(theta1, theta2, theta3):
    theta2_star = theta2 + phi2
    theta3_star = theta3 + phi3 - phi2
    
    T1 = dh_matrix(theta1, d1, 0, 0)
    T2 = dh_matrix(theta2_star, d2, a2, 90)
    T3 = dh_matrix(theta3_star, 0, a3, 0)
    T4 = dh_matrix(0, d4, a4, 0)
    
    T = T1 @ T2 @ T3 @ T4
    return T

# Probar con angulos cero
t1, t2, t3 = 0.0, 0.0, 0.0
T_orig = fk_original(t1, t2, t3)
T_shifted = fk_shifted(t1, t2, t3)

print("Original End Effector Position:")
print(T_orig[:3, 3])
print("\nShifted End Effector Position:")
print(T_shifted[:3, 3])

# Probar con otros angulos
t1, t2, t3 = 0.5, -0.3, 0.8
T_orig = fk_original(t1, t2, t3)
T_shifted = fk_shifted(t1, t2, t3)

print("\n--- Con t1=0.5, t2=-0.3, t3=0.8 ---")
print("Original End Effector Position:")
print(T_orig[:3, 3])
print("\nShifted End Effector Position:")
print(T_shifted[:3, 3])
