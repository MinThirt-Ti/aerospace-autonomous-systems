# Classical Lamination Theory Analysis
# DOCUMENTATION AND LABELLING BY GEMINI AI

import numpy as np
import pandas as pd

# 1. MATERIAL PROPERTIES (T300/5208) [cite: 3369]
E1 = 152000.0   # MPa
E2 = 8700.0     # MPa
G12 = 6400.0    # MPa
v12 = 0.23
v21 = v12 * E2 / E1  # Reciprocity

# Tsai-Wu Strength Data [cite: 3369, 3375]
Xt, Xc = 1500.0, -1250.0
Yt, Yc = 50.0, -200.0
S = 100.0

# 2. LAMINATE GEOMETRY [cite: 2279, 3369]
beta = 30.0  # Assigned off-axis alignment for ID Case01
t = 0.165    # mm per ply
num_plies = 24
H = num_plies * t # Total thickness

# ISO-3R Stacking Sequence (Top to Bottom) [cite: 2282]
iso_3r = np.array([
    90, 45, -45, 0, -45, 0, 45, 0, 90, -45, 90, 45, 
    90, 45, 0, 45, -45, 90, -45, 0, -45, 0, 90, 45
])

# 3. CLT CALCULATIONS: ABD MATRIX [cite: 2344, 2640]
denom = 1 - v12 * v21
Q = np.array([
    [E1/denom,     v12*E2/denom, 0],
    [v12*E2/denom, E2/denom,     0],
    [0,            0,            G12]
])

def get_transformed_q(angle_deg):
    theta = np.radians(angle_deg + beta)
    m, n = np.cos(theta), np.sin(theta)
    
    # Transformation formulas mapped to York (2010) 
    q11_p = Q[0,0]*m**4 + 2*(Q[0,1] + 2*Q[2,2])*m**2*n**2 + Q[1,1]*n**4
    q12_p = (Q[0,0] + Q[1,1] - 4*Q[2,2])*m**2*n**2 + Q[0,1]*(m**4 + n**4)
    q22_p = Q[0,0]*n**4 + 2*(Q[0,1] + 2*Q[2,2])*m**2*n**2 + Q[1,1]*m**4
    q66_p = (Q[0,0] + Q[1,1] - 2*Q[0,1] - 2*Q[2,2])*m**2*n**2 + Q[2,2]*(m**4 + n**4)
    q16_p = (Q[0,0] - Q[0,1] - 2*Q[2,2])*m**3*n + (Q[0,1] - Q[1,1] + 2*Q[2,2])*m*n**3
    q26_p = (Q[0,0] - Q[0,1] - 2*Q[2,2])*m*n**3 + (Q[0,1] - Q[1,1] + 2*Q[2,2])*m**3*n
    
    return np.array([[q11_p, q12_p, q16_p], [q12_p, q22_p, q26_p], [q16_p, q26_p, q66_p]])

A, B, D = np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))
z_coords = np.linspace(-H/2, H/2, num_plies + 1)

for i, angle in enumerate(iso_3r):
    zk, zk_1 = z_coords[i+1], z_coords[i]
    Qp = get_transformed_q(angle)
    A += Qp * (zk - zk_1)                   # [cite: 2763]
    B += 0.5 * Qp * (zk**2 - zk_1**2)       # [cite: 2344]
    D += (1/3.0) * Qp * (zk**3 - zk_1**3)   # [cite: 2640, 2764]

# 4. STRESS & TSAI-WU FAILURE ANALYSIS 
# Applied Load: M_x/M_y = 1.9 (standard impact test) [cite: 3371]
Mx = 1000.0 # Example moment Nmm/mm
My = Mx / 1.9
M_vec = np.array([Mx, My, 0])

# Solve for Curvatures (assuming N=0) [cite: 2347]
# {M} = [B]{e} + [D]{k} -> if B is null, {k} = [D]^-1 {M}
k_vec = np.linalg.solve(D, M_vec)

# Tsai-Wu Coefficients 
F1, F2 = (1/Xt + 1/Xc), (1/Yt + 1/Yc)
F11, F22, F66 = -1/(Xt*Xc), -1/(Yt*Yc), 1/(S**2)
F12 = -0.5 * np.sqrt(F11 * F22) # Stability term

def get_tsai_wu(sigma_global, angle_deg):
    theta = np.radians(angle_deg + beta)
    c, s = np.cos(theta), np.sin(theta)
    T = np.array([[c**2, s**2, 2*s*c], [s**2, c**2, -2*s*c], [-s*c, s*c, c**2-s**2]])
    s1, s2, s12 = T @ sigma_global # Rotate to material axes
    return F1*s1 + F2*s2 + F11*s1**2 + F22*s2**2 + F66*s12**2 + 2*F12*s1*s2

# 5. OUTPUTS
print("ABD Matrix Calculation Complete.")
print(f"A11: {A[0,0]:.2f}, D11: {D[0,0]:.2f}")
print("-" * 30)
print(f"Ply | Angle | Tsai-Wu Index")
for i, angle in enumerate(iso_3r):
    z_mid = (z_coords[i] + z_coords[i+1]) / 2
    sigma_g = get_transformed_q(angle) @ (z_mid * k_vec)
    tw_index = get_tsai_wu(sigma_g, angle)
    print(f"{i+1:3} | {angle:5} | {tw_index:.4f}")