# Classical Lamination Theory Analysis
# DOCUMENTATION AND LABELLING BY GEMINI AI

import numpy as np
import pandas as pd

# 1. MATERIAL PROPERTIES (T300/5208) [cite: 1092]
E1 = 152000.0  # MPa
E2 = 8700.0    # MPa
G12 = 6400.0   # MPa
v12 = 0.23
v21 = v12 * E2 / E1

# Calculate Reduced Stiffness (Q) 
denom = 1 - v12 * v21
Q11 = E1 / denom
Q12 = v12 * E2 / denom
Q22 = E2 / denom
Q66 = G12

# 2. LAMINATE PARAMETERS [cite: 2, 1092]
beta = 30.0  # Your assigned off-axis alignment
t = 0.165    # mm per ply
num_plies = 24
total_thickness = num_plies * t

# Assigned Stacking Sequence ISO-3R [cite: 5]
iso_3r = np.array([
    90, 45, -45, 0, -45, 0, 45, 0, 90, -45, 90, 45,
    90, 45, 0, 45, -45, 90, -45, 0, -45, 0, 90, 45
])

def get_abd_matrix(angles, beta, t, Q_vals):
    A = np.zeros((3, 3))
    B = np.zeros((3, 3))
    D = np.zeros((3, 3))
    
    Q11, Q12, Q22, Q66 = Q_vals
    
    # Starting from the bottom of the laminate [cite: 361, 365]
    z_lower = - (num_plies * t) / 2
    
    for angle in angles:
        theta = np.radians(angle + beta) # Shift by Beta 
        m = np.cos(theta)
        n = np.sin(theta)
        
        # Transformed Reduced Stiffness Q' [cite: 469, 471, 473, 503]
        q11_p = Q11*m**4 + 2*(Q12 + 2*Q66)*m**2*n**2 + Q22*n**4
        q12_p = (Q11 + Q22 - 4*Q66)*m**2*n**2 + Q12*(m**4 + n**4)
        q22_p = Q11*n**4 + 2*(Q12 + 2*Q66)*m**2*n**2 + Q22*m**4
        q66_p = (Q11 + Q22 - 2*Q12 - 2*Q66)*m**2*n**2 + Q66*(m**4 + n**4)
        q16_p = (Q11 - Q12 - 2*Q66)*m**3*n + (Q12 - Q22 + 2*Q66)*m*n**3
        q26_p = (Q11 - Q12 - 2*Q66)*m*n**3 + (Q12 - Q22 + 2*Q66)*m**3*n
        
        Q_prime = np.array([[q11_p, q12_p, q16_p],
                            [q12_p, q22_p, q26_p],
                            [q16_p, q26_p, q66_p]])
        
        z_upper = z_lower + t
        
        # Integration (Summation) [cite: 486, 487, 744]
        A += Q_prime * (z_upper - z_lower)
        B += 0.5 * Q_prime * (z_upper**2 - z_lower**2)
        D += (1/3.0) * Q_prime * (z_upper**3 - z_lower**3)
        
        z_lower = z_upper # Move to next ply interface
        
    return A, B, D

# Execution
A_mat, B_mat, D_mat = get_abd_matrix(iso_3r, beta, t, (Q11, Q12, Q22, Q66))

print("A Matrix (Extensional Stiffness):\n", A_mat)
print("\nB Matrix (Coupling Stiffness):\n", B_mat)
print("\nD Matrix (Bending Stiffness):\n", D_mat)