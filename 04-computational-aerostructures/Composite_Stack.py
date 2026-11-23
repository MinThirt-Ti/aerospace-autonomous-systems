import numpy as np

# 1. MATERIAL PROPERTIES (T300/5208 Graphite/Epoxy)
# Values from Report Template Table 1
E1 = 152000.0   # MPa
E2 = 8700.0     # MPa
G12 = 6400.0    # MPa
v12 = 0.23
v21 = v12 * E2 / E1

# Strength Data for Tsai-Wu
Xt, Xc = 1500.0, -1250.0
Yt, Yc = 50.0, -200.0
S = 100.0

# 2. LAMINATE SETUP (ID Case01)
beta = 30.0  # Off-axis alignment
t = 0.165    # mm per ply
num_plies = 24
H = num_plies * t

# ISO-3R Stacking Sequence
iso_3r = np.array([
    90, 45, -45, 0, -45, 0, 45, 0, 90, -45, 90, 45, 
    90, 45, 0, 45, -45, 90, -45, 0, -45, 0, 90, 45
])

# 3. REDUCED STIFFNESS (Q)
denom = 1 - v12 * v21
Q11 = E1 / denom
Q12 = v12 * E2 / denom
Q22 = E2 / denom
Q66 = G12

def get_transformed_q(angle_deg):
    theta = np.radians(angle_deg + beta)
    m, n = np.cos(theta), np.sin(theta)
    
    # York (2010) Transformed Stiffness Formulas
    q11_p = Q11*m**4 + 2*(Q12 + 2*Q66)*m**2*n**2 + Q22*n**4
    q12_p = (Q11 + Q22 - 4*Q66)*m**2*n**2 + Q12*(m**4 + n**4)
    q22_p = Q11*n**4 + 2*(Q12 + 2*Q66)*m**2*n**2 + Q22*m**4
    q66_p = (Q11 + Q22 - 2*Q12 - 2*Q66)*m**2*n**2 + Q66*(m**4 + n**4)
    q16_p = (Q11 - Q12 - 2*Q66)*m**3*n + (Q12 - Q22 + 2*Q66)*m*n**3
    q26_p = (Q11 - Q12 - 2*Q66)*m*n**3 + (Q12 - Q22 + 2*Q66)*m**3*n
    
    return np.array([[q11_p, q12_p, q16_p], 
                     [q12_p, q22_p, q26_p], 
                     [q16_p, q26_p, q66_p]])

# 4. SUMMATION FOR ABD MATRICES
A = np.zeros((3,3))
B = np.zeros((3,3))
D = np.zeros((3,3))
z_coords = np.linspace(-H/2, H/2, num_plies + 1)

for i, angle in enumerate(iso_3r):
    zk, zk_1 = z_coords[i+1], z_coords[i]
    Qp = get_transformed_q(angle)
    A += Qp * (zk - zk_1)
    B += 0.5 * Qp * (zk**2 - zk_1**2)
    D += (1/3.0) * Qp * (zk**3 - zk_1**3)

# Assemble 6x6 constitutive matrix
ABBD = np.block([[A, B], [B, D]])

# 5. FAILURE ANALYSIS (TSAI-WU)
F1 = (1/Xt + 1/Xc)
F2 = (1/Yt + 1/Yc)
F11 = -1/(Xt*Xc)
F22 = -1/(Yt*Yc)
F66 = 1/(S**2)
F12 = -0.5 * np.sqrt(F11 * F22) 

def get_tsai_wu_lambda(angle_deg, z_pos, k_vec_u):
    theta = np.radians(angle_deg + beta)
    Qp = get_transformed_q(angle_deg)
    sigma_global_u = Qp @ (z_pos * k_vec_u) 
    
    c, s = np.cos(theta), np.sin(theta)
    T = np.array([
        [c**2, s**2, 2*s*c], 
        [s**2, c**2, -2*s*c], 
        [-s*c, s*c, c**2-s**2]
    ])
    s1, s2, s12 = T @ sigma_global_u
    
    A_coeff = F11*s1**2 + F22*s2**2 + F66*s12**2 + 2*F12*s1*s2
    B_coeff = F1*s1 + F2*s2
    
    # Quadratic equation: A*lam^2 + B*lam - 1 = 0
    discriminant = B_coeff**2 - 4 * A_coeff * (-1)
    if discriminant >= 0 and A_coeff != 0:
        lam = (-B_coeff + np.sqrt(discriminant)) / (2 * A_coeff)
        if lam > 0:
            return lam
    return float('inf')

def get_stresses_and_fi(angle_deg, z_pos, k_vec):
    theta = np.radians(angle_deg + beta)
    Qp = get_transformed_q(angle_deg)
    
    # Global stress (sigma_x, sigma_y, tau_xy)
    sigma_global = Qp @ (z_pos * k_vec) 
    
    # Rotate to Material Axes (1, 2) for Tsai-Wu
    c, s = np.cos(theta), np.sin(theta)
    T = np.array([
        [c**2, s**2, 2*s*c], 
        [s**2, c**2, -2*s*c], 
        [-s*c, s*c, c**2-s**2]
    ])
    s1, s2, s12 = T @ sigma_global
    
    fi = F1*s1 + F2*s2 + F11*s1**2 + F22*s2**2 + F66*s12**2 + 2*F12*s1*s2
    return sigma_global, fi

# Find critical load multiplier for Bottom-Surface Impact
# A bottom-surface impact bends the plate upward causing tension on the top surface
Mx_unit = -1.0 
My_unit = Mx_unit / 1.9 
k_vec_unit = np.linalg.solve(D, np.array([Mx_unit, My_unit, 0]))

load_multiplier = float('inf')
for i, angle in enumerate(iso_3r):
    lam_bot = get_tsai_wu_lambda(angle, z_coords[i], k_vec_unit)
    lam_top = get_tsai_wu_lambda(angle, z_coords[i+1], k_vec_unit)
    load_multiplier = min(load_multiplier, lam_bot, lam_top)

Mx = Mx_unit * load_multiplier
My = My_unit * load_multiplier
k_vec = k_vec_unit * load_multiplier

# 6. PRINT RESULTS
np.set_printoptions(precision=4, suppress=True, linewidth=100)

print("--- INDIVIDUAL MATRICES ---")
print("A Matrix:\n", A)
print("\nB Matrix:\n", B)
print("\nD Matrix:\n", D)

print("\n--- FULL 6x6 ABBD MATRIX ---")
print(ABBD)

print("\n--- CRITICAL LOAD (FI = 1.0) ---")
print(f"Mx: {Mx:.2f} Nmm/mm")
print(f"My: {My:.2f} Nmm/mm")

print("\n--- TSAI-WU COEFFICIENTS ---")
print(f"F1:  {F1:.5e}")
print(f"F2:  {F2:.5e}")
print(f"F11: {F11:.5e}")
print(f"F22: {F22:.5e}")
print(f"F66: {F66:.5e}")

print("\n--- PLY-BY-PLY ANALYSIS (TOP & BOTTOM SURFACES) ---")
print(f"{'Ply':<4} | {'z (mm)':<8} | {'Angle':<6} | {'Sigma_x':<10} | {'Sigma_y':<10} | {'Tau_xy':<10} | {'FI (Tsai-Wu)':<12}")
print("-" * 85)

# Arrays for plotting
z_vals = []
sigma_x_vals = []
sigma_y_vals = []
tau_xy_vals = []

for i, angle in enumerate(iso_3r):
    z_bot = z_coords[i]
    z_top = z_coords[i+1]
    
    sigma_g_bot, fi_bot = get_stresses_and_fi(angle, z_bot, k_vec)
    print(f"{i+1:<4} | {z_bot:<8.3f} | {angle:<6} | {sigma_g_bot[0]:<10.4f} | {sigma_g_bot[1]:<10.4f} | {sigma_g_bot[2]:<10.4f} | {fi_bot:.6f}")
    
    sigma_g_top, fi_top = get_stresses_and_fi(angle, z_top, k_vec)
    print(f"{i+1:<4} | {z_top:<8.3f} | {angle:<6} | {sigma_g_top[0]:<10.4f} | {sigma_g_top[1]:<10.4f} | {sigma_g_top[2]:<10.4f} | {fi_top:.6f}")
    
    # Store values sequentially to create the correct interface jumps in the plot
    z_vals.extend([z_bot, z_top])
    sigma_x_vals.extend([sigma_g_bot[0], sigma_g_top[0]])
    sigma_y_vals.extend([sigma_g_bot[1], sigma_g_top[1]])
    tau_xy_vals.extend([sigma_g_bot[2], sigma_g_top[2]])

# 7. GENERATE THROUGH-THICKNESS STRESS PLOTS
import matplotlib.pyplot as plt

fig, axs = plt.subplots(1, 3, figsize=(18, 6), sharey=True)

# Common plot settings
titles = [r'$\sigma_x$ (MPa) - Combined strain and curvature', 
          r'$\sigma_y$ (MPa) - Combined strain and curvature', 
          r'$\tau_{xy}$ (MPa) - Combined strain and curvature']
x_data = [sigma_x_vals, sigma_y_vals, tau_xy_vals]

for i in range(3):
    axs[i].plot(x_data[i], z_vals, color='red', linewidth=2)
    axs[i].set_title(titles[i], fontweight='bold', pad=15)
    axs[i].set_xlabel('Stress (MPa)', fontweight='bold')
    if i == 0:
        axs[i].set_ylabel('z (mm)', fontweight='bold')
    
    # Configure axes to match standard engineering plots
    axs[i].axhline(0, color='black', linewidth=1.5)
    axs[i].axvline(0, color='black', linewidth=1.5)
    axs[i].grid(True, linestyle='--', alpha=0.7)
    axs[i].invert_yaxis() # Standard convention: z is negative at top surface, positive at bottom

plt.tight_layout()
plt.show()