import numpy as np
import matplotlib.pyplot as plt

# 1. MATERIAL PROPERTIES (T300/5208 Graphite/Epoxy)
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
sample_id = "case_01"
beta = 30.0  
t = 0.165    
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

ABBD = np.block([[A, B], [B, D]])

# 5. TSAI-WU COEFFICIENTS & SOLVER
F1 = (1/Xt + 1/Xc)
F2 = (1/Yt + 1/Yc)
F11 = -1/(Xt*Xc)
F22 = -1/(Yt*Yc)
F66 = 1/(S**2)
F12 = -0.5 * np.sqrt(F11 * F22) 

def get_tsai_wu_lambda(sigma_global_unit, angle_deg):
    theta = np.radians(angle_deg + beta)
    c, s = np.cos(theta), np.sin(theta)
    T = np.array([
        [c**2, s**2, 2*s*c], 
        [s**2, c**2, -2*s*c], 
        [-s*c, s*c, c**2-s**2]
    ])
    s1, s2, s12 = T @ sigma_global_unit
    
    A_coeff = F11*s1**2 + F22*s2**2 + F66*s12**2 + 2*F12*s1*s2
    B_coeff = F1*s1 + F2*s2
    
    discriminant = B_coeff**2 - 4 * A_coeff * (-1)
    if discriminant >= 0 and A_coeff != 0:
        lam = (-B_coeff + np.sqrt(discriminant)) / (2 * A_coeff)
        if lam > 0:
            return lam
    return float('inf')

def get_stress_components(angle_deg, z_pos, k_vec, e_vec):
    Qp = get_transformed_q(angle_deg)
    
    # Decompose into strain and curvature components
    sig_strain = Qp @ e_vec
    sig_curv = Qp @ (z_pos * k_vec)
    sig_total = sig_strain + sig_curv
    
    theta = np.radians(angle_deg + beta)
    c, s = np.cos(theta), np.sin(theta)
    T = np.array([[c**2, s**2, 2*s*c], [s**2, c**2, -2*s*c], [-s*c, s*c, c**2-s**2]])
    s1, s2, s12 = T @ sig_total
    
    fi = F1*s1 + F2*s2 + F11*s1**2 + F22*s2**2 + F66*s12**2 + 2*F12*s1*s2
    return sig_total, sig_strain, sig_curv, fi

def evaluate_load_case(k_unit, e_unit):
    lam_crit = float('inf')
    for i, angle in enumerate(iso_3r):
        Qp = get_transformed_q(angle)
        sig_u_bot = Qp @ (e_unit + z_coords[i] * k_unit)
        sig_u_top = Qp @ (e_unit + z_coords[i+1] * k_unit)
        lam_bot = get_tsai_wu_lambda(sig_u_bot, angle)
        lam_top = get_tsai_wu_lambda(sig_u_top, angle)
        lam_crit = min(lam_crit, lam_bot, lam_top)
    
    k_res = k_unit * lam_crit
    e_res = e_unit * lam_crit
    
    # Result containers
    z_v_plot = []
    tot_stresses = ([], [], []) # sx, sy, txy
    str_stresses = ([], [], [])
    cur_stresses = ([], [], [])
    fi_vals = []
    
    for i, angle in enumerate(iso_3r):
        for z in [z_coords[i], z_coords[i+1]]:
            z_v_plot.append(z)
            tot, st, cu, fi = get_stress_components(angle, z, k_res, e_res)
            for j in range(3):
                tot_stresses[j].append(tot[j])
                str_stresses[j].append(st[j])
                cur_stresses[j].append(cu[j])
            fi_vals.append(fi)
            
    return lam_crit, k_res, e_res, z_v_plot, tot_stresses, str_stresses, cur_stresses, fi_vals

# 6. RUN LOAD CASES
# A. Positive Bending
Mx_pos_unit = 1.0 
k_unit_pos = np.linalg.solve(D, np.array([Mx_pos_unit, Mx_pos_unit/1.9, 0]))
res_pos = evaluate_load_case(k_unit_pos, np.zeros(3))

# B. Negative Bending
Mx_neg_unit = -1.0 
k_unit_neg = np.linalg.solve(D, np.array([Mx_neg_unit, Mx_neg_unit/1.9, 0]))
res_neg = evaluate_load_case(k_unit_neg, np.zeros(3))

# C. Buckling (Comp)
Nx_unit = np.array([-1.0, 0, 0])
e_unit_comp = np.linalg.solve(A, Nx_unit)
res_comp = evaluate_load_case(np.zeros(3), e_unit_comp)

# 7. BUCKLING DESIGN SIZING & ITERATIONS
aspect_ratio = 1.5
N_fail = res_comp[0]

# K intermediates
buckling_iterations = []
K_min_val = float('inf')
m_crit_k = 1
for m in range(1, 6):
    t1 = D[0,0] * (m / aspect_ratio)**2
    t2 = 2 * (D[0,1] + 2 * D[2,2])
    t3 = D[1,1] * (aspect_ratio / m)**2
    K = t1 + t2 + t3
    buckling_iterations.append((m, t1, t2, t3, K))
    if K < K_min_val:
        K_min_val, m_crit_k = K, m

# Spreadsheet iteration checks
buckling_tables = []
b_final, a_final, m_final = 0, 0, 0
for m_assume in range(1, 4):
    K_assume = D[0,0]*(m_assume/aspect_ratio)**2 + 2*(D[0,1] + 2*D[2,2]) + D[1,1]*(aspect_ratio/m_assume)**2
    b_calc = np.pi * np.sqrt(K_assume / N_fail)
    a_calc = b_calc * aspect_ratio
    checks = []
    for m_check in range(1, 4):
        t1 = D[0,0] * (m_check / a_calc)**2
        t2 = 2 * (D[0,1] + 2 * D[2,2]) * (1 / b_calc**2)
        t3 = D[1,1] * (1 / b_calc**4) * (a_calc / m_check)**2
        Nx_check = np.pi**2 * (t1 + t2 + t3)
        checks.append((m_check, 1, Nx_check))
    buckling_tables.append((m_assume, a_calc, b_calc, checks))
    if np.argmin([c[2] for c in checks]) + 1 == m_assume and b_final == 0:
        b_final, a_final, m_final = b_calc, a_calc, m_assume

# 8. TERMINAL OUTPUTS (FULL V4 LOGGING)
np.set_printoptions(precision=4, suppress=True, linewidth=100)
print("--- FULL 6x6 ABBD MATRIX ---")
print(ABBD)

print(f"\n--- CRITICAL LOADS (FI = 1.0) ---")
print(f"Bending (+ve): Mx = {res_pos[0]:.2f}, My = {(res_pos[0]/1.9):.2f}")
print(f"Bending (-ve): Mx = {-res_neg[0]:.2f}, My = {(-res_neg[0]/1.9):.2f}")
print(f"Buckling (Comp): Nx = {-res_comp[0]:.2f}")

print("\n--- BUCKLING COEFFICIENT (K) INTERMEDIATES ---")
print(f"{'m':<4} | {'D11 Term':<15} | {'Coupling Term':<15} | {'D22 Term':<15} | {'Total K':<15}")
print("-" * 75)
for m, t1, t2, t3, K in buckling_iterations:
    marker = "  <-- Critical Mode" if m == m_crit_k else ""
    print(f"{m:<4} | {t1:<15.2f} | {t2:<15.2f} | {t3:<15.2f} | {K:<15.2f}{marker}")

print("\n--- SPREADSHEET BUCKLING ITERATION TABLES ---")
for m_assume, a_calc, b_calc, checks in buckling_tables:
    print(f"\nAssumed Mode: m = {m_assume} | a = {a_calc:.4f} | b = {b_calc:.4f}")
    print(f"      m |     n |              Nx")
    print("-" * 35)
    for m_ch, n_ch, Nx_ch in checks:
        marker = "  <== [Matches N_fail]" if np.isclose(Nx_ch, N_fail, rtol=1e-4) else ""
        print(f"{m_ch:7d} | {n_ch:5d} | {Nx_ch:15.6f}{marker}")
    actual_crit = np.argmin([c[2] for c in checks]) + 1
    if actual_crit == m_assume:
        print("Status: VALID")
    else:
        print(f"Status: INVALID (Buckles at m={actual_crit})")

def print_48_row_table(header, res):
    z_v, tot, _, _, fi = res[3], res[4], res[5], res[6], res[7]
    print(f"\n--- {header} ---")
    print(f"{'Ply':<4} | {'z':<8} | {'Sigma_x':<10} | {'Sigma_y':<10} | {'FI':<10}")
    print("-" * 50)
    for i in range(48):
        print(f"{i//2+1:<4} | {z_v[i]:<8.3f} | {tot[0][i]:<10.4f} | {tot[1][i]:<10.4f} | {fi[i]:.6f}")

print_48_row_table("BENDING (+ve)", res_pos)
print_48_row_table("BENDING NEGATIVE (COMPRESSION AT IMPACT)", res_neg)
print_48_row_table("BUCKLING COMPRESSION", res_comp)

# 9. EXPORT PLOTS (V5 STYLE)
def save_plots(case_name, res):
    z_v = res[3]
    titles = ['$\sigma_x$ (MPa)', '$\sigma_y$ (MPa)', r'$\tau_{xy}$ (MPa)', 'Tsai-Wu Index']
    f_suffixes = ['Sigma_x', 'Sigma_y', 'Tau_xy', 'TW']
    for i in range(4):
        fig, ax = plt.subplots(figsize=(8, 8))
        if i < 3:
            ax.plot(res[4][i], z_v, 'r-', label='Total Stress', linewidth=2)
            ax.plot(res[5][i], z_v, 'b--', label='Strain Comp.', linewidth=1.5)
            ax.plot(res[6][i], z_v, 'g--', label='Curv. Comp.', linewidth=1.5)
            ax.legend(loc='best')
        else:
            ax.plot(res[7], z_v, 'r-', linewidth=2)
        ax.set_title(f"{case_name} {titles[i]}", fontweight='bold')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.invert_yaxis()
        plt.tight_layout()
        plt.savefig(f"{sample_id}_{case_name}_{f_suffixes[i]}.png", dpi=300)
        plt.close(fig)

print("\nSaving 12 plots with component decomposition...")
save_plots("Bending_Pos", res_pos)
save_plots("Bending_Neg", res_neg)
save_plots("Buckling_Comp", res_comp)
print("Done.")