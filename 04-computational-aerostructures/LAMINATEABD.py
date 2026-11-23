import numpy as np

def parse_stacking_sequence(stack_string):
    # Strip formatting and split
    clean_str = stack_string.replace('[', '').replace(']T', '')
    plies = clean_str.split('/')
    
    n_total = len(plies)
    n_phi = sum(1 for p in plies if 'f' in p)
    n_psi = sum(1 for p in plies if 'y' in p)
    
    n_prop = (n_phi/n_total, n_psi/n_total)
    
    # Calculate bending distribution (z^3)
    h = 1.653
    t = h / n_total
    zeta_phi = 0
    zeta_psi = 0
    
    for i, ply in enumerate(plies):
        z_bot = -h/2 + i*t
        z_top = -h/2 + (i+1)*t
        val = (z_top**3 - z_bot**3) / 3
        
        if 'f' in ply:
            zeta_phi += val
        elif 'y' in ply:
            zeta_psi += val
            
    total_zeta = h**3 / 12
    zeta_prop = (zeta_phi/total_zeta, zeta_psi/total_zeta)
    
    return n_prop, zeta_prop

def get_abd_matrix(h, angles, proportions_n, proportions_zeta, mat_props):
    E1 = mat_props[0] * 1000
    E2 = mat_props[1] * 1000
    G12 = mat_props[2] * 1000
    v12 = mat_props[3]
    v21 = v12 * E2 / E1
    
    denom = 1 - v12 * v21
    Q11 = E1 / denom
    Q22 = E2 / denom
    Q12 = v12 * E2 / denom
    Q66 = G12
    
    def transform_Q(deg):
        theta = np.radians(deg)
        c, s = np.cos(theta), np.sin(theta)
        c2, s2 = c**2, s**2
        c4, s4 = c**4, s**4
        
        Q11_b = Q11*c4 + 2*(Q12 + 2*Q66)*s2*c2 + Q22*s4
        Q22_b = Q11*s4 + 2*(Q12 + 2*Q66)*s2*c2 + Q22*c4
        Q12_b = (Q11 + Q22 - 4*Q66)*s2*c2 + Q12*(s4 + c4)
        Q66_b = (Q11 + Q22 - 2*Q12 - 2*Q66)*s2*c2 + Q66*(s4 + c4)
        
        Q16_b = (Q11 - Q12 - 2*Q66)*s*c**3 + (Q12 - Q22 + 2*Q66)*s**3*c
        Q26_b = (Q11 - Q12 - 2*Q66)*s**3*c + (Q12 - Q22 + 2*Q66)*s*c**3
        
        return np.array([[Q11_b, Q12_b, Q16_b], 
                         [Q12_b, Q22_b, Q26_b], 
                         [Q16_b, Q26_b, Q66_b]])

    Q_phi = (transform_Q(angles[0]) + transform_Q(-angles[0])) / 2.0
    Q_psi = (transform_Q(angles[1]) + transform_Q(-angles[1])) / 2.0
    
    A = h * (proportions_n[0] * Q_phi + proportions_n[1] * Q_psi)
    D = (h**3 / 12) * (proportions_zeta[0] * Q_phi + proportions_zeta[1] * Q_psi)
    
    return A, D

# Inputs
thickness = 1.653
material = (138.0, 8.96, 7.1, 0.3)
stack = "[y/-y/-f/f/-y/y/f/-f/f/-f/f/-f/-f/f/-f/f/-f/f/-y/y/f/-f/y/-y]T"

# Process Stack
n_prop, zeta_prop = parse_stacking_sequence(stack)

print(f"Calculated n_prop: (phi: {n_prop[0]:.2f}, psi: {n_prop[1]:.2f})")
print(f"Calculated zeta_prop: (phi: {zeta_prop[0]:.2f}, psi: {zeta_prop[1]:.2f})")
print("-" * 30)

cases = {
    "(3) {+/-27.1 / +/-74.3}": (27.1, 74.3),
    "(4) {+/-62.9 / +/-15.7}": (62.9, 15.7)
}

for label, angles in cases.items():
    A, D = get_abd_matrix(thickness, angles, n_prop, zeta_prop, material)
    print(f"\nLaminate: {label}")
    print("A Matrix (N/mm):")
    print(np.round(A, 1))
    print("D Matrix (N-mm):")
    print(np.round(D, 1))