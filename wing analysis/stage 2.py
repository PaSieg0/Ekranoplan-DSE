import numpy as np

def compute_sweep_theory(
    Cl_interp,  # interpolated Cl at spanwise sections
    sweep_angle,  # sweep angle (Lambda) in radians
    M_inf,        # freestream Mach number
    V_inf,        # freestream velocity
    alpha_i=None, # induced angle of attack (optional), set to None for case (a)
    C_d_eff=None, # effective drag coefficient (optional), set to None for case (a)
    c_l=None,     # local lift coefficient (needed for Re_eff)
    c=None,       # local chord length (needed for Re_eff)
    Re_inf=None   # freestream Reynolds number (needed for Re_eff)
):
    
    # Simple sweep theory
    Cl_perp = Cl_interp * (1 / np.cos(sweep_angle))**2
    M_perp = M_inf * np.cos(sweep_angle)
    V_perp = V_inf * np.cos(sweep_angle)

    while alpha_i - alpha_i
    # Case (a): No induced angle or drag effects
    if alpha_i is None or C_d_eff is None:
        alpha_i = np.zeros_like(Cl_interp)
        C_d_eff = np.zeros_like(Cl_interp)
        Cl_eff = Cl_interp
    else:
        # Case (b): Compute effective Cl with induced angle and drag
        Cl_eff = (Cl_interp * np.cos(alpha_i)**2 + C_d_eff * np.sin(alpha_i)) / np.cos(alpha_i)

    # Case (c): Compute effective velocity and Reynolds number
    V_eff = V_inf / np.cos(alpha_i)

    if c_l is not None and c is not None and Re_inf is not None:
        Re_eff = Re_inf * (V_eff / V_inf) * (c_l / c)
    else:
        Re_eff = None

    return {
        "Cl_perp": Cl_perp,
        "M_perp": M_perp,
        "V_perp": V_perp,
        "alpha_i": alpha_i,
        "C_d_eff": C_d_eff,
        "Cl_eff": Cl_eff,
        "V_eff": V_eff,
        "Re_eff": Re_eff}
print(compute_sweep_theory(
    Cl_interp=np.array([0.5, 0.6, 0.7]),
    sweep_angle=np.radians(0),
    M_inf=0.3,
    V_inf=133,
    alpha_i=np.array([0.1, 0.2, 0.3]),
    C_d_eff=np.array([0.01, 0.02, 0.03]),
    c_l=np.array([1.0, 1.2, 1.4]),
    c=np.array([2.0, 2.2, 2.4]),
    Re_inf=63000000
))