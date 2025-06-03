import numpy as np
import json
import os


class DerivativesDatcom_asym:
    def __init__(self, Delta_c4, A, S, Av, Sh, Sv, dihedral, l_b, d_fusel, V_b, b, lp, Cl_alpha, e, taper, MAC=8.456, x_ac=31.69, x_cg=30, Cd0 = 0.017, c_h=5.107, Cm_alpha=0, x_h=67.6, Cl_alpha_h=np.rad2deg(0.16), x_w=29.355, M=0.3, theta_0=0, Cl_0=0):
        self.Delta_c4 = Delta_c4 #Quarter chord sweep: 0 
        self.l_b = l_b # length of body: 60 
        self.dihedral = dihedral #1
        self.A = A # aspect ratio of the wing: 8 
        self.S = S # Surface area of the wing: 507
        self.Av = Av # Aspect ratio of the vertical tail: 1.5 
        self.Sh = Sh # Surface area of the horizontal tail: 100
        self.Sv = Sv # Surface area of the vertical tail: 75
        self.d_fusel = d_fusel # Diameter of the fuselage: 6
        self.V_b = V_b # total body volume [m3]: 20000
        self.b = b #: 63.79
        self.lp = lp # 36.431
        self.Cl_alpha = Cl_alpha # Lift curve slope of the wing, in rad: 9.167
        self.e = e # Oswald efficiency factor of the wing : 0.85 (guessed, typical value for a subsonic aircraft)
        self.taper = taper # Taper ratio of the wing: 0.4
        self.MAC = MAC # Mean Aerodynamic Chord: 8.456
        self.x_bar = x_ac - x_cg # Distance from the leading edge of the wing to the center of gravity: 31.5(from excel)
        self.Cd0 = Cd0 # Zero-lift drag coefficient of the wing
        self.c_h = c_h
        self.Cm_alpha = Cm_alpha
        self.x_h = x_h
        self.Cl_alpha_h = Cl_alpha_h
        self.x_w = x_w
        self.x_cg = x_cg
        self.M = M
        self.theta_0 = theta_0
        self.Cl_0 = Cl_0

    def Cmq(self, Cl, alpha):
        """
        Calculate the pitch moment coefficient due to the pitch rate.

        Returns:
        float: Pitch moment coefficient.
        """
        #p.2686
        K_wb = 0.95 #1047
        K_bw = 0.15 #p.1047
        Cmqe = -0.7 * self.Cl_alpha * np.cos(self.Delta_c4) * ((self.A * (0.5 * self.x_bar / self.MAC + 2 * (self.x_bar / self.MAC)**2))/(self.A + 2 * np.cos(self.Delta_c4)) + 1/24 * self.A**3 * np.cos(self.Delta_c4)**2 / (self.A + 6 * np.cos(self.Delta_c4)) + 1/8) #p.2488
        x_m = 32.55 # longtitudinal
        x_c = 75.078 / 2 # cross-sectional
        S_b = self.d_fusel * self.l_b
        CmqB = 2 * self.Cm_alpha * ((1 - x_m/self.l_b)**2 - self.V_b /(S_b * self.l_b)*(x_c/self.l_b - x_m / self.l_b) / ((1-x_m/self.l_b) - self.V_b/(S_b*self.l_b))) #p.2655
        Cm_wb = (K_wb + K_bw)*(self.Sh/self.S)*(self.c_h/self.MAC)**2 * Cmqe + CmqB * (S_b / self.S)*(self.l_b/self.MAC)**2
        
        
        Cm_tail = 2*(K_wb + K_bw)*(self.Sh / self.S) * ((x_c - self.x_h)/self.MAC)**2 * self.Cl_alpha_h #p.2743

        return Cm_wb - Cm_tail
    
    def C_z_alpha(self):
        return -self.Cl_alpha #From FD
    
    def C_x_alpha(self, Cl):
        return Cl * (1 - 2*self.Cl_alpha / (np.pi * self.A * self.e))# From FD
    
    def Cmalpha(self):
        # From FD
        downwash_gradient = 2 * self.Cl_alpha / (np.pi * self.A)
        l_h = self.x_h - self.x_cg
        Cmalpha = self.Cl_alpha * (self.x_cg - self.x_w)/self.MAC - self.Cl_alpha_h*(1-downwash_gradient)*self.Sh*l_h / (self.S * self.MAC)
        print(downwash_gradient)
        print(f"Cl_alpha: {self.Cl_alpha}, x_cg: {self.x_cg}, x_w: {self.x_w}, MAC: {self.MAC}, x_h: {self.x_h}, Sh: {self.Sh}, S: {self.S}, l_h: {l_h}")
        return Cmalpha
    
    def C_Z_u(self, Cl):
        return -2 * Cl #From FD and other source, which tells us that rest can be ignored.
    
    def C_X_u(self):
        Cxu = -3 * self.Cd0 - 3 * self.Cl_0 * np.tan(self.theta_0)
        return Cxu

    def C_m_u(self):
        print('Ignored, see paper, Pitching moment coefficient science direct')
        return 0


    def C_m_alphadot(self):
        # K_wb = 0.95 #1047
        # K_bw = 0.15 #p.1047
        # X_ac__Cr = ...
        # S_b = self.d_fusel * self.l_b
        # Cl_alphadot = 1.5 * X_ac__Cr * Cl_alpha + 3 * Clg
        # Cm_alphadot_e = Cm_alphadot_dp + (self.x_cg / self.MAC) * Cl_alphadot
        # Cm_alphadot_B = ...
        # Cm_alphadot_wb = (K_bw + K_wb) * (self.Sh/self.S)*(self.c_h/self.MAC)**2 * Cm_alphadot_e + Cm_alphadot_B * ((S_b / self.S)*(self.l_b/self.MAC)**2)
        l_h = self.x_h - self.x_cg
        downwash_gradient = 2 * self.Cl_alpha / (np.pi * self.A)
        print(downwash_gradient)
        Cm_alphadot = -self.Cl_alpha * 1 * downwash_gradient *self.Sh*l_h**2 / (self.S * self.MAC**2)
        return np.deg2rad(Cm_alphadot)

    

derivatives = DerivativesDatcom_asym(0, 8, 507, 1.5, 100, 75, 1, 60, 6, 20000, 63.79, 36.431, 9.167, 0.85, 0.4)
print(derivatives.Cmq(1, np.deg2rad(2)))  # Example usage
# print(obj.C_x_alpha(1))
print(f'cmq:{derivatives.Cmq(1, np.deg2rad(2))}')  # Example usage


# Run the functions you want to store
aero_stability_outputs = {
    'C_m_q': derivatives.Cmq(1, np.deg2rad(2)), #Cl=1, alpha=2 degrees
    'C_z_alpha': derivatives.C_z_alpha(),
    'C_x_alpha': derivatives.C_x_alpha(1),  # Cl=1
    'C_m_alpha': derivatives.Cmalpha(),
    'C_Z_u': derivatives.C_Z_u(1),  # Cl=1
    'C_X_u': derivatives.C_X_u(),
    'C_m_u': derivatives.C_m_u(),
    'C_m_alphadot': derivatives.C_m_alphadot()

    # Add more functions here if needed
}

# Define path to JSON
json_path = os.path.join('Data', 'design3.json')

# Load existing JSON or create new structure
if os.path.exists(json_path):
    with open(json_path, 'r') as file:
        data = json.load(file)
else:
    data = {}

# Ensure correct nested structure exists
if 'outputs' not in data:
    data['outputs'] = {}
if 'aerodynamic_stability_coefficients_sym' not in data['outputs']:
    data['outputs']['aerodynamic_stability_coefficients_sym'] = {}

# Update values
data['outputs']['aerodynamic_stability_coefficients_sym'].update(aero_stability_outputs)

# Save back to file
with open(json_path, 'w') as file:
    json.dump(data, file, indent=4)
