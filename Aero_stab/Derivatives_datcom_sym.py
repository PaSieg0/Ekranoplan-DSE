import numpy as np
import json
import os
from utils import Data


class DerivativesDatcom_asym:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.Delta_c4 = aircraft_data.data['outputs']['wing_design']['sweep_c_4'] #degrees
        self.l_b = aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] # length of body 
        self.dihedral = aircraft_data.data['outputs']['wing_design']['dihedral'] #degrees
        self.A = aircraft_data.data['outputs']['wing_design']['aspect_ratio'] 
        self.S = aircraft_data.data['outputs']['wing_design']['S'] 
        self.Av = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['aspect_ratio'] # Aspect ratio of the vertical tail
        self.Ah = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['aspect_ratio']
        self.Sh = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] # Surface area of the horizontal tail: 100
        self.Sv = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S'] # Surface area of the vertical tail: 75
        self.V_b = V_b # total body volume [m3]: 20000
        self.b = aircraft_data.data['outputs']['design']['b']
        self.lp = lp # 36.431
        self.Cl_alpha = Cl_alpha # Lift curve slope of the wing, in rad: 9.167
        self.e = aircraft_data.data['inputs']['oswald_factor'] # Oswald efficiency factor of the wing : 0.85 (guessed, typical value for a subsonic aircraft)
        self.taper = aircraft_data.data['outputs']['wing_design']['taper_ratio'] # Taper ratio of the wing: 0.4
        self.MAC = aircraft_data.data['outputs']['wing_design']['MAC']  # Mean Aerodynamic Chord: 8.456
        self.x_bar = x_ac - x_cg # Distance from the leading edge of the wing to the center of gravity: 31.5(from excel)
        self.Cd0 = aircraft_data.data['inputs']['Cd0'] # Zero-lift drag coefficient of the wing
        self.c_h = c_h
        self.Cm_alpha = Cm_alpha
        self.x_h = x_h
        self.Cl_alpha_h = Cl_alpha_h
        self.x_w = x_w
        self.x_cg = x_cg
        self.M = M
        self.theta_0 = theta_0
        self.Cl_0 = Cl_0

        self.Cl = 0.5
        self.alpha = np.deg2rad(2)  # Example angle of attack in radians

    def Cmq(self):
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
    
    def C_x_alpha(self):
        return self.Cl * (1 - 2*self.Cl_alpha / (np.pi * self.A * self.e))# From FD
    
    def Cmalpha(self):
        # From FD
        downwash_gradient = 2 * self.Cl_alpha / (np.pi * self.A)
        l_h = self.x_h - self.x_cg
        Cmalpha = self.Cl_alpha * (self.x_cg - self.x_w)/self.MAC - self.Cl_alpha_h*(1-downwash_gradient)*self.Sh*l_h / (self.S * self.MAC)
        print(downwash_gradient)
        print(f"Cl_alpha: {self.Cl_alpha}, x_cg: {self.x_cg}, x_w: {self.x_w}, MAC: {self.MAC}, x_h: {self.x_h}, Sh: {self.Sh}, S: {self.S}, l_h: {l_h}")
        return Cmalpha
    
    def C_Z_u(self):
        return -2 * self.Cl #From FD and other source, which tells us that rest can be ignored.
    
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
    
    def update_json(self):
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
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym'] = aero_stability_outputs
        self.aircraft_data.save_design(self.design_file)

    
if __name__ == "__main__":
    aircraft_data = Data('design3.json')
    derivatives = DerivativesDatcom_asym(aircraft_data=aircraft_data)

    derivatives.update_json()


