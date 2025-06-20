import numpy as np
import json
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, MissionType, ISA, AircraftType
from aero.lift_curve import lift_curve

class DerivativesDatcom_sym:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.lift_curve = lift_curve()
        self.Delta_c4 = aircraft_data.data['outputs']['wing_design']['sweep_c_4'] #degrees
        self.l_b = aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] # length of body 
        self.dihedral = aircraft_data.data['outputs']['wing_design']['dihedral'] #degrees
        self.A = aircraft_data.data['outputs']['wing_design']['aspect_ratio'] 
        self.S = aircraft_data.data['outputs']['wing_design']['S'] 
        self.Av = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['aspect_ratio'] # Aspect ratio of the vertical tail
        self.Ah = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['aspect_ratio']
        self.Sh = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] # Surface area of the horizontal tail: 100
        self.Sv = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S'] # Surface area of the vertical tail: 75
        self.V_b = aircraft_data.data['outputs']['fuselage_dimensions']['total_volume'] # total body volume [m3]: 20000
        self.b = aircraft_data.data['outputs']['design']['b']
        self.lp = (aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station3'] / 2) + (aircraft_data.data['outputs']['empennage_design']['vertical_tail']['z_MAC'])  # ℓₚ is the distance parallel to the longitudinal body axis between the vehicle moment center and the quarter-chord point of the MAC of the vertical panel, positive for the panel aft of the vehicle moment center.
        self.Cl_alpha = self.lift_curve.dcl_dalpha()[0]*180/np.pi # Lift curve slope of the wing, in rad: 9.167
        self.e = aircraft_data.data['inputs']['oswald_factor'] # Oswald efficiency factor of the wing : 0.85 (guessed, typical value for a subsonic aircraft)
        self.taper = aircraft_data.data['outputs']['wing_design']['taper_ratio'] # Taper ratio of the wing: 0.4
        self.MAC = aircraft_data.data['outputs']['wing_design']['MAC']  # Mean Aerodynamic Chord: 8.456
        self.x_cg = aircraft_data.data['outputs']['cg_range']['most_aft_cg'] 
        self.x_ac = aircraft_data.data['outputs']['wing_design']['X_LEMAC'] +  aircraft_data.data['outputs']['wing_design']['MAC']*0.25 # Aerodynamic center of the wing: 0.25 * MAC
        self.x_bar = self.x_ac - self.x_cg # Distance from the leading edge of the wing to the center of gravity: 31.5(from excel)
        self.Cd0 = aircraft_data.data['inputs']['Cd0'] # Zero-lift drag coefficient of the wing
        self.c_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['MAC'] # Mean aerodynamic chord of the horizontal tail
        self.l_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']
        self.Cl_alpha_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['Cl_alpha'] * (180 / np.pi)
        self.x_w = self.x_ac
        isa = ISA(self.aircraft_data.data['inputs']['cruise_altitude'])
        self.M = isa.Mach(self.aircraft_data.data['requirements']['cruise_speed'])
        self.theta_0 = 0
        self.Cl_0 = 0
        self.d_fusel = aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station2']  # Diameter of the fuselage, assumed to be 0 for now
        self.S_b = aircraft_data.data['outputs']['fuselage_dimensions']['body_base_area'] # body base area
        self.C_r = aircraft_data.data['outputs']['wing_design']['chord_root']
        self.x_h=self.x_cg+self.l_h
        self.l_f=aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']

        self.Cl = 0.5
        self.alpha = np.deg2rad(2)  # Example angle of attack in radians

    def Cmq(self): # Cross validated with the help of Mr. Antens
        """
        Calculate the pitch moment coefficient due to the pitch rate.

        Returns:
        float: Pitch moment coefficient.
        """
        #p.2686
        K_wb = 1.3 #0.95 #1047
        K_bw = 0.4 #0.3 #p.1047
        Cmqe = -0.7 * self.Cl_alpha * np.cos(np.radians(self.Delta_c4)) * ((self.A * (0.5 * self.x_bar / self.MAC + 2 * (self.x_bar / self.MAC)**2))/(self.A + 2 * np.cos(np.radians(self.Delta_c4))) + 1/24 * self.A**3 * np.tan(np.radians(self.Delta_c4))**2 / (self.A + 6 * np.cos(np.radians(self.Delta_c4))) + 1/8) #p.2488
        
        self.MAC_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['MAC']  # Mean Aerodynamic Chord of the horizontal tail
        self.x_LEMAC = self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        x_m = self.x_cg # longtitudinal
        x_c = self.l_f/2 # cross-sectional
        S_b = self.S_b
        CmqB = 2 * self.Cmalpha() * (((1 - x_m/self.l_b)**2 - self.V_b /(S_b * self.l_b)*(x_c/self.l_b - x_m / self.l_b)) / ((1-x_m/self.l_b) - self.V_b/(S_b*self.l_b))) #p.2655
        Cm_wb = (K_wb + K_bw)*(self.Sh/self.S)*(self.c_h/self.MAC)**2 * Cmqe + CmqB * (S_b / self.S)*(self.l_b/self.MAC)**2
        
        Cm_tail = 2*(K_wb + K_bw)*(self.Sh / self.S) * ((x_m - (self.x_h+self.MAC_h/1.8))/self.MAC)**2 * (self.Cl_alpha_h) #p.2743
        # print(f'Sh: {self.Sh}, S: {self.S}, MAC: {self.MAC}, x_m: {x_m}, x_h: {self.x_h}, Cl_alpha_h: {self.Cl_alpha_h}')
        # print(f'xm-xcg= {x_m - self.x_h}) xm: {x_m}, x_h: {self.x_h}')
        # print(Cm_wb, Cm_tail)
        # print(f'final Cmq: {Cm_wb - Cm_tail}')
        print(Cm_wb - Cm_tail)
        return Cm_wb - Cm_tail
    
    def C_z_alpha(self):
        return -self.Cl_alpha #From FD
    
    def C_x_alpha(self):
        return self.Cl * (1 - 2*self.Cl_alpha / (np.pi * self.A * self.e))# From FD
    
    def Cmalpha(self):
        # From FD
        downwash_gradient = 2 * self.Cl_alpha / (np.pi * self.A)
        # downwash_gradient=0
        # l_h = self.x_h - self.x_cg
        Cmalpha = self.Cl_alpha * (self.x_cg - self.x_w)/self.MAC - self.Cl_alpha_h*(1-downwash_gradient)*self.Sh*self.l_h / (self.S * self.MAC)
        # print(self.Cl_alpha * (self.x_cg - self.x_w)/self.MAC, self.Cl_alpha_h*(1-downwash_gradient)*self.Sh*l_h / (self.S * self.MAC))
        return Cmalpha
    
    def C_Z_u(self):
        return -2 * self.Cl #From FD and other source, which tells us that rest can be ignored.
    
    def C_X_u(self):
        Cxu = -3 * self.Cd0 - 3 * self.Cl_0 * np.tan(np.radians(self.theta_0))
        return Cxu

    def C_m_u(self):
        # 'Ignored, see paper, Pitching moment coefficient science direct'), 
        return 0

    def C_m_alphadot(self): # Value is now is negative, should be positive ~0.2, From sensitivity analysis with SVV a larger negative value makes the SS more alike real life conditions.
        K_wb = 0.95 #p.1047
        K_bw = 0.15 #p.1047
        # X_ac__Cr = ((2/3)*(1-self.taper) + 1/2 * (1 - (self.taper**2/(1+self.taper))) * np.pi * np.log(1 + self.A/5)) / (1 + np.pi * np.log(1+self.A/5)) #p.664
        X_ac__Cr = 0.18 #p.688
        S_b = self.d_fusel * self.l_b
        Clg = 0.0388 # p.2604. Some liberties were taken with the graph reading
        Cm0_g = 0.08 #p.2624. Again, some liberties were taken
        Cl_alphadot = 1.5 * X_ac__Cr * self.Cl_alpha + 3 * Clg
        Cm_alphadot_dp = -81/32 * (self.x_ac / self.C_r)**2 * self.Cl_alpha + 9/2 * Cm0_g
        Cm_alphadot_e = Cm_alphadot_dp + (self.x_cg / self.MAC) * Cl_alphadot #p.2617

        # x_c = 0.5 * self.l_b
        # x_m = self.x_cg
        # S_b = self.S_b

        # Cm_alphadot_B = 2 * self.Cmalpha() * ((self.V_b / (S_b * self.l_b)) * ((x_c / self.l_b) - (x_m / self.l_b)) / (((1 - (x_m / self.l_b)) * self.V_b / (S_b * self.l_b))))
        # Cm_alphadot_wb = (K_bw + K_wb) * (self.Sh/self.S)*(self.c_h/self.MAC)**2 * Cm_alphadot_e + Cm_alphadot_B * ((S_b / self.S)*(self.l_b/self.MAC)**2)
        # print((K_bw + K_wb) * (self.Sh/self.S)*(self.c_h/self.MAC)**2 * Cm_alphadot_e, Cm_alphadot_B * ((S_b / self.S)*(self.l_b/self.MAC)**2))
        # print(Cm_alphadot_B, S_b / self.S,(self.l_b/self.MAC)**2)
        
        downwash_gradient = (2 * self.Cl_alpha / (np.pi * self.A))
        Cm_alphadot_tail = -(self.Cl_alpha * 1 * downwash_gradient * self.Sh*self.l_h**2) / (self.S * self.MAC**2)
        return Cm_alphadot_tail #No body contribution, see FD
    
    def update_json(self):
        # Run the functions you want to store
        # aero_stability_outputs = {
        #     'C_m_q': self.Cmq(), #Cl=1, alpha=2 degrees
        #     'C_z_alpha': self.C_z_alpha(),
        #     'C_x_alpha': self.C_x_alpha(),  # Cl=1
        #     'C_m_alpha': self.Cmalpha(),
        #     'C_Z_u': self.C_Z_u(),  # Cl=1
        #     'C_X_u': self.C_X_u(),
        #     'C_m_u': self.C_m_u(),
        #     'C_m_alphadot': self.C_m_alphadot()

        #     # Add more functions here if needed
        # }
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_m_q'] = self.Cmq()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_z_alpha'] = self.C_z_alpha()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_x_alpha'] = self.C_x_alpha()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_m_alpha'] = self.Cmalpha()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_Z_u'] = self.C_Z_u()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_X_u'] = self.C_X_u()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_m_u'] = self.C_m_u()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_m_alphadot'] = self.C_m_alphadot()
        # self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym'] = aero_stability_outputs
        self.aircraft_data.save_design('design3.json')

    
if __name__ == "__main__":
    aircraft_data = Data('design3.json')
    derivatives = DerivativesDatcom_sym(aircraft_data=aircraft_data)

    derivatives.update_json()


