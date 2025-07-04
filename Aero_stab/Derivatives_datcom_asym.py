import numpy as np
import json
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, MissionType, ISA, AircraftType
from aero.lift_curve import lift_curve

class DerivativesDatcom_asym:
    def __init__(self, aircraft_data: Data) -> None:
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
        self.d_fusel = aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station2'] # Diameter of the fuselage
        self.V_b = aircraft_data.data['outputs']['fuselage_dimensions']['total_volume'] # total body volume [m3]
        self.b = aircraft_data.data['outputs']['design']['b']

        self.lp = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v']
        self.zp = (aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station3'] / 2) + (aircraft_data.data['outputs']['empennage_design']['vertical_tail']['z_MAC'])  
        self.Cl_alpha = self.lift_curve.dcl_dalpha()[0]*(180*np.pi) # Lift curve slope of the wing in deg
        self.e = aircraft_data.data['inputs']['oswald_factor'] # Oswald efficiency factor of the wing : 0.85 (guessed, typical value for a subsonic aircraft)
        self.taper = aircraft_data.data['outputs']['wing_design']['taper_ratio'] # Taper ratio of the wing: 0.4
        self.MAC =  aircraft_data.data['outputs']['wing_design']['MAC']  # Mean Aerodynamic Chord
        self.x_bar = aircraft_data.data['outputs']['wing_design']['X_LEMAC']  - aircraft_data.data['outputs']['cg_range']['most_aft_cg'] #x_ac - x_cg # Distance from the leading edge of the wing to the center of gravity: 31.5(from excel)
        self.Cd0 = aircraft_data.data['inputs']['Cd0'] # Zero-lift drag coefficient of the wing

        self.c_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['MAC']
        self.alpha = aircraft_data.data['inputs']['alpha_cruise'] # Angle of attack in degrees, TODO: UPDATE!!
        self.Cl = 0.5   # Lift coefficient at the angle of attack, TODO: UPDATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.alpha_f = aircraft_data.data['inputs']['alpha_fuselage']*(180*np.pi) # Fuselage AoA in degrees, TODO: UPDATE!!
        self.h_b = aircraft_data.data['inputs']['cruise_altitude'] / self.b 
        isa = ISA(self.aircraft_data.data['inputs']['cruise_altitude'])
        self.M = isa.Mach(self.aircraft_data.data['requirements']['cruise_speed'])
        self.d_f = aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station3']
        self.Cl_alpha_Vtail = aircraft_data.data['outputs']['aerodynamics']['vertical/horizontal stab cl_alpha']*180/np.pi


    def CyB(self):
        """
        Calculate the side force coefficient due to the elevator deflection.

        Parameters:
        CL (float): Lift coefficient.

        Returns:
        tuple(float: total Side force coefficient, side force due to tail(needed for Cyp).
        """
        z_w = self.d_fusel / 2 # Distance from centerline wing to centerline body
        d = self.d_fusel # Max body height at wing intersection
        K = -2*1.85*(z_w / d)# From plot

        delta_CyB_dihedral = -0.0001* np.radians(self.dihedral)
        k2__k1 = 0.9 # Determined from plot page 834 DATCOM

        S0 = self.aircraft_data.data['outputs']['fuselage_dimensions']['cross_sectional_area_2'] # Cross sectional area of the body at the wing intersection [m2]
        Cl_alpha_body = 2*(k2__k1)* S0 / (self.V_b ** (2/3)) # Body lift curve slope [1/rad]

        CyB_body = -Cl_alpha_body
        S_ref_body = self.l_b * self.d_fusel
        S_ref_wing = self.S

        A_vbA_v = 1.2 # b_v / 2 * r, if we know empennage sizing->look at page 1667
        Kh = 0.8 # Depends on Sh/Sv-> page 1668 DATCOM
        Av_hb__Av_b = 1.0 # Depends on zh/bh, thus for a t-tail equal to 1 ->p1667
        self.A_eff_Vstab = A_vbA_v * self.Av * (1+Kh*(Av_hb__Av_b - 1))
        # Cl_alpha_Vtail = 0.16 # WIG, NACA 0012
        sidewash_coeff = 0.724 + 3.06 * (self.Sv / self.S) / (1 + np.cos(np.radians(self.Delta_c4))) +0.4*z_w / self.d_fusel + 0.009*self.A


        # CyB_wing  = Cl**2 (6 * np.tan(self.Delta_c4) / (np.pi * self.A * (self.A + 4 * np.cos(self.Delta_c4)))) * 1/ 57.3

        CyB_wing_body = K * CyB_body * (S_ref_body / S_ref_wing) * delta_CyB_dihedral # TODO: Check formula for angle unit
        CyB_tail = - 1 * self.Cl_alpha_Vtail * sidewash_coeff *self.Sv / self.S

        return CyB_wing_body + CyB_tail, CyB_tail
    
    def ClB(self):
        clB__cl_s = -0.002/5 # from plot->p1563
        Km_s = 1.03 #from plot p1564
        kf = 0.95 # from plot p1625
        clB__Cl_A = 0 #from plot
        clB__dihedral = -0.00024 #1/deg^2 #p1565
        Km_d = 1 #p1566
        dclB__dihedral = -0.0005 * np.sqrt(self.A)*(self.d_fusel / self.b)**2
        z_w = -self.d_fusel/2
        dclB_zw = 1.2 * np.sqrt(self.A) / 57.3 * (2/self.b) * (2 * self.d_fusel / self.b) * z_w

        #Dihedral is in degrees
        ClB_wb = self.Cl *(clB__cl_s*Km_s*kf+clB__Cl_A) + self.dihedral * (clB__dihedral * Km_d + dclB__dihedral) + dclB_zw + 0

        K = 1.4 #p.1600 smthng
        # cl_alpha_V_tail = 0.16*180/np.pi #p.1600 smthng
        dcyB_Vtail = -K * self.Cl_alpha_Vtail * self.Sv / self.S
        zp = self.zp
        dclB_Vtail = dcyB_Vtail * (zp * np.cos(np.radians(self.alpha)) - self.lp*np.sin(np.radians(self.alpha))) / self.b #changed the sin to also take in radians

        # print(ClB_wb + dclB_Vtail)
        # print('ClB:',ClB_wb, dclB_Vtail)
        # print(f'final ClB: {ClB_wb + dclB_Vtail}')
        return ClB_wb + dclB_Vtail
    
    def CnB(self):
        K_N = 0.0018 #p.1633
        K_R_l = 1.65 #p.1634
        S_B_s = self.d_fusel * self.l_b # Cross sectional area of the body at the wing intersection [m2]
        CnB_wb = -K_N * K_R_l * (S_B_s / self.S) * (self.l_b / self.b)

        K = 1.4 #p.1600 smthng
        # cl_alpha_V_tail = 0.16 #p.1600 smthng
        dcyB_Vtail = -K * self.Cl_alpha_Vtail * self.Sv / self.S
        CnB_Vtail = -dcyB_Vtail * (self.lp / self.b)
        return CnB_wb + CnB_Vtail, CnB_wb, CnB_Vtail
    
    def Cyp(self): 
        """
        Calculate the side force coefficient due to the roll rate.

        Returns:
        float: Side force coefficient.
        """
        sigma=np.exp(-2.48*(self.h_b)**(0.768))
        zp = self.zp # Distance from the centerline of the body to the centerline of the vertical tail
        z = zp * np.cos(np.radians(self.alpha)) - self.lp * np.sin(np.radians(self.alpha)) # distance from the centerline of the body to the centerline of the wing
        K =  (self.Cl_alpha * np.tan(np.radians(self.alpha)) + self.Cl / (np.cos(np.radians(self.alpha)))**2 - 2 * self.Cl / (np.pi * self.A * self.e) * (1 - sigma) * self.Cl_alpha) / (self.Cl_alpha * np.tan(np.radians(self.alpha)) + self.Cl / (np.cos(np.radians(self.alpha)))**2 - 2 * self.Cl / (np.pi * self.A * self.e) * self.Cl_alpha)
        B = np.sqrt(1 - self.M**2 * np.cos(np.radians(self.Delta_c4))**2) # p.2522
        Cyp__Cl_0 = -0.06 #p2529
        Cyp__Cl = ((self.A + 4 * np.cos(np.radians(self.Delta_c4))) * (self.A * B + np.cos(np.radians(self.Delta_c4))) * Cyp__Cl_0) / ((self.A * B + 4 * np.cos(np.radians(self.Delta_c4))) * (self.A + np.cos(np.radians(self.Delta_c4))))
        # print(Cyp__Cl)
        clp_cl0 =  0 # p2526
        dCyp_dihedral = (3 * np.sin(np.radians(self.dihedral) * (1 - 2 * z / (self.b / 2)) * np.sin(np.radians(self.dihedral)))) * clp_cl0
        Cyp_wb = K*((Cyp__Cl * self.Cl)) + dCyp_dihedral #p2522

        # dCyB_VWBH = self.CyB()[1]
        CyB_VWBH__CyB_Veff = 0.95 #p.1669
        CyB_Veff = self.Cl_alpha_Vtail
        dCyB_VWBH = -CyB_VWBH__CyB_Veff * CyB_Veff * self.Sv / self.S
        Cyp_Vtail = 2 * ((z - zp) / self.b) * dCyB_VWBH
        # print(dCyB_VWBH)
        # print(z, zp)
        # print(Cyp_wb + Cyp_Vtail)
        return Cyp_wb + Cyp_Vtail, K
    
    def Clp(self):
        """
        Calculate the roll moment coefficient due to the roll rate. Method taken from FD lecture notes.

        Returns:
        float: Roll moment coefficient.
        """
        Clp = -0.3 + self.taper / 0.5 * ((0.3- 0.5))
        return Clp
        
    def Cnp(self):
        """
        Calculate the yaw moment coefficient due to the roll rate.

        Returns:
        float: Yaw moment coefficient.
        """
        x__c = self.x_bar / self.MAC # Distance from the leading edge of the wing to the center of gravity, normalized by the mean aerodynamic chord
        B = np.sqrt(1 - self.M**2 * np.cos(np.radians(self.Delta_c4))**2) # p.2522
        Cnp__Cl_M0 = (-(1 / 6) * (self.A + 6 * (self.A + np.cos(np.radians(self.Delta_c4))))*(x__c * np.tan(np.radians(self.Delta_c4)) / self.A  + (np.tan(np.radians(self.Delta_c4)))**2 / 12)) / (self.A + 4 * np.cos(np.radians(self.Delta_c4))) #p2559
        Cnp__Cl = ((self.A + 4 * np.cos(np.radians(self.Delta_c4))) / (self.A * B + 4 * np.cos(np.radians(self.Delta_c4)))) * ((self.A * B + 0.5 * (self.A * B + np.cos(np.radians(self.Delta_c4)) * (np.tan(np.radians(self.Delta_c4)))**2)) / (self.A + 0.5 * (self.A + np.cos(np.radians(self.Delta_c4)) * (np.tan(np.radians(self.Delta_c4)))**2))) * Cnp__Cl_M0 #p2559
        Cnp_w = -self.Clp() * np.tan(np.radians(self.alpha)) - self.Cyp()[1] * (-self.Clp() * np.tan(np.radians(self.alpha)) - Cnp__Cl * self.Cl) # p.2559 

        zp = self.zp # Distance from the centerline of the body to the centerline of the vertical tail
        z = zp * np.cos(np.radians(self.alpha)) - self.lp * np.sin(np.radians(self.alpha)) # distance from the centerline of the body to the centerline of the wing
        Cnp_tail = 2 / self.b * (self.lp * np.cos(np.radians(self.alpha)) + zp * np.sin(np.radians(self.alpha)) * (z - zp) / self.b * self.CyB()[1])
        Cnp_tail = 0 # Tail contribution is very small, so we can ignore it for now
        return Cnp_w - Cnp_tail, Cnp_w, Cnp_tail
    
    def Cyr(self):
        return 0 # subtracted by much larger mu factor, therefore ignored

    def Clr(self): #Spiral instability, not large enough to cause problems
        """
        Calculate the roll moment coefficient due to the yaw rate.

        Returns:
        float: Roll moment coefficient.
        """
        # p2580
        B = np.sqrt(1 - self.M**2 * (np.cos(np.radians(self.Delta_c4)))**2) # p.2522
        Clr__cl_M0 = 0.2 #p.2589
        Clr__Cl = (1 + (self.A*(1-B**2))/(2*B*(self.A*B + 2*np.cos(np.radians(self.Delta_c4)))) + (self.A*B+2*np.cos(np.radians(self.Delta_c4)))/(self.A*B+4*np.cos(np.radians(self.Delta_c4))) * np.tan(np.radians(self.Delta_c4))**2 / 8) / (1 + (self.A + 2 * np.cos(np.radians(self.Delta_c4))) / (self.A + 4 * np.cos(np.radians(self.Delta_c4))) * np.tan(np.radians(self.Delta_c4))**2 / 8) * Clr__cl_M0 #p.2581

        clB__cl_s = -0.002/5 # from plot->p1563
        Km_s = 1.03 #from plot p1564
        kf = 0.95 # from plot p1625
        clB__Cl_A = 0 #from plot
        ClB__Cl = clB__cl_s*Km_s*kf+clB__Cl_A
        dClr_Cl = self.Cl * ClB__Cl - self.ClB()
        dClr__dihedral = 1/12 * (np.pi * self.A * np.sin(np.radians(self.Delta_c4))) / (self.A + 4 * np.cos(np.radians(self.Delta_c4)))

        # Dihedral should be in radians for this formula
        Clr_wb = self.Cl * Clr__Cl + dClr_Cl + dClr__dihedral * np.radians(self.dihedral)

        zp = self.zp # Distance from the centerline of the body to the centerline of the vertical tail
        # print(f'Cl: {self.Cl}, Clr__Cl: {Clr__Cl},dClr_cl: {dClr_Cl}, dClr_dihedral: {dClr__dihedral}, dihedral: {self.dihedral}')
        Clr_tail = - 2 / self.b**2 * (self.lp * np.cos(np.radians(self.alpha)) + zp * np.sin(np.radians(self.alpha)))*(zp*np.cos(np.radians(self.alpha) - self.lp * np.sin(np.radians(self.alpha)))) * self.CyB()[1] #p.2802
        # print('Clr_wb:',Clr_wb, 'Clr_tail:', Clr_tail)
        # print('Clr:',Clr_wb+ Clr_tail)
        return Clr_wb + Clr_tail
    
    def Cnr(self):
        """
        Calculate the yaw moment coefficient due to the yaw rate.

        Returns:
        float: Yaw moment coefficient.
        """
        Cnr__Cl2 = -0.015
        Cnr__Cd0 = -0.3
        Cnr_wb = Cnr__Cl2 * self.Cl**2 + Cnr__Cd0 * self.Cd0

        zp = self.zp # Distance from the centerline of the body to the centerline of the vertical tail
        Cnr_tail = 2 / self.b**2 * (self.lp * np.cos(np.radians(self.alpha)) + zp * np.sin(np.radians(self.alpha)))**2 * self.CyB()[1]

        return Cnr_wb + Cnr_tail
    
    def CyBdot(self):
        """
        Calculate the side force coefficient due to the rate of change of the roll angle.
        
        Returns:
        float: Side force coefficient.
        """
        # p.2825
        zp = self.zp
        C_L_alpha_V = 0.124
        sigma_B_alpha = -0.001 # p.2834
        sigma_B_dihedral = np.rad2deg(-0.93) # p.2846
        sigma_B_WB = 0.135 # p.2867
    
        sigma_B = sigma_B_alpha * np.radians(self.alpha) + sigma_B_dihedral / 57.3 * np.radians(self.dihedral) + sigma_B_WB
        CyBdot = 2*C_L_alpha_V * sigma_B * self.Sv / self.S * (self.lp * np.cos(np.radians(self.alpha)) + zp * np.sin(np.radians(self.alpha)) / self.b) 
        return CyBdot
    
    def ClBdot(self):
        zp = self.zp
        ClB_dot = self.CyBdot() * (zp * np.cos(np.radians(self.alpha_f)) - self.lp * np.sin(np.radians(np.radians(self.alpha_f)))) / self.b
        return ClB_dot
    
    def CnBdot(self):
        zp = self.zp
        CnB_dot = -self.CyBdot() * (self.lp * np.cos(self.alpha_f) + zp * np.sin(self.alpha_f)) / self.b
        return CnB_dot
    
    def update_json(self):
        # Run the functions you want to store
        # Run the functions you want to store
        # Note, sometimes the [0] entry is used. THis is due to some function outputs being used in other functions. The main coefficient is always the first one
        # aero_stability_outputs = {
        #     'CyB': self.CyB()[0],  # Example usage with Cl = 0.5
        #     'ClB': self.ClB(),  # Example usage with Cl = 0.5 and alpha = 5 degrees
        #     'CnB': self.CnB()[0],  # Example usage
        #     'Cyp': self.Cyp()[0],  # Example usage with Cl = 0.5, alpha = 5 degrees, h_b = 0.05
        #     'Clp': self.Clp(),  # Example usage
        #     'Cnp': self.Cnp()[0],  # Example usage with Cl = 0.5, alpha = 5 degrees
        #     'Cyr': self.Cyr(),  # Example usage with Cl = 0.5, alpha = 5 degrees
        #     'Clr': self.Clr(),  # Example usage with Cl = 0.5, alpha = 5 degrees
        #     'Cnr': self.Cnr(),  # Example usage with Cl = 0.5, alpha = 5 degrees
        #     'CyBdot': self.CyBdot(),  # Example usage with Cl = 0.5, alpha = 5 degrees
        #     'ClBdot': self.ClBdot(),  # Example usage with Cl = 0.5, alpha = 5 degrees
        #     'CnBdot': self.CnBdot(),
        #     # Add more functions here if needed
        # }
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['CyB'] = self.CyB()[0]
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['ClB'] = self.ClB()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['CnB'] = self.CnB()[0]
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Cyp'] = self.Cyp()[0]
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Clp'] = self.Clp()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Cnp'] = self.Cnp()[0]
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Cyr'] = self.Cyr()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Clr'] = self.Clr()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Cnr'] = self.Cnr()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['CyBdot'] = self.CyBdot()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['ClBdot'] = self.ClBdot()
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['CnBdot'] = self.CnBdot()

        # lift_curve_slope = {
        #     'Cl_alpha': self.lift_curve.dcl_dalpha()[0]  # Lift curve slope of the wing, in deg
        # }

        # self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym'] = aero_stability_outputs
        # self.aircraft_data.data['outputs']['aerodynamics'] = lift_curve_slope
        self.aircraft_data.save_design('design3.json')

if __name__ == '__main__':
    aircraft_data = Data("design3.json")
    derivatives = DerivativesDatcom_asym(aircraft_data=aircraft_data)

    derivatives.update_json()

    