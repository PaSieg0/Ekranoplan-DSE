import numpy as np
import json
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, MissionType, ISA, AircraftType

class DerivativesDatcom_asym:
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.Delta_c4 = aircraft_data.data['outputs']['wing_design']['sweep_c_4'] #degrees
        self.l_b = aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] # length of body 
        self.dihedral = aircraft_data.data['outputs']['wing_design']['dihedral'] #degrees
        self.A = aircraft_data.data['outputs']['wing_design']['aspect_ratio'] 
        self.S = aircraft_data.data['outputs']['wing_design']['S'] 
        self.Av = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['aspect_ratio'] # Aspect ratio of the vertical tail
        self.Ah = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['aspect_ratio']
        self.Sh = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] # Surface area of the horizontal tail: 100
        self.Sv = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S'] # Surface area of the vertical tail: 75
        self.d_fusel = np.array([aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station1'], aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station2'], aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station3']]) # Diameter of the fuselage
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
        self.alpha = 5.0 # Angle of attack in degrees, TODO: UPDATE!!
        self.Cl = 0.5   # Lift coefficient at the angle of attack, TODO: UPDATE!!


    def CyB(self):
        """
        Calculate the side force coefficient due to the elevator deflection.

        Parameters:
        CL (float): Lift coefficient.

        Returns:
        tuple(float: total Side force coefficient, side force due to tail(needed for Cyp).
        """
        z_w = 2 #TODO #distance from centerline wing to centerline body
        d = 6 #max body height at wing intersection
        K = -2*1.85*(z_w / d)# from plot

        delta_CyB_dihedral = -0.0001* self.dihedral
        k2__k1 = 0.9 # determined from plot page 834 DATCOM
        S0 = 30 # Cross sectional area of the body at the wing intersection [m2]
        Cl_alpha_body = 2*(k2__k1)* S0 / (self.V_b ** (2/3)) # body lift curve slope [1/rad]

        CyB_body = -Cl_alpha_body
        S_ref_body = self.l_b * self.d_fusel
        S_ref_wing = self.S

        A_vbA_v = 1.2 #b_v / 2 * r, if we know empennage sizing->look at page 1667
        Kh = 0.8 #depends on Sh/Sv-> page 1668 DATCOM
        Av_hb__Av_b = 1.7 #Depends on zh/bh, thus for a t-tail equal to 1 ->p1667
        A_eff_Vstab = A_vbA_v * self.Av * (1+Kh*(Av_hb__Av_b - 1))
        Cl_alpha_Vtail = 0.16 # WIG, NACA 0012
        sidewash_coeff = 0.724 + 3.06 * (self.Sv / self.S) / (1 + np.cos(np.radians(self.Delta_c4))) +0.4*z_w / self.d_fusel + 0.009*self.A


        # CyB_wing  = Cl**2 (6 * np.tan(self.Delta_c4) / (np.pi * self.A * (self.A + 4 * np.cos(self.Delta_c4)))) * 1/ 57.3

        CyB_wing_body = K * CyB_body * (S_ref_body / S_ref_wing) * delta_CyB_dihedral # TODO: Check formula for angle unit
        CyB_tail = - 1 * Cl_alpha_Vtail * sidewash_coeff *self.Sv / self.S

        return CyB_wing_body + CyB_tail, CyB_tail
    
    def ClB(self):
        clB__cl_s = -0.002/5 # from plot->p1563
        Km_s = 1.03 #from plot p1564
        kf = 0.95 # from plot p1625
        clB__Cl_A = 0 #from plot
        clB__dihedral = -0.00024 #p1565
        Km_d = 1 #p1566
        dclB__dihedral = -0.0005 * np.sqrt(self.A)*(self.d_fusel / self.b)**2
        dclB_zw = 1.2 * np.sqrt(self.A) / 57.3 * (2/self.b) * (2 * self.d_fusel / self.b)

        #TODO: check if dihedral should indeed be in degrees for this formula
        ClB_wb = Cl *(clB__cl_s*Km_s*kf+clB__Cl_A) + self.dihedral * (clB__dihedral * Km_d + dclB__dihedral) + dclB_zw + 0

        K = 1.4 #p.1600 smthng
        cl_alpha_V_tail = 0.16 #p.1600 smthng
        dcyB_Vtail = -K * cl_alpha_V_tail * self.Sv / self.S
        zp = 7 #Guessed
        dclB_Vtail = dcyB_Vtail * (zp * np.cos(np.radians(self.alpha)) - self.lp*np.sin((self.alpha))) / self.b

        return ClB_wb + dclB_Vtail
    
    def CnB(self):
        K_N = 0.0018 #p.1633
        K_R_l = 1.65 #p.1634
        S_B_s = self.d_fusel * self.l_b # Cross sectional area of the body at the wing intersection [m2]
        CnB_wb = -K_N * K_R_l * S_B_s / self.S * self.l_b / self.b

        K = 1.4 #p.1600 smthng
        cl_alpha_V_tail = 0.16 #p.1600 smthng
        dcyB_Vtail = -K * cl_alpha_V_tail * self.Sv / self.S
        CnB_Vtail = -dcyB_Vtail * (self.lp / self.b)
        return CnB_wb + CnB_Vtail, CnB_wb, CnB_Vtail
    
    def Cyp(self, h_b):
        """
        Calculate the side force coefficient due to the roll rate.

        Returns:
        float: Side force coefficient.
        """
        sigma=np.exp(-2.48*(h_b)**(0.768))
        zp = 7 # Guessed, distance from the centerline of the body to the centerline of the vertical tail
        z = zp * np.cos(np.radians(self.alpha)) - self.lp * np.sin(np.radians(self.alpha)) # distance from the centerline of the body to the centerline of the wing
        K =  (self.Cl_alpha * np.tan(np.radians(self.alpha)) + self.Cl / (np.cos(np.radians(self.alpha)))**2 - 2 * self.Cl / (np.pi * self.A * self.e) * (1 - sigma) * self.Cl_alpha) / (self.Cl_alpha * np.tan(np.radians(self.alpha)) + self.Cl / (np.cos(np.radians(self.alpha)))**2 - 2 * self.Cl / (np.pi * self.A * self.e) * self.Cl_alpha)
        B = np.sqrt(1 - 0.34**2 * np.cos(np.radians(self.Delta_c4))**2) # p.2522
        Cyp__Cl_0 = -0.18 #p2529
        Cyp__Cl = ((self.A + 4 * np.cos(np.radians(self.Delta_c4))) * (self.A * B + np.cos(np.radians(self.Delta_c4))) * Cyp__Cl_0) / (self.A * B + 4 * np.cos(np.radians(self.Delta_c4)) * (self.A + np.cos(np.radians(self.Delta_c4))))

        clp_cl0 =  0 # p2526
        dCyp_dihedral = (3 * np.sin(np.radians(self.dihedral) * (1 - 2 * z / (self.b / 2)) * np.sin(np.radians(self.dihedral)))) * clp_cl0
        Cyp_wb = K*((Cyp__Cl * self.Cl)) + dCyp_dihedral

        dCyB_VWBH = self.CyB(self.Cl)[1]
        Cyp_Vtail = Cyp_wb + 2 * ((z - zp) / self.b) * dCyB_VWBH
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
        print(x__c)
        print(f"x__c: {x__c}")
        B = np.sqrt(1 - 0.34**2 * np.cos(np.radians(self.Delta_c4))**2) # p.2522
        Cnp__Cl_M0 = (-(1 / 6) * (self.A + 6 * (self.A + np.cos(np.radians(self.Delta_c4))))*(x__c * np.tan(np.radians(self.Delta_c4)) / self.A  + (np.tan(np.radians(self.Delta_c4)))**2 / 12)) / (self.A + 4 * np.cos(np.radians(self.Delta_c4)))
        Cnp__Cl = ((self.A + 4 * np.cos(np.radians(self.Delta_c4))) / (self.A * B + 4 * np.cos(np.radians(self.Delta_c4)))) * ((self.A * B + 0.5 * (self.A * B + np.cos(np.radians(self.Delta_c4)) * (np.tan(np.radians(self.Delta_c4)))**2)) / (self.A + 0.5 * (self.A + np.cos(np.radians(self.Delta_c4)) * (np.tan(np.radians(self.Delta_c4)))**2))) * Cnp__Cl_M0
        Cnp_w = -self.Clp() * np.tan(np.radians(self.alpha)) - self.Cyp(self.Cl, self.alpha, h_b = 0.05)[1] * (-self.Clp() * np.tan(np.radians(self.alpha)) - Cnp__Cl * self.Cl) # p.2559 

        zp = 7 # Guessed, distance from the centerline of the body to the centerline of the vertical tail
        z = zp * np.cos(np.radians(self.alpha)) - self.lp * np.sin(np.radians(self.alpha)) # distance from the centerline of the body to the centerline of the wing
        Cnp_tail = 2 / self.b * (self.lp * np.cos(np.radians(self.alpha)) + zp * np.sin(np.radians(self.alpha)) * (z - zp) / self.b * self.CyB(self.Cl)[1])
        return Cnp_w - Cnp_tail, Cnp_w, Cnp_tail
    
    def Cyr(self):
        print('Cyr is very very vry very very very very small, Sam said so, thereby it is true')
        return 0

    def Clr(self):
        """
        Calculate the roll moment coefficient due to the yaw rate.

        Returns:
        float: Roll moment coefficient.
        """
        # p2580
        B = np.sqrt(1 - 0.34**2 * np.cos(np.radians(self.Delta_c4))**2) # p.2522
        Clr__cl_M0 = .24 #p.2589
        Clr__Cl = (1 + (self.A*(1-B**2))/(2*B*(self.A*B + 2*np.cos(np.radians(self.Delta_c4)))) + (self.A*B+2*np.cos(np.radians(self.Delta_c4)))/(self.A*B+4*np.cos(np.radians(self.Delta_c4))) * np.tan(np.radians(self.Delta_c4))**2 / 8 * Clr__cl_M0) / (1 + (self.A + 2 * np.cos(np.radians(self.Delta_c4))) / (self.A + 4 * np.cos(np.radians(self.Delta_c4))) * np.tan(np.radians(self.Delta_c4))**2 / 8) #p.2581

        clB__cl_s = -0.002/5 # from plot->p1563
        Km_s = 1.03 #from plot p1564
        kf = 0.95 # from plot p1625
        clB__Cl_A = 0 #from plot
        ClB__Cl = clB__cl_s*Km_s*kf+clB__Cl_A
        dClr_Cl = self.Cl * ClB__Cl - self.ClB(self.Cl, self.alpha)
        dClr__dihedral = 1/12 * (np.pi * self.A * np.sin(np.radians(self.Delta_c4))) / (self.A + 4 * np.cos(np.radians(self.Delta_c4)))

        #TODO: Check if dihedral should indeed be in degrees for this formula
        Clr_wb = self.Cl * Clr__Cl + dClr_Cl + dClr__dihedral * self.dihedral

        zp = 7 # Guessed, distance from the centerline of the body to the centerline of the vertical tail
        Clr_tail = 2 / self.b**2 * (self.lp * np.cos(np.radians(self.alpha)) + zp * np.sin(np.radians(self.alpha))*(zp*np.cos(np.radians(self.alpha) - self.lp * np.sin(np.radians(self.alpha)))) * self.CyB(self.Cl)[1]) #p.2802
        return Clr_wb - Clr_tail
    
    def Cnr(self, Cl, alpha):
        """
        Calculate the yaw moment coefficient due to the yaw rate.

        Returns:
        float: Yaw moment coefficient.
        """
        Cnr__Cl2 = -.015
        Cnr__Cd0 = -.3
        Cnr_wb = Cnr__Cl2 * Cl**2 + Cnr__Cd0 * self.Cd0

        zp = 7 # Guessed, distance from the centerline of the body to the centerline of the vertical tail
        Cnr_tail = 2 / self.b**2 * (self.lp * np.cos(alpha) + zp * np.sin(alpha))**2 * self.CyB(Cl)[1]

        return Cnr_wb + Cnr_tail
    
    def CyBdot(self, Cl, alpha):
        """
        Calculate the side force coefficient due to the rate of change of the roll angle.
        !!!!!!THIS IS THE MOST SKETCHY ONE, IT SHOULD BE VERY SMALL BUT IT IS NOT. HOWEVER CLBDOT DEPENDS ON THIS AND LOOKS FINE!!!!!

        Returns:
        float: Side force coefficient.
        """
        # p.2825
        zp = 7
        C_L_alpha_V = 0.124
        sigma_B_alpha = -.001 # p.2834
        sigma_B_dihedral = np.rad2deg(-0.93) # p.2846
        sigma_B_WB = 0.135 # p.2867
        sigma_B = sigma_B_alpha * alpha + sigma_B_dihedral / 57.3 * self.dihedral + sigma_B_WB
        CyBdot = 2*C_L_alpha_V * sigma_B * self.Sv / self.S * (self.lp * np.cos(alpha) + zp * np.sin(alpha) / self.b) 
        return CyBdot
    
    def ClBdot(self, Cl, alpha_f):
        zp = 7
        ClB_dot = self.CyBdot(Cl,alpha_f) * (zp * np.cos(alpha_f) - self.lp * np.sin(alpha_f)) / self.b
        return ClB_dot




# cyb = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, 1,60, 6, 2000, 63.79, 36.431, 0.16, 0.85)
# print(cyb.CnB())
# Cyp = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, 1,60, 6, 2000, 63.79, 36.431, 0.16, 0.85)
# print(Cyp.Cyp(0.5, 0.1, 0.05))
# Clp = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, 1,60, 6, 2000, 63.79, 36.431, 0.16, 0.85, 0.4)
# print(Clp.Clp())
# Cnp = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, 1,60, 6, 2000, 63.79, 36.431, 0.16, 0.85, 0.4)
# print(Cnp.Cnp(1, np.deg2rad(2)))
# Cnr = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, 1,60, 6, 2000, 63.79, 36.431, 0.16, 0.85, 0.4)
# print(Cnr.Cnr(1, np.deg2rad(2)))
# Clr = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, np.deg2rad(1) ,60, 6, 2000, 63.79, 36.431, 0.16, 0.85, 0.4)
# print(Clr.Clr(1, np.deg2rad(2)))
derivatives = DerivativesDatcom_asym(0, 8, 507, 1.5, 100, 75, np.deg2rad(1), 60, 6, 2000, 63.79, 36.431, 0.16, 0.85, 0.4)
# print(CyBdot.CyB(1))



# Run the functions you want to store
# Note, sometimes the [0] entry is used. THis is due to some function outputs being used in other functions. The main coefficient is always the first one
aero_stability_outputs = {
    'CyB': derivatives.CyB(0.5)[0],  # Example usage with Cl = 0.5
    'ClB': derivatives.ClB(0.5, np.deg2rad(5)),  # Example usage with Cl = 0.5 and alpha = 5 degrees
    'CnB': derivatives.CnB()[0],  # Example usage
    'Cyp': derivatives.Cyp(0.5, np.deg2rad(5), 0.05)[0],  # Example usage with Cl = 0.5, alpha = 5 degrees, h_b = 0.05
    'Clp': derivatives.Clp(),  # Example usage
    'Cnp': derivatives.Cnp(0.5, np.deg2rad(5))[0],  # Example usage with Cl = 0.5, alpha = 5 degrees
    'Cyr': derivatives.Cyr(),  # Example usage with Cl = 0.5, alpha = 5 degrees
    'Clr': derivatives.Clr(0.5, np.deg2rad(5)),  # Example usage with Cl = 0.5, alpha = 5 degrees
    'Cnr': derivatives.Cnr(0.5, np.deg2rad(5)),  # Example usage with Cl = 0.5, alpha = 5 degrees
    'CyBdot': derivatives.CyBdot(0.5, np.deg2rad(5)),  # Example usage with Cl = 0.5, alpha = 5 degrees
    'ClBdot': derivatives.ClBdot(0.5, np.deg2rad(5)),  # Example usage with Cl = 0.5, alpha = 5 degrees

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
if 'aerodynamic_stability_coefficients_asym' not in data['outputs']:
    data['outputs']['aerodynamic_stability_coefficients_asym'] = {}

# Update values
data['outputs']['aerodynamic_stability_coefficients_asym'].update(aero_stability_outputs)

# Save back to file
with open(json_path, 'w') as file:
    json.dump(data, file, indent=4)
    
if __name__ == 'main':
    aircraft_data = Data("design3.json")
    