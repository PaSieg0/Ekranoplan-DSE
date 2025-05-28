import numpy as np

class DerivativesDatcom:
    def __init__(self, Delta_c4, A, S, Av, Sh, Sv, dihedral, l_b, d_fusel, V_b, b, lp):
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
        self.lp = lp

    def CyB(self, Cl, z_w=2, d=6, k2__k1=0.9, S0=30):
        """
        Calculate the side force coefficient due to the elevator deflection.

        Parameters:
        CL (float): Lift coefficient.
        Delta_c4 (float): Elevator deflection in radians.
        A (float): Aspect ratio of the wing.

        Returns:
        float: Side force coefficient.
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
        sidewash_coeff = 0.724 + 3.06 * (self.Sv / self.S) / (1 + np.cos(self.Delta_c4)) +0.4*z_w / self.d_fusel + 0.009*self.A


        # CyB_wing  = Cl**2 (6 * np.tan(self.Delta_c4) / (np.pi * self.A * (self.A + 4 * np.cos(self.Delta_c4)))) * 1/ 57.3
        CyB_wing_body = K * CyB_body * (S_ref_body / S_ref_wing) * delta_CyB_dihedral
        CyB_tail = - 1 * Cl_alpha_Vtail * sidewash_coeff *self.Sv / self.S

        return CyB_wing_body + CyB_tail
    
    def ClB(self, Cl, alpha):
        clB__cl_s = -0.002/5 # from plot->p1563
        Km_s = 1.03 #from plot p1564
        kf = 0.95 # from plot p1625
        clB__Cl_A = 0 #from plot
        clB__dihedral = -.00024 #p1565
        Km_d = 1 #p1566
        dclB__dihedral = -0.0005 * np.sqrt(self.A)*(self.d_fusel / self.b)**2
        dclB_zw = 1.2 * np.sqrt(self.A) / 57.3 * (2/self.b) * (2 * self.d_fusel / self.b)

        ClB_wb = Cl *(clB__cl_s*Km_s*kf+clB__Cl_A) + self.dihedral * (clB__dihedral * Km_d + dclB__dihedral) + dclB_zw + 0

        K = 1.4 #p.1600 smthng
        cl_alpha_V_tail = 0.16 #p.1600 smthng
        dcyB_Vtail = -K * cl_alpha_V_tail * self.Sv / self.S
        zp = 7 #Guessed
        dclB_Vtail = dcyB_Vtail * (zp * np.cos(alpha) - self.lp*np.sin(alpha)) / self.b

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
        return CnB_wb + CnB_Vtail
    
    def Cyp(self):
        """
        Calculate the side force coefficient due to the roll rate.

        Returns:
        float: Side force coefficient.
        """
    
    
    
        
        
    
cyb = DerivativesDatcom(0, 8, 507, 1.5, 100, 75, 1,60, 6, 2000, 63.79, 66)
print(cyb.CnB())


    
