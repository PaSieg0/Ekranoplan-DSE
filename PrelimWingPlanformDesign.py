import numpy as np
import json



class WingPlanform:
    def __init__(self, 
                 wing_area: float,
                 aspect_ratio: float) -> None:

        
        self.fuselage_length = 53
        self.x_c = 0
        self.taper_ratio = 0.4
        self.x_c_OEW_cg = 0.2
        self.x_c_wing_cg = 0.4
        self.fuse_x_cg = 0.4 #lf
        self.mass_fraction_wing = 0.08 #OEW
        self.mass_fraction_fuse = 0.08
        self.sweep_c_4 = 0


        if wing_type == wing_type.highwing:
            self.dihedral = 1

        elif wing_type == wing_type.lowwing:
            self.dihedral = 5



        self.wing_area = wing_area
        self.aspect_ratio = aspect_ratio

    def x_LE(self):
        b = np.sqrt(self.aspect_ratio * self.wing_area)
        sweep_x_c = np.rad2deg(np.arctan(np.tan(self.sweep_c_4)-((4/self.aspect_ratio)*(self.x_c-0.25)*((1-self.taper_ratio)/(1+self.taper_ratio)))))
        chord_root = 2 * self.wing_area / (b * (1 + self.taper_ratio))
        chord_tip = self.taper_ratio * chord_root
        MAC = (1+self.taper_ratio+self.taper_ratio**2)/((1+self.taper_ratio)*(2/3)*chord_root)
        y_MAC = (b/6)*(1+2*self.taper_ratio)/(1+self.taper_ratio)
        X_LEMAC = (self.fuse_x_cg*self.fuselage_length) + MAC*(self.x_c_wing_cg*self.mass_fraction_wing/self.mass_fraction_fuse-self.x_c_OEW_cg*(1+self.mass_fraction_wing/self.mass_fraction_fuse))
        X_LE = X_LEMAC - y_MAC*np.tan(np.deg2rad(self.dihedral))
        return X_LE






