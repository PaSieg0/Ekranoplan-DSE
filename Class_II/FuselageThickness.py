import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from scipy.integrate import quad

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials, EvaluateType

class FuselageThickness:
    def __init__(self, aircraft_data: Data, airfoil_data: Data, fuselage_mat: Materials):
          
        self.data = airfoil_data.data
        self.aircraft_data = aircraft_data

        self.front_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['rear_spar']

        self.t_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['t_spar']/1000
        self.t_skin = self.aircraft_data.data['inputs']['structures']['wing_box']['t_skin']/1000
        self.t_wing = self.aircraft_data.data['inputs']['structures']['wing_box']['t_wing']/1000

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.b_array = np.arange(0, self.b/2, 0.01)
        # self.chord_array = self.chord_span_function(self.b_array)

        self.chord_length = self.chord_root

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']

        self.n_stringers = self.aircraft_data.data['inputs']['structures']['wing_box']['n_stringers']

        self.fuel_tank = self.aircraft_data.data['inputs']['fuel_tank']

        self.material = self.aircraft_data.data['inputs']['structures']['materials'][fuselage_mat.name.lower()]
        self.G = self.material['G']
        self.E = self.material['E']
        self.sigma_y = self.material['sigma_y']
        self.poisson_ratio = self.material['poisson_ratio']

        self.rho_fuselage = self.material['rho']

        self.fuselage_width = np.array([5.7, 5.7, 5.7])
        self.fuselage_height = np.array([4.5, 6, 4.5])
        self.fuselage_ratio =  self.fuselage_height / self.fuselage_width


    def calculate_fuselage_centroid(self):

        self.a_dim = 0.5 / np.tan(np.radians(65))
        self.s_dim = 0.5 / np.sin(np.radians(65))
        self.o_dim = self.s_dim/2 * np.sin(np.radians(25))

        #Unit is meters
        self.y_bar_fuselage = (2*(self.fuselage_ratio*((self.a_dim + 0.4)*self.fuselage_width)) + 2*(self.s_dim * (self.o_dim * self.fuselage_width)) + ((self.a_dim + self.fuselage_ratio) * self.fuselage_width)) / (2*self.s_dim + 2*self.fuselage_ratio + 1)
        
        self.x_bar_fuselage = (self.s_dim * 0.25 * self.fuselage_width + (self.s_dim * 0.75 * self.fuselage_width) + (0.5*self.fuselage_width) + (self.fuselage_ratio*self.fuselage_width)) / (2*self.s_dim + 2*self.fuselage_ratio + 1)

        return self.x_bar_fuselage, self.y_bar_fuselage
        
    
    def calculate_fuselage_moi(self):
        # Thin walled assumption used for fuselage
        # All moments of inertia are calculated about the centroid of the fuselage
        # All moments of inertia still have to be multiplied by the thickness of the skin
        # Unit is m^4 --> means that t_skin must also be in METERS!

        self.I_xy_fuselage = 0 # * t_skin

        self.I_xx_fuselage = self.fuselage_width*(((self.fuselage_ratio + self.a_dim)*self.fuselage_width - self.y_bar_fuselage)**2) + 2*(((self.fuselage_ratio*self.fuselage_width)**3)/12 + self.fuselage_ratio*self.fuselage_width*((self.a_dim + self.fuselage_ratio/2)*self.fuselage_width - self.y_bar_fuselage)**2) + 2*(((self.s_dim**3 * (np.sin(np.radians(25)))**2)/12) + self.s_dim*(self.o_dim -self.y_bar_fuselage)**2) # * t_skin

        self.I_yy_fuselage = (self.fuselage_width**3) / 12 + 2*(self.fuselage_ratio*self.fuselage_width*(self.fuselage_width/2)**2) + 2*(self.s_dim*(0.25*self.fuselage_width)**2)

        return self.I_xx_fuselage, self.I_yy_fuselage, self.I_xy_fuselage
        
    
    def calculate_stresses(self):

        self.sigma_1 = (((M_x * self.I_yy_fuselage)*((self.a_dim+(self.fuselage_ratio*self.fuselage_width)-self.y_bar_fuselage))) + (M_y*self.I_xx_fuselage)*(-0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)
        self.sigma_2 = (((M_x * self.I_yy_fuselage)*((self.a_dim+(self.fuselage_ratio*self.fuselage_width)-self.y_bar_fuselage))) + (M_y*self.I_xx_fuselage)*(0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)

        self.sigma_3 = ((M_x * self.I_yy_fuselage) * -(self.y_bar_fuselage-self.a_dim) + (M_y *self.I_xx_fuselage)*(-0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)
        self.sigma_4 = ((M_x * self.I_yy_fuselage) * -(self.y_bar_fuselage-self.a_dim) + (M_y *self.I_xx_fuselage)*(0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)

        self.sigma_5 = ((M_x * self.I_yy_fuselage)*(-self.y_bar_fuselage)) / (self.I_xx_fuselage*self.I_yy_fuselage)
        return self.sigma_1, self.sigma_2, self.sigma_3, self.sigma_4, self.sigma_5

    def calculate_boom_areas(self):
        for i in range(5):
            # All boom areas are a function of the skin thickness
            if i == 0:
                B1 = (self.fuselage_width / 6) * (2 + (self.sigma_2 / self.sigma_1)) + ((0.8*self.fuselage_width / 6) * (2+ (sigma_3/sigma_1)))
            elif i == 1:
                B2 = (self.fuselage_width / 6) * (2 + (self.sigma_1 / self.sigma_2)) + ((0.8*self.fuselage_width / 6) * (2+ (sigma_4/sigma_2)))
            elif i == 2:
                B3 = (0.8*self.fuselage_width / 6) * (2 + (self.sigma_1 / self.sigma_3)) + ((0.5517*self.fuselage_width / 6) * (2+ (sigma_5/sigma_3)))
            elif i == 3:
                B4 = (0.8*self.fuselage_width / 6) * (2 + (self.sigma_2 / self.sigma_4)) + ((0.5517*self.fuselage_width / 6) * (2+ (sigma_5/sigma_4)))
            elif i == 4:
                B5 = (0.5517*self.fuselage_width / 6) * (2 + (self.sigma_3 / self.sigma_5)) + ((0.5517*self.fuselage_width / 6) * (2+ (sigma_4/sigma_5)))
        return B1, B2, B3, B4, B5
            
        


if __name__ == '__main__':
    aircraft_data = Data("design3.json")
    airfoil_data = Data("AirfoilData/example_airfoil.json")
    fuselage_material = Materials.Al5052  
    
    fuselage = FuselageThickness(aircraft_data, airfoil_data, fuselage_material)
    
    x_bar, y_bar = fuselage.calculate_fuselage_centroid()
    Ixx, Iyy, Ixy = fuselage.calculate_fuselage_moi()
    print(x_bar, y_bar)
    print(Ixx, Iyy, Ixy)