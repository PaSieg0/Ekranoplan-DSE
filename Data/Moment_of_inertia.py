import os
import sys
import numpy as np
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)
from utils import Data  


class MomentOfInertia:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.fuselage_weight = self.aircraft_data.data['outputs']['component_weights']['fuselage'] / 9.81
        self.wing_weight = self.aircraft_data.data['outputs']['component_weights']['wing'] / 9.81
        self.engine_weight = self.aircraft_data.data['outputs']['component_weights']['engine'] / 9.81
        self.cargo_weight = 90000

        self.fuselage_diameter = self.aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station1']
        self.engine_diameter = self.aircraft_data.data['inputs']['engine']['prop_diameter']
        self.engine1_y = self.aircraft_data.data['outputs']['engine_positions']['y_engines'][0]
        self.engine2_y = self.aircraft_data.data['outputs']['engine_positions']['y_engines'][1]
        self.engine3_y = self.aircraft_data.data['outputs']['engine_positions']['y_engines'][2]
        self.engine1_z = self.aircraft_data.data['outputs']['engine_positions']['z_engines'][0]
        self.engine2_z = self.aircraft_data.data['outputs']['engine_positions']['z_engines'][1]
        self.engine3_z = self.aircraft_data.data['outputs']['engine_positions']['z_engines'][2]
        self.b_wing = self.aircraft_data.data['outputs']['design']['b']
        self.h_b_wing = self.aircraft_data.data['outputs']['design']['h_b']

    def calculate_radii(self):
        self.r_fuselage = self.fuselage_diameter / 2
        self.r_engine = self.engine_diameter / 2
    
    def calculate_distances(self):
        self.engine1_d = np.sqrt(self.engine1_y**2 + self.engine1_z**2)
        self.engine2_d = np.sqrt(self.engine2_y**2 + self.engine2_z**2)
        self.engine3_d = np.sqrt(self.engine3_y**2 + self.engine3_z**2)
        self.wing_d = self.h_b_wing * self.b_wing

    def calculate_Ixx(self):
        self.calculate_radii()
        self.calculate_distances()
        self.Ixx_fuselage = 0.5 * (self.fuselage_weight+self.cargo_weight) * (self.r_fuselage**2)
        self.Ixx_wing = self.wing_weight * self.wing_d**2
        self.Ixx_engine1 = (0.5 * self.engine_weight * self.r_engine**2) + (self.engine_weight * self.engine1_d**2)
        self.Ixx_engine2 = (0.5 * self.engine_weight * self.r_engine**2) + (self.engine_weight * self.engine2_d**2)
        self.Ixx_engine3 = (0.5 * self.engine_weight * self.r_engine**2) + (self.engine_weight * self.engine3_d**2)
        self.Ixx_total = (self.Ixx_fuselage +
                          self.Ixx_wing + 
                          self.Ixx_engine1 + 
                          self.Ixx_engine2 + 
                          self.Ixx_engine3)
        return (self.Ixx_total)

if __name__ == "__main__":
    aircraft_data = Data("design3.json")
    moment_of_inertia = MomentOfInertia(aircraft_data)
    Ixx = moment_of_inertia.calculate_Ixx()
    print(Ixx)    

    #QIHAO the goat
    


    



