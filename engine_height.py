import numpy as np 
from utils import Data

class EngineHeight:
    def __init__(self, data: Data):
        self.data = data
        self.diameter_engine = 5.5  # m
        self.clearance_engine_fuselage = 2  # m
        self.clearance_enginetips = 0.1  # m
        self.sea_water_density = 1020  # kg/m^3
        
        # Extract required data from input
        self.wing_type = self.data.data['inputs']['wing_type']
        self.d_fuselage = self.data.data['outputs']['general']['d_fuselage']
        self.l_fuselage = self.data.data['outputs']['general']['l_fuselage']
        self.dihedral = self.data.data['outputs']['wing_design']['dihedral']
        self.n_engines = self.data.data['inputs']['n_engines']
        self.MTOM = self.data.data['outputs']['design']['MTOM']
        self.n_fuselage = self.data.data['inputs']['n_fuselages']
        if 'fuselage_separation' in self.data.data['outputs']['general']:
            self.fuselage_separation = self.data.data['outputs']['general']['fuselage_separation']
        

    def calculate_floating_depth(self):    
        return self.MTOM / (self.sea_water_density * self.l_fuselage * self.d_fuselage * self.n_fuselage)

    def calculate_engine_positions(self):
        if self.n_fuselage == 1:
            floating_depth = self.calculate_floating_depth()
            # Calculate y positions
            y_engines = np.zeros(int(self.n_engines/2))
            for j in range(int(self.n_engines/2)):
                if j == 0:
                    y_engines[j] = self.d_fuselage/2 + (self.diameter_engine/2)
                else:
                    y_engines[j] = y_engines[j-1] + (self.diameter_engine + self.clearance_enginetips)
        else:
            floating_depth = self.calculate_floating_depth()
            if self.fuselage_separation > self.diameter_engine*2+self.clearance_engine_fuselage*2 + self.clearance_enginetips:
                y_engines = np.zeros(int(self.n_engines/2))
                
        # Calculate z positions and clearances
        z_engines = np.zeros_like(y_engines)
        if self.wing_type == "HIGH":
            z_engines = self.d_fuselage + np.tan(np.deg2rad(self.dihedral))*y_engines
        elif self.wing_type == "LOW":
            z_engines = np.tan(np.deg2rad(self.dihedral))*y_engines
        
        wing_tip_clearance = z_engines - floating_depth
        
        return y_engines, z_engines, wing_tip_clearance

def main():
    for i in range(1, 5):
        json_file = f"design{i}.json"
        aircraft_data = Data(json_file)
        engine_height = EngineHeight(aircraft_data)
        y_engines, _, wing_tip_clearance = engine_height.calculate_engine_positions()
        
        print(f"Engine clearance distance for design {i} is {wing_tip_clearance} m")

if __name__ == "__main__":
    main()



