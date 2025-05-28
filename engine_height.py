import numpy as np 
from utils import Data

class EngineHeight:
    def __init__(self, data: Data):
        self.data = data
        self.design_number = self.data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.diameter_engine = self.data.data['inputs']['engine']['prop_diameter']  # m
        self.clearance_engine_fuselage = 2  # m
        self.clearance_enginetips = 0.1  # m
        self.sea_water_density = self.data.data['rho_water']  # kg/m^3
        
        # Extract required data from input
        self.wing_type = self.data.data['inputs']['wing_type']
        self.d_fuselage = self.data.data['outputs']['general']['d_fuselage']
        self.l_fuselage = self.data.data['outputs']['general']['l_fuselage']
        self.dihedral = self.data.data['outputs']['wing_design']['dihedral']
        self.n_engines = self.data.data['inputs']['engine']['n_engines']
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
            self.y_engines = np.zeros(int(self.n_engines/2))
            for j in range(int(self.n_engines/2)):
                if j == 0:
                    self.y_engines[j] = self.d_fuselage/2 + (self.diameter_engine/2)
                else:
                    self.y_engines[j] = self.y_engines[j-1] + (self.diameter_engine + self.clearance_enginetips)
        else:
            floating_depth = self.calculate_floating_depth()
            if self.fuselage_separation > self.diameter_engine*2+self.clearance_engine_fuselage*2 + self.clearance_enginetips:
                self.y_engines = np.zeros(int(self.n_engines/2))
                
        # Calculate z positions and clearances
        self.z_engines = np.zeros_like(self.y_engines)
        if self.wing_type == "HIGH":
            self.z_engines = self.d_fuselage + np.tan(np.deg2rad(self.dihedral))*self.y_engines
        elif self.wing_type == "LOW":
            self.z_engines = np.tan(np.deg2rad(self.dihedral))*self.y_engines
        
        self.wing_tip_clearance = self.z_engines - floating_depth

        self.update_attributes()
        self.data.save_design(design_file=self.design_file)
        print(self.y_engines)


    def update_attributes(self):
        self.data.data['outputs']['engine_positions'] = {
            'y_engines': self.y_engines.tolist(),
            'z_engines': self.z_engines.tolist(),
            'wing_tip_clearance': self.wing_tip_clearance.tolist()
        }

def main():

    json_file = "design3.json"
    aircraft_data = Data(json_file)
    engine_height = EngineHeight(aircraft_data)
    engine_height.calculate_engine_positions()
    
    # print(f"\nEngine y-coordinates (spanwise position) for design 3:")
    # print(f"Left side engines: {-y_engines[::-1]} m")
    # print(f"Right side engines: {y_engines} m\n")
    # print(f"Engine clearance distance for design 3 is {wing_tip_clearance} m")


if __name__ == "__main__":
    main()



