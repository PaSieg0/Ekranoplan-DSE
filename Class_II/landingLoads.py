import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

class LandingLoads:
    def __init__(self, data: Data):
        self.data = data
        self.center_of_pressure_from_AC = 0.25 # 25% of the MAC from the leading edge
        self.x_LEMAC = self.data.data["outputs"]["wing_design"]["X_LEMAC"]
        self.x_aft_cg = self.data.data["outputs"]["cg_range"]["most_aft_cg"]
        self.x_fore_cg = self.data.data["outputs"]["cg_range"]["most_forward_cg"]
        self.wing_MAC = self.data.data["outputs"]["wing_design"]["MAC"]

        self.x_center_of_pressure = self.x_LEMAC + self.center_of_pressure_from_AC * self.wing_MAC
        
    def calculate_radius_of_gyration(self):
        """Calculate the radius of gyration about the CG (midpoint of CG range)"""
        # 1. Get CG range from data
        x_cg_fore = self.x_fore_cg
        x_cg_aft = self.x_aft_cg
        # 2. Midpoint CG
        x_cg = 0.5 * (x_cg_fore + x_cg_aft)

        # 3. Get component masses and positions
        weights = self.data.data['outputs']['component_weights']
        positions = self.data.data['outputs']['component_positions']
        # Exclude total_OEW if present
        component_masses = {k: v for k, v in weights.items() if k != 'total_OEW'}

        I = 0.0
        total_mass = 0.0
        for comp, mass in component_masses.items():
            pos = positions[comp][0] if isinstance(positions[comp], (list, tuple, np.ndarray)) else positions[comp]
            I += mass * (pos - x_cg) ** 2
            total_mass += mass

        # Add fuel and cargo if present
        try:
            fuel_mass = self.data.data['outputs']['design']['max_fuel']
            cargo_mass = self.data.data['requirements']['cargo_mass']
            x_LEMAC = self.x_LEMAC
            MAC = self.wing_MAC
            x_fuel = x_LEMAC + MAC / 2
            cargo_x_start = self.data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose'] + self.data.data['outputs']['fuselage_dimensions']['l_nose']
            cargo_length = self.data.data['outputs']['fuselage_dimensions']['cargo_length']
            x_cargo = cargo_x_start + cargo_length / 2
            I += fuel_mass * (x_fuel - x_cg) ** 2
            I += cargo_mass * (x_cargo - x_cg) ** 2
            total_mass += fuel_mass + cargo_mass
        except Exception:
            pass

        # 4. Calculate radius of gyration
        if total_mass > 0:
            k = np.sqrt(I / total_mass)
        else:
            k = 0.0
        return k
        

if __name__ == "__main__":
    # Example usage
    data = Data("design3.json")
    landing_loads = LandingLoads(data)
    
    radius_of_gyration = landing_loads.calculate_radius_of_gyration()
    print(f"Radius of gyration about CG: {radius_of_gyration:.2f} m")
    # You can add more functionality to visualize or further analyze the results