import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
from Cgcalculation import CGCalculation


class LandingLoads:
    def __init__(self, data: Data):
        self.data = data
        self.center_of_pressure_from_AC = 0.25 # 25% of the MAC from the leading edge
        self.x_LEMAC = self.data.data["outputs"]["wing_design"]["X_LEMAC"]
        self.x_aft_cg = self.data.data["outputs"]["cg_range"]["most_aft_cg"]
        self.x_fore_cg = self.data.data["outputs"]["cg_range"]["most_forward_cg"]
        self.wing_MAC = self.data.data["outputs"]["wing_design"]["MAC"]
        self.stall_speed = self.data.data["requirements"]["stall_speed_landing"]*3.2808399
        self.x_center_of_pressure = self.x_LEMAC + self.center_of_pressure_from_AC * self.wing_MAC
        
    def calculate_radius_of_gyration(self):
        cg_calculator = CGCalculation(data)
        self.mtow_cg = cg_calculator.calculate_cg(OEW=False)
        weights = self.data.data['outputs']['component_weights']
        positions = self.data.data['outputs']['component_positions']
        # Exclude total_OEW if present
        component_masses = {k: v for k, v in weights.items() if k != 'total_OEW'}

        I = 0.0
        total_mass = 0.0
        for comp, mass in component_masses.items():
            if comp != 'fuselage':
                pos = positions[comp]
                distance_from_cg = np.linalg.norm(pos - self.mtow_cg)
                print(f"Component: {comp}, Distance from CG: {distance_from_cg} m")
                I += mass * distance_from_cg ** 2
                total_mass += mass
        # Add fuselage, fuel and cargo. fuselage and cargo modelled as a thin rod
        try:
            fuselage_mass = component_masses['fuselage']
            fuselage_length = self.data.data['outputs']['fuselage_dimensions']['l_fuselage']
            fuselage_cg_position = positions['fuselage']
            fuselage_moment_of_inertia_cm = 1/12* fuselage_mass * fuselage_length ** 2
            fuselage_distance_from_cg = np.linalg.norm(fuselage_cg_position - self.mtow_cg)
            fuselage_moment_of_inertia_about_cg = fuselage_moment_of_inertia_cm + fuselage_mass * fuselage_distance_from_cg ** 2
            fuselage_radius_of_gyration = np.sqrt(fuselage_moment_of_inertia_about_cg / fuselage_mass)

            cargo_mass = self.data.data['requirements']['cargo_mass']
            cargo_length = self.data.data['outputs']['fuselage_dimensions']['cargo_length']
            cargo_cg_position = self.data.data['outputs']['fuselage_dimensions']['l_nose'] + cargo_length / 2
            cargo_moment_of_inertia_cm = 1/12 * cargo_mass * cargo_length ** 2
            cargo_distance_from_cg = np.abs(cargo_cg_position - self.mtow_cg[0])  # Assuming x-axis alignment
            cargo_moment_of_inertia_about_cg = cargo_moment_of_inertia_cm + cargo_mass * cargo_distance_from_cg ** 2
            cargo_radius_of_gyration = np.sqrt(cargo_moment_of_inertia_about_cg / cargo_mass)
            
            fuel_mass = self.data.data['outputs']['design']['max_fuel']
            fuel_position = positions['wing']
            fuel_distance_from_cg = np.linalg.norm(fuel_position - self.mtow_cg)
            fuel_moment_of_inertia_about_cg = fuel_mass * fuel_distance_from_cg ** 2
            fuel_radius_of_gyration = np.sqrt(fuel_moment_of_inertia_about_cg / fuel_mass)
            I += fuselage_moment_of_inertia_about_cg + cargo_moment_of_inertia_about_cg + fuel_moment_of_inertia_about_cg

            total_mass += fuselage_mass + cargo_mass + fuel_mass

        except Exception:
            pass

        # 4. Calculate radius of gyration
        if total_mass > 0:
            self.k = np.sqrt(I / total_mass)
        else:
            self.k = 0.0
        return self.k

    def calculate_k2(self, x_pos):
        """
        Calculates the k2 factor for an array of longitudinal positions.

        The k2 factor is determined based on the position (x_pos) relative
        to the center of gravity (x_cg) and the forebody length.

        Args:
            x_pos (np.ndarray): An array of longitudinal positions along the fuselage.

        Returns:
            np.ndarray: An array of k2 values, same length as x_pos.
        """
        forebody_length = self.data.data['outputs']['fuselage_dimensions']['l_forebody'] + self.data.data['outputs']['fuselage_dimensions']['l_nose']
        x_cg = self.mtow_cg[0]
        d = np.abs(x_cg - forebody_length)

        k2 = np.zeros_like(x_pos, dtype=float)

        # Condition 1: 0 <= x_pos < x_cg
        mask1 = (x_pos < x_cg) & (x_pos >= 0)
        k2[mask1] = 1 + np.abs(x_cg - x_pos[mask1]) / (forebody_length - d)

        # Condition 2: x_cg < x_pos < forebody_length
        mask2 = (x_pos < forebody_length) & (x_pos > x_cg)
        k2[mask2] = 1.0

        return k2
    
    def calculate_load(self):
        MTOM = self.data.data['outputs']['design']['MTOM']*2.2046
        x_over_i_y = (self.x_center_of_pressure/self.k)**2
        self.load = (0.0085*2*(MTOM/(1+x_over_i_y))**(2/3)*self.stall_speed**2*(1/np.tan(0.52359878))**(2/3)+MTOM/3)*0.4535
        self.data.data['outputs']['general']['n_landing'] = self.load/landing_loads.data.data['outputs']['design']['MTOM']
        self.data.save_design("design3.json")


if __name__ == "__main__":
    # Example usage
    data = Data("design3.json")
    landing_loads = LandingLoads(data)
    radius_of_gyration = landing_loads.calculate_radius_of_gyration()
    landing_loads.calculate_load()
    print(f"The load is: {landing_loads.load}")
    print(f"n: {landing_loads.load/landing_loads.data.data['outputs']['design']['MTOM']}")
