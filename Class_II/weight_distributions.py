import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, EvaluateType

class load_diagram:
    def __init__(self, aircraft_data: Data, plot=True) -> None:
        self.aircraft_data = aircraft_data
        
        # Initialize component weights
        self._init_aircraft_parameters()
        self._init_component_weights()
        self.plot = plot

        
    def _init_component_weights(self, weights=True):
        """Initialize all component weightsfrom the aircraft data"""
        g = 9.81  # acceleration due to gravity [m/s^2]
        weights = self.aircraft_data.data['outputs']['component_weights']
        # Keep the original weights for load calculations and add their locations
        # IMPORTANT: Ensure the `component_locations` list correctly aligns
        # with the order of items in `weights.items()` and has the correct length.
        # If the order or number of items in `weights.items()` changes, this list
        # will cause incorrect location assignments or indexing errors.
        component_locations = ['nose', 'wing', 'nose', 'wing', 'forebody', 'nose', 'all', 'forebody', 'tail', 'nose', 'wing', 'wing', 'tail', 'wing', 'wing', 'wing', 'wing', 'nose', 'forebody', 'tailcone', 'nose', 'wing']

        self.component_weights = {
            key: {'weight': value, 'location': loc} 
            for (key, value), loc in zip(weights.items(), component_locations) if key != "total_OEW"  # Exclude total OEW as it's not a component
        }
        
    def _init_aircraft_parameters(self):
        """Initialize aircraft parameters needed for CG calculation"""
        self.x_LEMAC_wing = self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.MAC_wing = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.cargo_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_width = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_width']
        self.cargo_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_height']
        self.cargo_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_width = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_width']
        self.cargo_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_height']
        self.cargo_density = self.aircraft_data.data['requirements']['cargo_density']
        self.cargo_x_start = (self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose'] + 
                                  self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'])
        self.cargo_mass = self.aircraft_data.data['requirements']['cargo_mass']
        self.wing_root_chord = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.wing_x_LE = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.nmax = np.maximum(self.aircraft_data.data['outputs']['general']['nmax'], self.aircraft_data.data['outputs']['general']['n_landing'])
        self.fuel_weight = self.aircraft_data.data['outputs']['design']['total_fuel']  
        self.root_moment = self.aircraft_data.data['outputs']['wing_stress']['max_torque']  # Root moment for CG calculation
        self.v_tail_chord_root = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
        self.v_tail_LE_pos = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']
        self.tail_moment = self.aircraft_data.data['outputs']['horizontal_tail_stress']['max_torque']  # Horizontal tail moment for CG calculation
        
        # Fuselage parameters
        self.l_nose = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose']
        self.l_forebody = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        self.l_afterbody = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']
        self.l_tailcone = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone']
        
        # Tail parameters
        self.x_LE_vertical_tail = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']
        self.x_MAC_vertical_tail = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['MAC']
        self.x_LE_horizontal_tail = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos']
        self.x_MAC_horizontal_tail = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['MAC']
        self.safety_factor = 1.5
        # Add new parameters needed for wing load calculation
        self.MTOW_json = self.aircraft_data.data['outputs']['max']['MTOW']  # Maximum takeoff weight
        self.l_fuselage = (self.l_nose + self.l_forebody + 
                             self.l_afterbody + self.l_tailcone)
        self.x_points = np.arange(0, self.l_fuselage, 0.01)
        self.OEW_loads = np.zeros_like(self.x_points)
        self.fuel_margin = 0 #TODO link to json?
        self.nose_share=0.1
        self.forebody_share=0.25
        self.afterbody_share=0.4
        self.tailcone_share=0.25

    def get_weights(self):

        self.section_weights = {
            'nose': 0.0,
            'forebody': 0.0,
            'cargo': 0.0,
            'wing': 0.0,
            'fuel': 0.0, # Note: Fuel is usually handled separately from OEW
            'afterbody': 0.0,
            'tailcone': 0.0,
            'tail': 0.0,
            'cargo': 0.0
        } # nose, forebody, wing, fuel, afterbody, tailcone, tail starting weights.

        # Add the OEW section weights, starting with the fuselage weights.
        # If 'all' is in the component weights, it will be added to all sections albeit after the share factors.
        for component, details in self.component_weights.items():
            location = details['location']
            weight = details['weight']

            if location == 'nose':
                self.section_weights['nose'] += weight 
            elif location == 'forebody':
                self.section_weights['forebody'] += weight 
            elif location == 'wing':
                self.section_weights['wing'] += weight 
            elif location == 'afterbody':
                self.section_weights['afterbody'] += weight 
            elif location == 'tailcone':
                self.section_weights['tailcone'] += weight 
            elif location == 'tail':
                self.section_weights['tail'] += weight 
            elif location == 'all':
                self.section_weights['nose'] += weight * self.nose_share 
                self.section_weights['forebody'] += weight * self.forebody_share 
                self.section_weights['afterbody'] += weight * self.afterbody_share 
                self.section_weights['tailcone'] += weight * self.tailcone_share 
                
        # Add cargo weight
        cargo_weight = (self.cargo_mass + 10000) * 9.81
        self.section_weights['cargo'] += cargo_weight
        # print(f"Adding cargo weight: {cargo_weight} N to cargo section")
        # Add fuel weight   
        
        # self.section_weights['fuel'] += self.fuel_weight
        # print(f"Adding fuel weight: {self.fuel_weight} N to fuel section")
        # print(f"Calculated OEW: {sum(self.section_weights.values())-self.section_weights['fuel']-self.section_weights['cargo']} N")
        # print(f"Actual OEW by summing component weights: {sum([v['weight'] for v in self.component_weights.values()])} N")

        # Calculate total OEW from the new dictionary (values of the dict)
        self.mtow_from_sections = sum(self.section_weights.values())
        # Make all the values negative
        self.section_weights = {k: -v for k, v in self.section_weights.items()}
        # print(f"section weights (negative values): {self.section_weights}")
        
        # print(f"Calculated MTOW (sum of sections): {self.mtow_from_sections} N")
        # print(f"MTOW from json: {self.MTOW_json} N")
        # print(f"Difference between calculated and json MTOW: {self.mtow_from_sections-self.MTOW_json} N")

        return self.section_weights
    

    def get_load_distribution(self):
        # Define masks for each section using the provided logic
        self.masks = {
            'nose_mask': (self.x_points < self.l_nose),
            'forebody_mask': (self.x_points >= self.l_nose) & (self.x_points < self.l_nose + self.l_forebody),
            'cargo_mask': (self.x_points >= self.cargo_x_start) & (self.x_points < self.cargo_x_start + self.cargo_length),
            'wing_mask': (self.x_points >= self.wing_x_LE) & (self.x_points < self.wing_x_LE + self.wing_root_chord),
            'fuel_mask': (self.x_points >= self.wing_x_LE + self.fuel_margin) & (self.x_points < self.wing_x_LE + self.wing_root_chord - self.fuel_margin),
            'afterbody_mask': (self.x_points >= self.l_nose + self.l_forebody) & (self.x_points < self.l_nose + self.l_forebody + self.l_afterbody),
            'tailcone_mask': (self.x_points >= self.l_nose + self.l_forebody + self.l_afterbody),
            'tail_mask': (self.x_points >= self.x_LE_vertical_tail) & (self.x_points < self.x_LE_vertical_tail + self.v_tail_chord_root)
        }
        self.lengths = {
            'nose': self.l_nose,
            'forebody': self.l_forebody,
            'cargo': self.cargo_length,
            'wing': self.wing_root_chord,
            'fuel': self.wing_root_chord - self.fuel_margin * 2,
            'afterbody': self.l_afterbody,
            'tailcone': self.l_tailcone,
            'tail': self.v_tail_chord_root
        }

        self.load_distributions = {
            'nose': np.zeros_like(self.x_points),
            'forebody': np.zeros_like(self.x_points),
            'cargo': np.zeros_like(self.x_points),
            'wing': np.zeros_like(self.x_points),
            'fuel': np.zeros_like(self.x_points),
            'afterbody': np.zeros_like(self.x_points),
            'tailcone': np.zeros_like(self.x_points),
            'tail': np.zeros_like(self.x_points)
        }

        
        # Calculate load distribution for each section
        self.section_distributions = {
            'nose': self.section_weights['nose'] / self.lengths['nose'] if self.lengths['nose'] != 0 else 0,
            'forebody': self.section_weights['forebody'] / self.lengths['forebody'] if self.lengths['forebody'] != 0 else 0,
            'cargo': self.section_weights['cargo'] / self.lengths['cargo'] if self.lengths['cargo'] != 0 else 0,
            'wing': (self.section_weights['wing'] + (self.mtow_from_sections-75552.16346682305)) / self.lengths['wing'] if self.lengths['wing'] != 0 else 0, # Corrected: wing weight and total MTOW
            'fuel': self.section_weights['fuel'] / self.lengths['fuel'] if self.lengths['fuel'] != 0 else 0,
            'afterbody': self.section_weights['afterbody'] / self.lengths['afterbody'] if self.lengths['afterbody'] != 0 else 0,
            'tailcone': self.section_weights['tailcone'] / self.lengths['tailcone'] if self.lengths['tailcone'] != 0 else 0,
            'tail': (self.section_weights['tail']+75552.16346682305) / self.lengths['tail'] if self.lengths['tail'] != 0 else 0
        }

        # Sum all section distributions into a final load distribution
        self.total_load_distribution = np.zeros_like(self.x_points)
        for section, mask in self.masks.items():
            section_name = section.replace('_mask', '')
            if section_name in self.section_distributions:
                self.load_distributions[section_name][mask] = self.section_distributions[section_name]
                self.total_load_distribution[mask] += self.section_distributions[section_name]
        self.total_load_distribution *= self.nmax
        return self.total_load_distribution, self.load_distributions


    def get_internal_loads(self,show_individual_contributions=False):
        
        # Calculate shear force by integrating the load distribution
        delta_x = self.x_points[1] - self.x_points[0]
        self.shear = np.cumsum(self.total_load_distribution * delta_x)
        shear_force_kN = self.shear / 1e3 # Convert to kN

        # Calculate bending moment by integrating the shear force
        self.moment = np.cumsum(self.shear * delta_x)
        moment_to_counteract = self.moment[-1]
        # Add wing moment
        wing_root_center = self.x_LEMAC_wing + self.MAC_wing / 2
        point_moment_mask = self.x_points >= wing_root_center
        #self.moment[point_moment_mask] += 0

        tail_point_mask = self.x_points >= self.x_LE_vertical_tail
        # add a linearly increasing moment to the vertical tail mask which starts at 0 and ends at moment_to_counteract
        if np.any(tail_point_mask):
            tail_indices = np.where(tail_point_mask)[0]
            n_tail = len(tail_indices)
            if n_tail > 1:
                linear_moment = np.linspace(0, moment_to_counteract, n_tail)
                self.moment[tail_indices] -= linear_moment
            elif n_tail == 1:
                self.moment[tail_indices[0]] -= moment_to_counteract
        self.moment[tail_point_mask] += 75552.16346682305+1
    

        bending_moment_MNm = self.moment / 1e6 # Convert to MNm
        # Define section boundaries and labels
        section_boundaries = [
            0,
            self.l_nose,
            self.l_nose + self.l_forebody,
            self.l_nose + self.l_forebody + self.l_afterbody,
            self.l_nose + self.l_forebody + self.l_afterbody + self.l_tailcone
        ]
        section_labels = ['Nose', 'Forebody', 'Afterbody', 'Tailcone']

        if self.plot:
            # Plot 1: Load Distribution
            plt.figure(figsize=(12, 5))
            if show_individual_contributions:
                colors = plt.cm.get_cmap('tab10', len(self.load_distributions.keys()))
            for i, (section_name, distribution_array) in enumerate(self.load_distributions.items()):
                plt.plot(self.x_points, distribution_array/1000, label=f'{section_name.capitalize()} Contribution', color=colors(i), linestyle='-')
            plt.plot(self.x_points, self.total_load_distribution/1000, label='Total Load Distribution', color='k', linewidth=3, linestyle='-')
            plt.ylabel('Load Distribution (kN/m)')
            plt.xlabel('Longitudinal Position (m)')
            plt.grid(True, which='both', linestyle='--', linewidth=0.7, alpha=0.5)
            plt.legend(fontsize=8)
            ylim_ax1 = plt.ylim()
            for x in section_boundaries:
                plt.axvline(x=x, color='gray', linestyle=':', linewidth=1, alpha=0.7)
            for i in range(len(section_boundaries)-1):
                mid = (section_boundaries[i] + section_boundaries[i+1]) / 2
            plt.text(mid, ylim_ax1[1]*0.95, section_labels[i], ha='center', va='top', fontsize=11, color='gray')
            plt.show()

            # Plot 2: Shear Force
            plt.figure(figsize=(12, 5))
            plt.plot(self.x_points, shear_force_kN, label='Internal Shear Force', color='tab:blue', linewidth=2)
            plt.ylabel('Shear Force (kN)')
            plt.xlabel('Longitudinal Position (m)')
            plt.grid(True, which='both', linestyle='--', linewidth=0.7, alpha=0.5)
            # plt.legend()
            for x in section_boundaries:
                plt.axvline(x=x, color='gray', linestyle=':', linewidth=1, alpha=0.7)
            plt.show()

            # Plot 3: Bending Moment
            plt.figure(figsize=(12, 5))
            plt.plot(self.x_points, bending_moment_MNm, label='Internal Bending Moment', color='tab:orange', linewidth=2)
            plt.plot([self.x_points[-1], self.x_points[-1]], [bending_moment_MNm[-1], 0], color='tab:orange', linewidth=2)
            plt.xlabel('Longitudinal Position (m)')

            plt.ylabel('Bending Moment (MNm)')
            plt.grid(True, which='both', linestyle='--', linewidth=0.7, alpha=0.5)
            # plt.legend()
            for x in section_boundaries:
                plt.axvline(x=x, color='gray', linestyle=':', linewidth=1, alpha=0.7)
            plt.show()
        
if __name__ == "__main__":
    # Example usage
    aircraft_data = Data('design3.json')
    cg_calculation = load_diagram(aircraft_data)
    
    cg_calculation.get_weights()
    
    # Get the load distributions
    total_load_dist, individual_load_dist = cg_calculation.get_load_distribution()
    
    # Plot all diagrams in one go
    cg_calculation.get_internal_loads(show_individual_contributions=True)