import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

class CGCalculation:
    def __init__(self, aircraft_data: Data) -> None:
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f'design{self.design_number}.json'
        self.aircraft_data = aircraft_data
        
        self._init_aircraft_parameters()
        self.update_component_positions()

        self._init_component_masses()
        self._init_component_positions()

        self.wing_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']
        self.cargo_bottom = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_bottom']
        
    def _init_component_masses(self):
        """Initialize all component masses from the aircraft data"""
        weights = self.aircraft_data.data['outputs']['component_weights']
        self.component_masses = {
            f"{key}_mass": value for key, value in weights.items()
            if key != "total_OEW"  # Exclude total OEW as it's not a component
        }

    def _init_component_positions(self):
        positions = self.aircraft_data.data['outputs']['component_positions']
        self.component_positions = {
            f"{key}_position": value for key, value in positions.items()
            }

    def _init_aircraft_parameters(self):
        """Initialize aircraft parameters needed for CG calculation"""
        self.x_LEMAC_wing = self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.MAC_wing = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.cargo_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_width = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_width']
        self.cargo_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_height']
        self.cargo_density = self.aircraft_data.data['requirements']['cargo_density']
        self.cargo_x_start = (self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose'] + 
                            self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'])
        self.fuel_mass = self.aircraft_data.data['outputs']['design']['max_fuel']
        self.cargo_mass = self.aircraft_data.data['requirements']['cargo_mass']

        
        # Fuselage parameters
        self.nose_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose']
        self.fus_straight_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        self.fus_afterbody_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']
        self.fus_tailcone_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone']
        self.total_fuselage_length = (self.nose_length + self.fus_straight_length +
                                    self.fus_afterbody_length + self.fus_tailcone_length)
        
        self.cargo_bottom = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_bottom']
        
        self.fuselage_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage']
        self.endplate_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['endplate_height']

        self.wing_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']

        self.horizontal_tail_height = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height']
        self.vertical_tail_height = self.fuselage_height + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']/2
        
        # Tail parameters
        self.x_LE_vertical_tail = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']
        self.x_MAC_vertical_tail = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['MAC']
        self.x_LE_horizontal_tail = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos']
        self.x_MAC_horizontal_tail = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['MAC']
        
        self.total_fuselage_length = (self.nose_length + self.fus_straight_length + 
                                    self.fus_afterbody_length + self.fus_tailcone_length)
        
    def update_component_positions(self):
        self.aircraft_data.data['outputs']['component_positions']['fuselage'] = [0.45*self.total_fuselage_length, 0, self.fuselage_height/2]
        self.aircraft_data.data['outputs']['component_positions']['wing'] = [self.x_LEMAC_wing + self.MAC_wing / 2, 0, self.wing_height]
        # TODO: CHANGE THE Z POSITION OF THE FLOATER
        self.aircraft_data.data['outputs']['component_positions']['floater'] = [self.x_LEMAC_wing + self.MAC_wing / 2, 0, 0]
        self.aircraft_data.data['outputs']['component_positions']['floater_endplate'] = [self.x_LEMAC_wing, 0, self.endplate_height]
        # TODO: MAKE ENGINE POSITION DYNAMIC
        engine_x = self.x_LEMAC_wing - self.aircraft_data.data['inputs']['engine']['engine_length']/2
        engine_z = np.mean(self.aircraft_data.data['outputs']['engine_positions']['z_engines'])
        self.aircraft_data.data['outputs']['component_positions']['engine'] = [engine_x, 0, engine_z]
        self.aircraft_data.data['outputs']['component_positions']['nacelle_group'] = [engine_x, 0, engine_z]
        self.aircraft_data.data['outputs']['component_positions']['horizontal_tail'] = [self.x_LE_horizontal_tail + self.x_MAC_horizontal_tail / 2, 0, self.horizontal_tail_height]
        self.aircraft_data.data['outputs']['component_positions']['vertical_tail'] = [self.x_LE_vertical_tail + self.x_MAC_vertical_tail / 2, 0, self.vertical_tail_height]
        self.aircraft_data.data['outputs']['component_positions']['door'] = [self.nose_length, 0, self.fuselage_height / 2]
        self.aircraft_data.data['outputs']['component_positions']['flight_control'] = [self.nose_length, 0, self.fuselage_height / 2]
        self.aircraft_data.data['outputs']['component_positions']['anchor'] = [self.nose_length / 2, 0, self.fuselage_height / 4]
        mid_fuse = [self.total_fuselage_length / 2, 0, self.fuselage_height / 2]
        for comp in ['air_conditioning', 'anti_ice', 'apu_installed', 'avionics', 'electrical', 'furnishings', 'handling_gear', 'instruments', 'starter_pneumatic', 'engine_controls', 'fuel_system']:
            self.aircraft_data.data['outputs']['component_positions'][comp] = mid_fuse
        self.aircraft_data.data['outputs']['component_positions']['military_cargo_handling_system'] = [self.cargo_x_start + self.cargo_length / 2, 0, self.cargo_bottom + self.cargo_height / 2]
        

    def calculate_cg(self, nacelle_length: float = 0.0, OEW: bool = False, plot: bool = False) -> float:
        """Calculate center of gravity position"""
        cg_x = np.array([0.0, 0.0, 0.0])
        if OEW:
            total_mass = sum(self.component_masses.values())
        else:
            total_mass = sum(self.component_masses.values()) + self.fuel_mass + self.cargo_mass

        # print(f"Total mass: {total_mass} kg")

        # Calculate component contributions
        for component, mass in self.component_masses.items():
            component_name = component.replace('_mass', '_position')
            x_position = np.asarray(self.component_positions[component_name])
            # print(f"Component: {component_name}, Weight: {mass} N, Position: {x_position} m")
            cg_x += mass * x_position
            # print(f"cg_x", cg_x)

        # Add fuel and cargo if not OEW
        if not OEW:
            self.fuel_position = np.array([
                self.x_LEMAC_wing + self.MAC_wing / 2,
                0,
                self.wing_height
            ])

            self.cargo_position = np.array([
                self.cargo_x_start + self.cargo_length / 2,
                0,
                self.cargo_bottom + self.cargo_height / 2
            ])

            cg_x += self.cargo_mass * self.cargo_position
            cg_x += self.fuel_mass * self.fuel_position

        if plot:
            self._plot_cg(cg_x, total_mass, OEW, nacelle_length)

        return cg_x / total_mass

    def _plot_cg(self, cg_x: float, total_mass: float, OEW: bool, nacelle_length: float):
        """Plot CG and component positions"""
        x_positions = []
        masses = []
        labels = []

        # Add component positions
        for component, mass in self.component_masses.items():
            x_position = self.get_component_position(component, nacelle_length)
            x_positions.append(x_position)
            masses.append(mass)
            labels.append(component)

        # Add fuel and cargo if not OEW
        if not OEW:
            x_positions.extend([
                self.cargo_x_start + self.cargo_length / 2,
                self.x_LEMAC_wing + self.MAC_wing/2
            ])
            masses.extend([self.cargo_mass, self.fuel_mass])
            labels.extend(["cargo_mass", "fuel_mass"])

        # Create plot
        plt.figure(figsize=(12, 3))
        plt.scatter(x_positions, [1]*len(x_positions), s=100, c='b', alpha=0.6, label='Components')
        
        # Add labels
        for i, label in enumerate(labels):
            plt.text(x_positions[i], 1.02, label, rotation=90, va='bottom', ha='center', fontsize=8)

        # Add fuselage sections
        sections = [
            (0, "Nose", self.nose_length),
            (self.nose_length, "Straight Section", self.nose_length + self.fus_straight_length),
            (self.nose_length + self.fus_straight_length, "Afterbody", 
             self.nose_length + self.fus_straight_length + self.fus_afterbody_length),
            (self.nose_length + self.fus_straight_length + self.fus_afterbody_length, 
             "Tailcone", self.total_fuselage_length)
        ]

        for start, label, end in sections:
            plt.axvline(x=start, color='g', linestyle=':', alpha=0.5)
            plt.text(start, 0.95, f"{label}\n({start:.1f}m)", ha='right', va='top', rotation=90)

        # Add CG line and formatting
        cg_label = "OEW CG" if OEW else "CG"
        plt.axvline(cg_x / total_mass, color='r', linestyle='--', label=cg_label)
        plt.xlabel("Fuselage X Position (m)")
        plt.yticks([])
        plt.title("Operating Empty Weight CG" if OEW else "Component Positions and Center of Gravity")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


    def update_json(self):
        """Update the JSON file with calculated CG positions"""
        self.oew_cg = self.calculate_cg(OEW=True)
        self.mtow_cg = self.calculate_cg(OEW=False)


        self.aircraft_data.data['outputs']['cg_range']['OEW_cg'] = list(self.oew_cg)
        self.aircraft_data.data['outputs']['cg_range']['MTOW_cg'] = list(self.mtow_cg)
        
        # # Update CG values in the JSON data
        # if 'cg_positions' not in self.aircraft_data.data['outputs']:
        #     self.aircraft_data.data['outputs']['cg_positions'] = {}
            
        # self.aircraft_data.data['outputs']['cg_positions'].update({
        #     'OEW_CG': oew_cg,
        #     'MTOW_CG': mtow_cg
        # })
        
        # Save updated data to JSON file
        self.aircraft_data.save_design(self.design_file)

    def load_diagram(self, fore_and_afterbody_share = 0.35):
        # Calculate distributed loads
        fuselage_distributed_nose = self.component_masses['fuselage_mass'] * (1 - 2*fore_and_afterbody_share) / (2 * self.nose_length)
        fuselage_distributed_forebody = self.component_masses['fuselage_mass']*fore_and_afterbody_share/ self.fus_straight_length
        fuselage_distributed_afterbody = self.component_masses['fuselage_mass']*fore_and_afterbody_share/ self.fus_afterbody_length
        fuselage_tailcone_distributed = self.component_masses['fuselage_mass'] * (1-2*fore_and_afterbody_share) / (2 * self.fus_tailcone_length)
        cargo_distributed = self.cargo_mass * 9.81 / self.cargo_length

        print(f"Fuselage distributed load (nose): {fuselage_distributed_nose:.2f} N/m")
        print(f"Fuselage distributed load (forebody): {fuselage_distributed_forebody:.2f} N/m")
        print(f"Fuselage distributed load (afterbody): {fuselage_distributed_afterbody:.2f} N/m")
        print(f"Fuselage distributed load (tailcone): {fuselage_tailcone_distributed:.2f} N/m")
        print(f"Cargo distributed load: {cargo_distributed:.2f} N/m")

        x_points = np.arange(0, self.total_fuselage_length, 0.05)
        loads = np.zeros_like(x_points)
        
        # Add fuselage distributed loads for each section
        nose_mask = (x_points < self.nose_length)
        forebody_mask = (x_points >= self.nose_length) & (x_points < self.nose_length + self.fus_straight_length)
        afterbody_mask = (x_points >= self.nose_length + self.fus_straight_length) & (x_points < self.nose_length + self.fus_straight_length + self.fus_afterbody_length)
        tailcone_mask = (x_points >= self.nose_length + self.fus_straight_length + self.fus_afterbody_length)
        
        loads[nose_mask] += fuselage_distributed_nose
        loads[forebody_mask] += fuselage_distributed_forebody
        loads[afterbody_mask] += fuselage_distributed_afterbody
        loads[tailcone_mask] += fuselage_tailcone_distributed
        
        # Add cargo distributed load
        cargo_mask = (x_points >= self.cargo_x_start) & (x_points <= self.cargo_x_start + self.cargo_length)
        loads[cargo_mask] += cargo_distributed
        
        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(x_points, loads, 'b-', label='Total Load', linewidth=2)
        
        # Add fuselage section lines
        sections = [
            (0, "Nose", self.nose_length),
            (self.nose_length, "Straight Section", self.nose_length + self.fus_straight_length),
            (self.nose_length + self.fus_straight_length, "Afterbody", 
             self.nose_length + self.fus_straight_length + self.fus_afterbody_length),
            (self.nose_length + self.fus_straight_length + self.fus_afterbody_length, 
             "Tailcone", self.total_fuselage_length)
        ]
        
        for start, label, end in sections:
            plt.axvline(x=start, color='g', linestyle=':', alpha=0.5)
            plt.text(start, plt.ylim()[1], f"{label}\n({start:.1f}m)", 
                    ha='right', va='top', rotation=90)
        
        # Highlight cargo area
        plt.axvspan(self.cargo_x_start, self.cargo_x_start + self.cargo_length, 
                   alpha=0.2, color='r', label='Cargo Area')
        
        # Add CG positions
        mtow_cg = self.calculate_cg(OEW=False)[0]
        oew_cg = self.calculate_cg(OEW=True)[0]
        plt.axvline(x=mtow_cg, color='red', linestyle='--', label='MTOW CG')
        plt.axvline(x=oew_cg, color='blue', linestyle='--', label='OEW CG')
        
        # Format the plot
        plt.xlabel("Fuselage Station (m)")
        plt.ylabel("Load Distribution (N/m)")
        plt.title("Load Distribution Along Fuselage")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()
    
def main():
    data = Data("design3.json")
    cg_calculator = CGCalculation(data)
    
    # Calculate and print CG positions
    mtow_cg = cg_calculator.calculate_cg(OEW=False)
    oew_cg = cg_calculator.calculate_cg(OEW=True)
    print(f"Total CG position: {mtow_cg} m")
    print(f"OEW CG position: {oew_cg} m")
    cg_calculator.load_diagram()

    # Update JSON with CG positions
    cg_calculator.update_json()

if __name__ == "__main__":
    main()




