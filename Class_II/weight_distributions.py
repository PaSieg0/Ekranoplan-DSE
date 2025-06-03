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
        
        # Initialize component masses
        self._init_component_masses()
        self._init_aircraft_parameters()
        
    def _init_component_masses(self):
        """Initialize all component masses and weights from the aircraft data"""
        g = 9.81  # acceleration due to gravity [m/s^2]
        weights = self.aircraft_data.data['outputs']['component_weights']
        
        # Convert weights [N] to masses [kg] by dividing with g
        self.component_masses = {
            f"{key}_mass": value/g for key, value in weights.items()
            if key != "total_OEW"  # Exclude total OEW as it's not a component
        }
        # Keep the original weights for load calculations
        self.component_weights = {
            key: value for key, value in weights.items()
            if key != "total_OEW"  # Exclude total OEW as it's not a component
        }

    def _init_aircraft_parameters(self):
        """Initialize aircraft parameters needed for CG calculation"""
        self.x_LEMAC_wing = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.MAC_wing = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.cargo_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_width = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_width']
        self.cargo_height = self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_height']
        self.cargo_density = self.aircraft_data.data['requirements']['cargo_density']
        self.cargo_x_start = (self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose'] + 
                            self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'])
        self.fuel_mass = self.aircraft_data.data['outputs']['design']['max_fuel']
        self.cargo_mass = self.aircraft_data.data['requirements']['cargo_mass']
        self.wing_root_chord = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.wing_x_LE = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.nmax = self.aircraft_data.data['outputs']['general']['nmax']

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
        
        # Add new parameters needed for wing load calculation
        self.MTOW = self.aircraft_data.data['outputs']['max']['MTOW']  # Maximum takeoff weight
        self.l_fuselage = (self.l_nose + self.l_forebody + 
                            self.l_afterbody + self.l_tailcone)

    def get_component_position(self, component: str, nacelle_length: float = 0.0) -> float:
        """Get x-position for a given component"""
        if component == "fuselage_mass":
            return self.l_fuselage*0.45
        elif component in ["wing_mass", "floater_mass"]:
            return self.x_LEMAC_wing + self.MAC_wing / 2
        elif component in ["engine_mass", "nacelle_group_mass"]:
            return self.x_LEMAC_wing - nacelle_length/2
        elif component == "horizontal_tail_mass":
            return self.x_LE_horizontal_tail + self.x_MAC_horizontal_tail / 2
        elif component == "vertical_tail_mass":
            return self.x_LE_vertical_tail + self.x_MAC_vertical_tail / 2
        elif component in ["door_mass", "flight_control_mass"]:
            return self.l_nose
        elif component == "flight_control":
            return self.l_nose
        else:
            return self.l_fuselage / 2

    def calculate_cg(self, nacelle_length: float = 0.0, OEW: bool = False, plot: bool = False) -> float:
        """Calculate center of gravity position"""
        cg_x = 0.0
        if OEW:
            total_mass = sum(self.component_masses.values())
        else:
            total_mass = sum(self.component_masses.values()) + self.fuel_mass + self.cargo_mass

        # Calculate component contributions
        for component, mass in self.component_masses.items():
            x_position = self.get_component_position(component, nacelle_length)
            cg_x += mass * x_position

        # Add fuel and cargo if not OEW
        if not OEW:
            cg_x += self.cargo_mass * (self.cargo_x_start + self.cargo_length / 2)
            cg_x += self.fuel_mass * (self.x_LEMAC_wing + self.MAC_wing/2)

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
            (0, "Nose", self.l_nose),
            (self.l_nose, "Straight Section", self.l_nose + self.l_forebody),
            (self.l_nose + self.l_forebody, "Afterbody", 
             self.l_nose + self.l_forebody + self.l_afterbody),
            (self.l_nose + self.l_forebody + self.l_afterbody, 
             "Tailcone", self.l_fuselage)
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
        oew_cg = self.calculate_cg(OEW=True)
        mtow_cg = self.calculate_cg(OEW=False)
        
        # Update CG values in the JSON data
        if 'cg_positions' not in self.aircraft_data.data['outputs']:
            self.aircraft_data.data['outputs']['cg_positions'] = {}
            
        self.aircraft_data.data['outputs']['cg_positions'].update({
            'OEW_CG': oew_cg,
            'MTOW_CG': mtow_cg
        })
          # Save updated data to JSON file
        self.aircraft_data.save_design(self.design_file)

    def load_diagram(self, show_verification=False):
        """Calculate and plot fuselage load distribution"""
        # Calculate load distribution parameters based on typical fuselage weight distribution
        forebody_share = 0.45
        nose_share = 0.15 
        tail_share = 0.20  
        afterbody_share = 0.20 

        # Calculate distributed loads for each section
        fuselage_distributed_nose = self.component_weights['fuselage'] * nose_share / self.l_nose
        fuselage_distributed_forebody = self.component_weights['fuselage'] * forebody_share / self.l_forebody
        fuselage_distributed_afterbody = self.component_weights['fuselage'] * afterbody_share / self.l_afterbody
        fuselage_distributed_tailcone = self.component_weights['fuselage'] * tail_share / self.l_tailcone

        # Calculate wing loads
        wing_load = self.MTOW * self.nmax  # Convert MTOW to N and multiply by load factor
        wing_load_distributed = wing_load / self.wing_root_chord  # Distribute load over wing root chord length

        print(f"Wing_load/MTOW: {wing_load/self.MTOW} kN/m")
        # Calculate section weights for verification
        calculated_nose_weight = fuselage_distributed_nose * self.l_nose
        calculated_forebody_weight = fuselage_distributed_forebody * self.l_forebody
        calculated_afterbody_weight = fuselage_distributed_afterbody * self.l_afterbody
        calculated_tailcone_weight = fuselage_distributed_tailcone * self.l_tailcone

        if show_verification:
            # Print section weights and verify percentages
            print("\nFuselage Section Weight Distribution:")
            print(f"Nose section (0 to {self.l_nose:.1f}m):")
            print(f"  Weight: {calculated_nose_weight/1000:.2f} kN")
            print(f"  Percentage: {calculated_nose_weight/self.component_weights['fuselage']*100:.1f}%")
            
            print(f"\nForebody section ({self.l_nose:.1f}m to {self.l_nose + self.l_forebody:.1f}m):")
            print(f"  Weight: {calculated_forebody_weight/1000:.2f} kN")
            print(f"  Percentage: {calculated_forebody_weight/self.component_weights['fuselage']*100:.1f}%")
            
            print(f"\nAfterbody section ({self.l_nose + self.l_forebody:.1f}m to {self.l_nose + self.l_forebody + self.l_afterbody:.1f}m):")
            print(f"  Weight: {calculated_afterbody_weight/1000:.2f} kN")
            print(f"  Percentage: {calculated_afterbody_weight/self.component_weights['fuselage']*100:.1f}%")
            
            print(f"\nTailcone section ({self.l_nose + self.l_forebody + self.l_afterbody:.1f}m to {self.l_fuselage:.1f}m):")
            print(f"  Weight: {calculated_tailcone_weight/1000:.2f} kN")
            print(f"  Percentage: {calculated_tailcone_weight/self.component_weights['fuselage']*100:.1f}%")
        
        # Generate detailed load distribution
        x_points = np.arange(0, self.l_fuselage, 0.01)  # Increased resolution
        loads = np.zeros_like(x_points)
        cargo_loads = np.zeros_like(x_points)  # Add separate array for cargo loads
        wing_loads = np.zeros_like(x_points)   # Add array for wing loads

        # Add fuselage distributed loads for each section
        nose_mask = (x_points < self.l_nose)
        forebody_mask = (x_points >= self.l_nose) & (x_points < self.l_nose + self.l_forebody)
        afterbody_mask = (x_points >= self.l_nose + self.l_forebody) & (x_points < self.l_nose + self.l_forebody + self.l_afterbody)
        tailcone_mask = (x_points >= self.l_nose + self.l_forebody + self.l_afterbody)
        
        loads[nose_mask] += fuselage_distributed_nose
        loads[forebody_mask] += fuselage_distributed_forebody
        loads[afterbody_mask] += fuselage_distributed_afterbody
        loads[tailcone_mask] += fuselage_distributed_tailcone

        # Add cargo distributed loads
        cargo_mask = (x_points >= self.cargo_x_start) & (x_points < self.cargo_x_start + self.cargo_length)
        cargo_distributed = self.cargo_mass * 9.81 / self.cargo_length  # Convert mass to weight and distribute evenly
        cargo_loads[cargo_mask] = cargo_distributed

        # Add wing distributed loads
        wing_mask = (x_points >= self.wing_x_LE) & (x_points < self.wing_x_LE + self.wing_root_chord)
        wing_loads[wing_mask] = wing_load_distributed

        print(f"end of cargo location: {self.cargo_x_start + self.cargo_length:.1f} m")
        print(f" start of tailcone: {self.l_nose + self.l_forebody + self.l_afterbody:.1f} m")
        # Running weight verification
        section_weights = []
        section_ends = [self.l_nose, self.l_nose + self.l_forebody, 
                       self.l_nose + self.l_forebody + self.l_afterbody, self.l_fuselage]
        start_idx = 0
        
        for end in section_ends:
            end_idx = np.searchsorted(x_points, end)
            section_weight = np.trapz(loads[start_idx:end_idx], x_points[start_idx:end_idx])
            section_weights.append(section_weight)
            start_idx = end_idx        
            total_weight_calculated = sum(section_weights)
        total_cargo_weight = np.trapz(cargo_loads, x_points)
        
        if show_verification:
            print(f"\nLoad Distribution Weight Verification:")
            print(f"Target fuselage weight: {self.component_weights['fuselage']/1000:.2f} kN")
            print(f"Integrated weight: {total_weight_calculated/1000:.2f} kN")
            print(f"Difference: {abs(self.component_weights['fuselage'] - total_weight_calculated)/1000:.2f} kN")
            print(f"Error: {abs(self.component_weights['fuselage'] - total_weight_calculated)/self.component_weights['fuselage']*100:.2f}%")
            print(f"\nCargo Weight Verification:")
            print(f"Target cargo weight: {self.cargo_mass * 9.81/1000:.2f} kN")
            print(f"Integrated cargo weight: {total_cargo_weight/1000:.2f} kN")

            # Add wing load verification
            total_wing_load = np.trapz(wing_loads, x_points)
            print(f"\nWing Load Verification:")
            print(f"Calculated wing load (MTOW×nmax): {wing_load/1000:.2f} kN")
            print(f"Integrated wing load: {total_wing_load/1000:.2f} kN")
            print(f"Error: {abs(wing_load - total_wing_load)/wing_load*100:.2f}%")

        # Create the plot with enhanced visualization
        plt.figure(figsize=(12, 8))
        
        # Plot individual loads
        plt.plot(x_points, loads/1000, 'b-', label='Fuselage Load Distribution', linewidth=2)  # Convert to kN/m
        plt.plot(x_points, cargo_loads/1000, 'r-', label='Cargo Load Distribution', linewidth=2, alpha=0.6)  # Add cargo load plot
        plt.plot(x_points, wing_loads/1000, 'g-', label='Wing Load Distribution (MTOW×nmax)', linewidth=2, alpha=0.6)  # Add wing load plot
        
        # Plot total loads
        total_loads = loads + cargo_loads + wing_loads  # Include wing loads in total
        plt.plot(x_points, total_loads/1000, 'k-', label='Total Load Distribution', linewidth=2)

        # Add section lines and labels with enhanced formatting
        sections = [
            (0, "Nose Section", self.l_nose),
            (self.l_nose, "Forebody Section", self.l_nose + self.l_forebody),
            (self.l_nose + self.l_forebody, "Afterbody Section", 
             self.l_nose + self.l_forebody + self.l_afterbody),
            (self.l_nose + self.l_forebody + self.l_afterbody, 
             "Tailcone Section", self.l_fuselage)
        ]
        
        section_weights = [calculated_nose_weight, calculated_forebody_weight, 
                         calculated_afterbody_weight, calculated_tailcone_weight]
        
        for i, (start, label, end) in enumerate(sections):
            # Vertical section boundaries
            plt.axvline(x=start, color='g', linestyle='--', alpha=0.5)
            
            # Section labels with weight and percentage
            mid_x = (start + (end if i < len(sections)-1 else self.l_fuselage)) / 2
            weight_pct = section_weights[i] / self.component_weights['fuselage'] * 100
            plt.text(mid_x, plt.ylim()[1], 
                    f"{label}\n{section_weights[i]/1000:.1f} kN\n({weight_pct:.1f}%)",
                    ha='center', va='top', fontsize=9)
            
            # Add horizontal reference line at zero
            plt.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Format the plot
        plt.xlabel("Fuselage Station (m)")
        plt.ylabel("Load Distribution (kN/m)")
        plt.title("Fuselage Load Distribution by Section")
        plt.grid(True, alpha=0.3, which='both')        
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
        plt.tight_layout()
        plt.show()


def main():
    data = Data("design3.json")
    cg_calculator = CGCalculation(data)
    
    # Calculate and print CG positions
    mtow_cg = cg_calculator.calculate_cg(OEW=False)
    oew_cg = cg_calculator.calculate_cg(OEW=True)
    print(f"Total CG position: {mtow_cg:.3f} m")
    print(f"OEW CG position: {oew_cg:.3f} m")
    cg_calculator.load_diagram()

    # Update JSON with CG positions
    cg_calculator.update_json()


if __name__ == "__main__":
    main()




