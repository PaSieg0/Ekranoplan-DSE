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
        self.cargo_x_start = (self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose'] + 
                            self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'])
        self.cargo_mass = self.aircraft_data.data['requirements']['cargo_mass']
        self.wing_root_chord = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.wing_x_LE = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.nmax = self.aircraft_data.data['outputs']['general']['nmax']
        self.fuel_mass = self.aircraft_data.data['outputs']['design']['max_fuel']  # Maximum fuel mass
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

        self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] = self.l_fuselage
          # Save updated data to JSON file
        self.aircraft_data.save_design(self.design_file)

    def load_diagram(self, show_verification=False):
        """Calculate and plot fuselage load distribution"""
        # Calculate load distribution parameters based on typical fuselage weight distribution
        forebody_share = 0.45
        nose_share = 0.15 
        tail_share = 0.20  
        afterbody_share = 0.20 

        # Additional nose section components (using component_weights which are already in N)
        nose_components = [
            'apu_installed', 'anchor', 'flight_control', 'instruments',
            'furnishings', 'air_conditioning'
        ]
        nose_additional_weight = sum(self.component_weights.get(comp, 0) for comp in nose_components)
        
        # Additional forebody section components (using component_weights which are already in N)
        forebody_components = [
            'military_cargo_handling_system', 'electrical', 'handling_gear'
        ]
        forebody_additional_weight = sum(self.component_weights.get(comp, 0) for comp in forebody_components)

        # Additional tailcone section components (using component_weights which are already in N)
        tailcone_components = ['door']
        tailcone_additional_weight = sum(self.component_weights.get(comp, 0) for comp in tailcone_components)        # Component weight verification is disabled

        # Calculate distributed loads for each section
        # Note: both fuselage share and additional components need nmax factor for load cases
        fuselage_distributed_nose = -(self.component_weights['fuselage'] * nose_share + nose_additional_weight) * self.nmax / self.l_nose
        fuselage_distributed_forebody = -(self.component_weights['fuselage'] * forebody_share + forebody_additional_weight) * self.nmax / self.l_forebody
        fuselage_distributed_afterbody = -self.component_weights['fuselage'] * afterbody_share * self.nmax / self.l_afterbody
        fuselage_distributed_tailcone = -(self.component_weights['fuselage'] * tail_share + tailcone_additional_weight) * self.nmax / self.l_tailcone
        cargo_distributed = -self.cargo_mass * 9.81 * self.nmax / self.cargo_length 
        fuel_margin_from_root_edges = 0.5
        fuel_distributed = -self.fuel_mass * self.nmax / (self.wing_root_chord-2*fuel_margin_from_root_edges)  # Distribute fuel load over fuselage lengt        # Calculate wing loads
        # Components that we have as masses (in kg)
        wing_mass = self.component_masses.get('wing_mass', 0)
        floater_mass = self.component_masses.get('floater_mass', 0)
        engine_mass = self.component_masses.get('engine_mass', 0)
        nacelle_mass = self.component_masses.get('nacelle_group_mass', 0)
        
        # Convert masses to forces (N)
        mass_based_force = (wing_mass + floater_mass + engine_mass + nacelle_mass) * 9.81
        
        # Components that we have as weights (already in N)
        weight_components = ['anti_ice', 'fuel_system', 'avionics', 'starter_pneumatic', 'engine_controls']
        direct_weight_force = sum(self.component_weights.get(comp, 0) for comp in weight_components)
          # Wing-supported component verification is disabled
        
        # Total wing-supported weight (all in N) with load factor
        wing_supported_weight = (mass_based_force + direct_weight_force) * self.nmax
        
        # Calculate empennage weight (using masses)
        empennage_mass = self.component_masses.get('vertical_tail_mass', 0) + self.component_masses.get('horizontal_tail_mass', 0)
        empennage_supported_weight = empennage_mass * 9.81 * self.nmax
          # Calculate net wing lift force (total lift minus wing-supported masses)
        wing_load = self.MTOW * self.nmax - wing_supported_weight  # Convert MTOW to N and multiply by load factor
        wing_load_distributed = wing_load / self.wing_root_chord  # Distribute load over wing root chord length
        # Case 1: No horizontal tail force
        horizontal_tail_force = 0
        empennage_load = -(empennage_supported_weight + horizontal_tail_force)*self.nmax
        empennage_load_distributed = empennage_load / self.v_tail_chord_root
        print(f"\nEmpennage total load (no horizontal tail force): {empennage_load:.2f} N")
        
        # Case 2: With horizontal tail force
        horizontal_tail_force = 0
        empennage_load = - (empennage_supported_weight + horizontal_tail_force)*self.nmax
        empennage_load_distributed = empennage_load / self.v_tail_chord_root
        print(f"Empennage total load (with 80 kN horizontal tail force): {empennage_load:.2f} N\n")
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
        self.x_points = np.arange(0, self.l_fuselage, 0.01)  # Increased resolution
        loads = np.zeros_like(self.x_points)  # Local variable for loads
        cargo_loads = np.zeros_like(self.x_points)  # Add separate array for cargo loads
        aerodynamic_loads = np.zeros_like(self.x_points)   # Add array for wing loads
        fuel_loads = np.zeros_like(self.x_points)   # Add array for fuel loads

        # Add fuselage distributed loads for each section
        nose_mask = (self.x_points < self.l_nose)
        forebody_mask = (self.x_points >= self.l_nose) & (self.x_points < self.l_nose + self.l_forebody)
        afterbody_mask = (self.x_points >= self.l_nose + self.l_forebody) & (self.x_points < self.l_nose + self.l_forebody + self.l_afterbody)
        tailcone_mask = (self.x_points >= self.l_nose + self.l_forebody + self.l_afterbody)
        fuel_mask = (self.x_points >= self.wing_x_LE + fuel_margin_from_root_edges) & (self.x_points < self.wing_x_LE + self.wing_root_chord - fuel_margin_from_root_edges)

        # Add distributed loads (only once per section)
        loads[nose_mask] = fuselage_distributed_nose
        loads[forebody_mask] = fuselage_distributed_forebody
        loads[afterbody_mask] = fuselage_distributed_afterbody
        loads[tailcone_mask] = fuselage_distributed_tailcone

        # Add cargo distributed loads
        cargo_mask = (self.x_points >= self.cargo_x_start) & (self.x_points < self.cargo_x_start + self.cargo_length)
        cargo_loads[cargo_mask] = cargo_distributed

        # Add wing distributed loads
        wing_mask = (self.x_points >= self.wing_x_LE) & (self.x_points < self.wing_x_LE + self.wing_root_chord)
        vertical_tail_mask = (self.x_points >= self.x_LE_vertical_tail) & (self.x_points < self.x_LE_vertical_tail + self.v_tail_chord_root)
        aerodynamic_loads[wing_mask] = wing_load_distributed
        aerodynamic_loads[vertical_tail_mask] = empennage_load_distributed        # Add fuel distributed loads
        fuel_mask = (self.x_points >= self.wing_x_LE + fuel_margin_from_root_edges) & (self.x_points < self.wing_x_LE + self.wing_root_chord - fuel_margin_from_root_edges)
        fuel_loads[fuel_mask] = fuel_distributed        # Running weight verification
        section_weights = []
        section_ends = [self.l_nose, self.l_nose + self.l_forebody, 
                       self.l_nose + self.l_forebody + self.l_afterbody, self.l_fuselage]
        start_idx = 0
        for end in section_ends:
            end_idx = np.searchsorted(self.x_points, end)
            section_weight = np.trapezoid(loads[start_idx:end_idx], self.x_points[start_idx:end_idx])
            section_weights.append(section_weight)
            start_idx = end_idx
        
        total_weight_calculated = sum(section_weights)
        total_cargo_weight = np.trapezoid(cargo_loads, self.x_points)
        
        if show_verification:
            print(f"\nLoad Distribution Weight Verification:")
            print(f"Target fuselage weight: {self.component_weights['fuselage']/1000:.2f} kN")
            print(f"Integrated weight: {total_weight_calculated/1000:.2f} kN")
            print(f"Difference: {abs(self.component_weights['fuselage'] - total_weight_calculated)/1000:.2f} kN")
            print(f"Error: {abs(self.component_weights['fuselage'] - total_weight_calculated)/self.component_weights['fuselage']*100:.2f}%")
            print(f"\nCargo Weight Verification:")
            print(f"Target cargo weight: {self.cargo_mass * 9.81/1000:.2f} kN")
            print(f"Integrated cargo weight: {total_cargo_weight/1000:.2f} kN")

            # Add wing load verification``
            total_wing_load = np.trapezoid(aerodynamic_loads, self.x_points)
            print(f"\nWing Load Verification:")
            print(f"Calculated wing load (MTOW×nmax): {wing_load/1000:.2f} kN")
            print(f"Integrated wing load: {total_wing_load/1000:.2f} kN")
            print(f"Error: {abs(wing_load - total_wing_load)/wing_load*100:.2f}%")        # Calculate total loads
        total_loads = loads + cargo_loads + aerodynamic_loads + fuel_loads
        
        # Define sections
        sections = [
            (0, "Nose Section", self.l_nose),
            (self.l_nose, "Forebody Section", self.l_nose + self.l_forebody),
            (self.l_nose + self.l_forebody, "Afterbody Section", 
             self.l_nose + self.l_forebody + self.l_afterbody),
            (self.l_nose + self.l_forebody + self.l_afterbody, 
             "Tailcone Section", self.l_fuselage)
        ]
        
        # Running weight verification
        section_weights = []
        section_ends = [self.l_nose, self.l_nose + self.l_forebody, 
                       self.l_nose + self.l_forebody + self.l_afterbody, self.l_fuselage]
        start_idx = 0
        for end in section_ends:
            end_idx = np.searchsorted(self.x_points, end)
            section_weight = np.trapezoid(loads[start_idx:end_idx], self.x_points[start_idx:end_idx])
            section_weights.append(section_weight)
            start_idx = end_idx
        
        # Calculate shear force through integration
        self.shear = np.zeros_like(self.x_points)
        for i in range(1, len(self.x_points)):
            self.shear[i] = np.trapezoid(total_loads[:i], self.x_points[:i])
            
        # Calculate bending moment through integration of shear
        self.moment = np.zeros_like(self.x_points)
        for i in range(1, len(self.x_points)):
            self.moment[i] = np.trapezoid(-self.shear[:i], self.x_points[:i])  # Negative shear to match sign convention
        
        # Add wing root moment to all points after the wing root
        wing_root_center = self.wing_x_LE + self.wing_root_chord/2
        moment_mask = self.x_points >= wing_root_center
        self.moment[moment_mask] += -2*self.root_moment

        # Create figure with three subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 15), height_ratios=[1, 1, 1])
          # Plot load distribution in top subplot
        ax1.plot(self.x_points, loads/1000, 'b-', label='Fuselage Load Distribution', linewidth=2)
        ax1.plot(self.x_points, cargo_loads/1000, 'r-', label='Cargo Load Distribution', linewidth=2, alpha=0.6)
        ax1.plot(self.x_points, aerodynamic_loads/1000, 'g-', label='Wing Load Distribution (MTOW×nmax)', linewidth=2, alpha=0.6)
        ax1.plot(self.x_points, fuel_loads/1000, 'm-', label='Fuel Load Distribution', linewidth=2, alpha=0.6)
        ax1.plot(self.x_points, total_loads/1000, 'k-', label='Total Load Distribution', linewidth=3)# Plot shear force in middle subplot
        ax2.plot(self.x_points, self.shear/1000, 'b-', label='Shear Force', linewidth=2)  # Positive for clockwise rotation        # Plot moment diagram in bottom subplot
        ax3.plot(self.x_points, -selfmoment/1000000, 'r-', label='Bending Moment', linewidth=2)  # Positive for upper fiber compression
        
        # Add vertical line at wing root to show where moment increases
        wing_root_center = self.wing_x_LE + self.wing_root_chord/2
        ax3.axvline(x=wing_root_center, color='green', linestyle='--', alpha=0.5,
                   label=f'Wing Root (Added moment: {self.root_moment/1000000:.2f} MN·m)')
        
        # Add section lines and labels to all plots
        for i, (start, label, end) in enumerate(sections):
            # Add vertical lines to all plots
            ax1.axvline(x=start, color='g', linestyle='--', alpha=0.5)
            ax2.axvline(x=start, color='g', linestyle='--', alpha=0.5)
            ax3.axvline(x=start, color='g', linestyle='--', alpha=0.5)
            
            # Add section labels
            mid_x = (start + (end if i < len(sections)-1 else self.l_fuselage)) / 2
            weight_pct = section_weights[i] / (self.component_weights['fuselage'] * self.nmax) * 100
            
            # Labels for each plot
            ax1.text(start, ax1.get_ylim()[1], f"{label}", 
                    ha='right', va='top', rotation=90, fontsize=9)
            ax2.text(start, ax2.get_ylim()[1], f"{label}", 
                    ha='right', va='top', rotation=90, fontsize=9)
            ax3.text(start, ax3.get_ylim()[1], f"{label}", 
                    ha='right', va='top', rotation=90, fontsize=9)

        # Add horizontal reference lines
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Format plots
        ax1.set_xlabel("Fuselage Station (m)")
        ax1.set_ylabel("Load Distribution (kN/m)")
        ax1.set_title("Fuselage Load Distribution")
        ax1.grid(True, alpha=0.3, which='both')
        ax1.legend(loc='center', bbox_to_anchor=(0.5, -0.2), ncol=3)

        ax2.set_xlabel("Fuselage Station (m)")
        ax2.set_ylabel("Shear Force (kN)")
        ax2.set_title("Shear Force Diagram")
        ax2.grid(True, alpha=0.3, which='both')
        ax2.legend()

        ax3.set_xlabel("Fuselage Station (m)")
        ax3.set_ylabel("Bending Moment (MN·m)")
        ax3.set_title("Bending Moment Diagram")
        ax3.grid(True, alpha=0.3, which='both')
        ax3.legend()
          # Maximum values calculations
        max_shear_idx = np.argmax(np.abs(self.shear))
        max_moment_idx = np.argmax(np.abs(self.moment))

        plt.tight_layout()
        plt.show()

        self.update_json()


def main():
    data = Data("design3.json")
    cg_calculator = CGCalculation(data)
    
    # Calculate and print CG positions
    mtow_cg = cg_calculator.calculate_cg(OEW=False)
    oew_cg = cg_calculator.calculate_cg(OEW=True)
    print(f"MTOW CG position: {mtow_cg:.3f} m")
    print(f"OEW CG position: {oew_cg:.3f} m")
    
    # Generate and plot the load diagram
    cg_calculator.load_diagram(show_verification=False)
    
    # Update JSON with CG positions
    cg_calculator.update_json()


if __name__ == "__main__":
    main()




