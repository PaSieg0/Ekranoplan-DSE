import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, EvaluateType
from AerodynamicForces import AerodynamicForces

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
        self.root_moment = -50000000  # Root moment for CG calculation
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

        # Add more vertical space between subplots)
        # Add more vertical space between subplots and adjust bottom margin for x-label visibility
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

    def load_diagram(self, show_verification=False, sensitivity_study=True, nominal_shear_max=9278.751924008404, nominal_moment_max=259.8050538722353):
        """Calculate and plot fuselage load distribution"""
        # Calculate load distribution parameters based on typical fuselage weight distribution

        # Components that we have as masses (in kg)
        wing_mass = self.component_masses.get('wing_mass', 0)
        floater_mass = self.component_masses.get('floater_mass', 0)
        engine_mass = self.component_masses.get('engine_mass', 0)
        nacelle_mass = self.component_masses.get('nacelle_group_mass', 0)
        
        # Convert masses to forces (N)
       
       
          # Generate detailed load distribution
        self.x_points = np.arange(0, self.l_fuselage, 0.01)  # Increased resolution
        aerodynamic_loads = np.zeros_like(self.x_points)   # Add array for wing loads

        wing_load = self.MTOW * self.nmax*self.safety_factor #- wing_supported_weight  # Convert MTOW to N and multiply by load factor
        wing_load_distributed = wing_load / self.wing_root_chord  # Distribute load over wing root chord length
        # Add wing distributed loads
        wing_mask = (self.x_points >= self.wing_x_LE) & (self.x_points < self.wing_x_LE + self.wing_root_chord)
        aerodynamic_loads[wing_mask] = wing_load_distributed
        #aerodynamic_loads[vertical_tail_mask] = empennage_load_distributed        # Add fuel distributed loads


        
        total_loads = aerodynamic_loads #+ fuselage_loads  + cargo_loads + fuel_loads
        
        # Define sections
        
        sensitivity_array_wing_root_chord = np.arange(0.005, 0.015, 0.001)
        
        # Running weight verification

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
        ax1.plot(self.x_points, aerodynamic_loads/1000, 'g-', label='Wing Load Distribution')
        ax2.plot(self.x_points, self.shear/1000, 'b-', label='Internal Shear Force', linewidth=2)  # Positive for clockwise rotation        # Plot moment diagram in bottom subplot
        ax3.plot(self.x_points, -self.moment/1000000, 'r-', label='Internal Bending Moment', linewidth=2)  # Positive for upper fiber compression
        # Add vertical line to bring moment to zero at the end
        final_shear = self.shear[-1]/1000  # Get the final shear value

        final_moment = -self.moment[-1]/1000000  # Get the final moment value
        print(f"Final maximum shear: {np.max(abs(final_shear))} kN, Final maximum moment: {np.max(abs(final_moment))} MN·m")
        ax3.plot([self.l_fuselage, self.l_fuselage], [final_moment, 0], 'r', linewidth=2, alpha=0.7)
        ax2.plot([self.l_fuselage, self.l_fuselage], [final_shear, 0], 'b', linewidth=2, alpha=0.7)

        # Add vertical lines at wing root LE and TE for all plots
        wing_le = self.wing_x_LE
        wing_te = self.wing_x_LE + self.wing_root_chord

        for ax in [ax1, ax2, ax3]:
            ax.axvline(x=wing_le, color='green', linestyle='--', alpha=0.5, label='Wing Root LE')
            ax.axvline(x=wing_te, color='green', linestyle='--', alpha=0.5, label='Wing Root TE')

        # Add legend for each subplot in the upper left
        for ax in [ax1, ax2, ax3]:
            handles, labels = ax.get_legend_handles_labels()
            # Remove duplicates while preserving order
            seen = set()
            unique_handles = []
            unique_labels = []
            for h, l in zip(handles, labels):
                if l not in seen:
                    unique_handles.append(h)
                    unique_labels.append(l)
                    seen.add(l)
            ax.legend(handles=unique_handles, labels=unique_labels, loc='upper left', fontsize=10)

        # Add legend entries for the first plot (ax1)
        handles, labels = ax1.get_legend_handles_labels()
        # Only add if not already present
        if 'Wing Root LE' not in labels:
            handles.append(plt.Line2D([], [], color='green', linestyle='--', alpha=0.5, label='Wing Root LE'))
            labels.append('Wing Root LE')
        if 'Wing Root TE' not in labels:
            handles.append(plt.Line2D([], [], color='green', linestyle='--', alpha=0.5, label='Wing Root TE'))
            labels.append('Wing Root TE')
        ax1.legend(handles=handles, labels=labels, loc='center', bbox_to_anchor=(0.5, -0.2), ncol=3)
        # Add horizontal reference lines
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Format plots
        ax1.set_ylabel("Load Distribution (kN/m)")
        ax1.set_title("Load Distribution")
        ax1.grid(True, alpha=0.3, which='both')
        ax1.legend(loc='upper left')

        ax2.set_ylabel("Shear Force (kN)")
        ax2.set_title("Shear Force Diagram")
        ax2.grid(True, alpha=0.3, which='both')
        ax2.legend()

        ax3.set_xlabel("Fuselage Station (m)", labelpad=0.05)
        ax3.set_ylabel("Bending Moment (MN·m)")
        ax3.set_title("Bending Moment Diagram")
        ax3.grid(True, alpha=0.3, which='both')
        ax3.legend()
          # Maximum values calculations
        max_shear_idx = np.argmax(np.abs(self.shear))
        max_moment_idx = np.argmax(np.abs(self.moment))

        plt.tight_layout(h_pad=4)
        plt.subplots_adjust(bottom=0.05)
        plt.subplots_adjust(top=0.95)
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
