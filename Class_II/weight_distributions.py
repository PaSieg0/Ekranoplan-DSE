import sys
import os
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data 

data = Data("design3.json")

#TODO: link to json file
component_masses = {
    "air_conditioning_mass": 0,
    "handling_gear_mass": 78,
    "hydraulics_mass": 120,
    "starter_pneumatic_mass": 148,
    "flight_control_mass": 191,
    "engine_controls_mass": 212,
    "fuel_system_mass": 240,
    "anchor_mass": 250,
    "instruments_mass": 398,
    "anti_ice_mass": 521,
    "apu_installed_mass": 699,
    "electrical_mass": 846,
    "avionics_mass": 971,
    "floater_mass": 1907,
    "horizontal_tail_mass": 2917,
    "door_mass": 3100,
    "vertical_tail_mass": 3273,#
    "military_cargo_handling_system_mass": 3662,
    "nacelle_group_mass": 4827,
    "furnishings_mass": 8148,
    "engine_mass": 19726,#
    "wing_mass": 32174,#
    "fuselage_mass": 38980#
}


def calculate_cg(data, component_masses, nacelle_length=0.0, plot=False):
    """
    Calculate the center of gravity (CG) of the aircraft based on component masses and their positions.
    """
    # FOR NOW ONLY IN THE X DIRECTION, LATER WE CAN ADD Y AND Z
    x_LEMAC_wing = data.data['outputs']['wing_design']['X_LE']
    MAC_wing = data.data['outputs']['wing_design']['MAC']
    cargo_length = data.data['outputs']['general']['cargo_length']
    cargo_x_start = data.data['outputs']['general']['cargo_distance_from_nose']
    fuel_mass = data.data['outputs']['design']['max_fuel']  # Fuel mass
    cargo_mass = data.data['requirements']['design_payload'] # Cargo mass
    nose_length = data.data['outputs']['general']['l_nose']  # Nose length
    fus_straight_length = data.data['outputs']['general']['l_fus_straight']  # Straight fuselage length
    fus_afterbody_length = data.data['outputs']['general']['l_afterbody']  # Afterbody length
    fus_tailcone_length = data.data['outputs']['general']['l_tailcone']  # Tail length 
    x_LE_vertical_tail = data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']  # Leading edge of the vertical tail
    x_MAC_vertical_tail = data.data['outputs']['empennage_design']['vertical_tail']['MAC']  # Mean Aerodynamic Chord of the vertical tail
    x_LE_horizontal_tail = data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos']  # Leading edge of the horizontal tail
    x_MAC_horizontal_tail = data.data['outputs']['empennage_design']['horizontal_tail']['MAC']  # Mean Aerodynamic Chord of the horizontal tail

    cg_x = 0.0
    total_mass = sum(list(component_masses.values()) + [fuel_mass, cargo_mass])
    total_fuselage_length = nose_length + fus_straight_length + fus_afterbody_length + fus_tailcone_length

    # Calculate the CG position based on component masses and their x-coordinates
    for component, mass in component_masses.items():
        if component == "fuselage_mass":
            x_position = total_fuselage_length * 0.45  # Assuming fuselage CG is slightly more forward because of the taper
        elif component == "wing_mass" or component == "floater_mass":
            x_position = x_LEMAC_wing + MAC_wing / 2
        elif component == "engine_mass" or component == "nacelle_group_mass":
            x_position = x_LEMAC_wing - nacelle_length/2  # Assuming engine position matches wing
        elif component == "horizontal_tail_mass":
            x_position = x_LE_horizontal_tail + x_MAC_horizontal_tail / 2
        elif component == "vertical_tail_mass":
            x_position = x_LE_vertical_tail + x_MAC_vertical_tail / 2
        elif component == "door_mass" or component == "flight_control_mass":
            x_position = nose_length
        else:
            x_position = total_fuselage_length / 2  # Default position at fuselage midpoint
        
        cg_x += mass * x_position
    # Add fuel and cargo contributions
    cg_x += cargo_mass * (cargo_x_start + cargo_length / 2)  # Fuel CG at the center of the cargo area
    cg_x += fuel_mass * (x_LEMAC_wing + MAC_wing/2)  # Fuel CG at the center of the fuselage

    if plot:
        import matplotlib.pyplot as plt

        # Prepare lists for plotting
        x_positions = []
        masses = []
        labels = []

        for component, mass in component_masses.items():
            if component == "fuselage_mass":
                x_position = total_fuselage_length * 0.45
            elif component == "wing_mass" or component == "floater_mass":
                x_position = x_LEMAC_wing + MAC_wing / 2
            elif component == "engine_mass" or component == "nacelle_group_mass":
                x_position = x_LEMAC_wing - nacelle_length/2
            elif component == "horizontal_tail_mass":
                x_position = x_LE_horizontal_tail + x_MAC_horizontal_tail / 2
            elif component == "vertical_tail_mass":
                x_position = x_LE_vertical_tail + x_MAC_vertical_tail / 2
            elif component == "door_mass" or component == "flight_control_mass":
                x_position = nose_length
            else:
                x_position = total_fuselage_length / 2
            x_positions.append(x_position)
            masses.append(mass)
            labels.append(component)

        # Add fuel and cargo
        x_positions.append(cargo_x_start + cargo_length / 2)
        masses.append(cargo_mass)
        labels.append("cargo_mass")
        x_positions.append(x_LEMAC_wing + MAC_wing/2)
        masses.append(fuel_mass)
        labels.append("fuel_mass")

        # Plot
        plt.figure(figsize=(12, 3))
        plt.scatter(x_positions, [1]*len(x_positions), s=100, c='b', alpha=0.6, label='Components')
        for i, label in enumerate(labels):
            plt.text(x_positions[i], 1.02, label, rotation=90, va='bottom', ha='center', fontsize=8)
        
        # Add fuselage section markers
        sections = [
            (0, "Nose", nose_length),
            (nose_length, "Straight Section", nose_length + fus_straight_length),
            (nose_length + fus_straight_length, "Afterbody", nose_length + fus_straight_length + fus_afterbody_length),
            (nose_length + fus_straight_length + fus_afterbody_length, "Tailcone", total_fuselage_length)
        ]
        
        for start, label, end in sections:
            plt.axvline(x=start, color='g', linestyle=':', alpha=0.5)
            plt.text(start, 0.95, f"{label}\n({start:.1f}m)", ha='right', va='top', rotation=90)
        
        plt.axvline(cg_x / total_mass, color='r', linestyle='--', label='CG')
        plt.xlabel("Fuselage X Position (m)")
        plt.yticks([])
        plt.title("Component Positions and Center of Gravity")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    return cg_x / total_mass 

def output_load_distributions(data, component_masses, nacelle_length=0.0):
    """
    Output the load distributions for the aircraft components.
    If point load, then stays the same as the calculate_cg above. 
    If distributed load such as with the fuselage weight, wing weight, etc., 
    then the load distribution is calculated based on the component's length and mass.
    """
    # FOR NOW ONLY IN THE X DIRECTION, LATER WE CAN ADD Y AND Z
    x_LEMAC_wing = data.data['outputs']['wing_design']['X_LE']
    MAC_wing = data.data['outputs']['wing_design']['MAC']
    cargo_length = data.data['outputs']['general']['cargo_length']
    cargo_x_start = data.data['outputs']['general']['cargo_distance_from_nose']
    fuel_mass = data.data['outputs']['design']['max_fuel']  # Fuel mass
    cargo_mass = data.data['requirements']['design_payload'] # Cargo mass
    nose_length = data.data['outputs']['general']['l_nose']  # Nose length
    fus_straight_length = data.data['outputs']['general']['l_fus_straight']  # Straight fuselage length
    fus_afterbody_length = data.data['outputs']['general']['l_afterbody']  # Afterbody length
    fus_tailcone_length = data.data['outputs']['general']['l_tailcone']  # Tail length 
    x_LE_vertical_tail = data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']  # Leading edge of the vertical tail
    x_MAC_vertical_tail = data.data['outputs']['empennage_design']['vertical_tail']['MAC']  # Mean Aerodynamic Chord of the vertical tail
    x_LE_horizontal_tail = data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos']  # Leading edge of the horizontal tail
    x_MAC_horizontal_tail = data.data['outputs']['empennage_design']['horizontal_tail']['MAC']  # Mean Aerodynamic Chord of the horizontal tail

calculate_cg(data, component_masses, nacelle_length=0, plot=False)  # Calculate CG to ensure it is correct