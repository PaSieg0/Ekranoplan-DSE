import pandas as pd
import numpy as np
from utils import Data
import matplotlib.pyplot as plt
from ClassIWeightEstimation import ClassI, MissionType

def range_equation(type, W4_W5, eta_p, L_D, cp, cj, V, g):
    """
    Calculate the range of an aircraft based on the Breguet range equation.
    
    Returns:
        float: The range of the aircraft in meters.
    """
   
    if type == "PROP":
        R = (eta_p * L_D * np.log(W4_W5)) / (g * cp)

    elif type == "JET":
        R = (V * L_D * np.log(W4_W5)) / (g * cj)
    
    return R
    


       
if __name__ == "__main__":
    data = Data("design1.json")
    g = 9.81
    eta_p = data.data['prop_efficiency']
    L_D = 36.144597464183306 #TODO: Will be added to the json file for each design later
    cp = data.data['prop_consumption']
    cj = data.data['jet_consumption']
    V = data.data['cruise_speed']

    fuefracs_no_cruise = ClassI(aircraft_data=data, mission_type="DESIGN").fuel_fractions
    Mff_nocruise = 1
    for fraction in fuefracs_no_cruise.values():
        Mff_nocruise *= fraction

    Mff_harmonic = 1 - (data.data['design']['Fuel_used'] - (data.data['max_payload'] - data.data['design_payload'])*g) / data.data['design']['MTOW']
    Mff_design = 1 - (data.data['design']['Fuel_used'] / data.data['design']['MTOW'])
    Mff_maxrange = 1 - (data.data['design']['Fuel_max']) / (data.data['design']['MTOW'])
    Mff_ferry = 1 - (data.data['design']['Fuel_max']) / (data.data['design']['MTOW']- data.data['design_payload']*g)

    print(f"Design Mff (Harmonic): {Mff_harmonic}")
    print(f"Design Mff (Design): {Mff_design}")
    print(f"Design Mff (Max Range): {Mff_maxrange}")
    print(f"Design Mff (Ferry): {Mff_ferry}")

    W4_W5_harmonic = np.sqrt(1/Mff_harmonic * Mff_nocruise**2)
    W4_W5_design = np.sqrt(1/Mff_design * Mff_nocruise**2)
    W4_W5_maxrange = np.sqrt(1/Mff_maxrange * Mff_nocruise**2)
    W4_W5_ferry = np.sqrt(1/Mff_ferry * Mff_nocruise**2)

    range_harmonic = range_equation(data.data['aircraft_type'], W4_W5_harmonic, data.data['prop_efficiency'], L_D, cp, cj, V, g)
    range_design = range_equation(data.data['aircraft_type'], W4_W5_design, data.data['prop_efficiency'], L_D, cp, cj, V, g)
    range_max = range_equation(data.data['aircraft_type'], W4_W5_maxrange, data.data['prop_efficiency'], L_D, cp, cj, V, g)
    range_ferry = range_equation(data.data['aircraft_type'], W4_W5_ferry, data.data['prop_efficiency'], L_D, cp, cj, V, g)

    print(f"Range (Harmonic): {range_harmonic} m")
    print(f"Range (Design): {range_design} m")
    print(f"Range (Max): {range_max} m")
    print(f"Range (Ferry): {range_ferry} m")

    # Convert range values from meters to nautical miles (1 nautical mile = 1852 meters)
    range_harmonic_nm = range_harmonic / 1852
    range_design_nm = range_design / 1852
    range_max_nm = range_max / 1852
    range_ferry_nm = range_ferry / 1852

    points = [(0, data.data['max_payload'] / 1000), 
              (range_harmonic_nm, data.data['max_payload'] / 1000), 
              (range_design_nm, data.data['design_payload'] / 1000),
              (range_max_nm, (data.data['design_payload'] - (data.data['design']['Fuel_max'] - data.data['design']['Fuel']) / g) / 1000), 
              (range_ferry_nm, 0)]
    
    # Extract x and y coordinates from points
    x_coords, y_coords = zip(*points)

    # Plot the points and connect them in order
    plt.figure()
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', label='Payload-Range Curve')
    
    # Highlight the design point
    plt.plot(range_design_nm, data.data['design_payload'] / 1000, color='lime', label='Design Point', marker='o', markersize=8)
    
    plt.xlabel('Range (nautical miles)')
    plt.ylabel('Payload (tonnes)')
    plt.title('Payload-Range Diagram')
    plt.legend(loc='lower left')
    plt.grid(True)
    plt.show()

    