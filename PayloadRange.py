import os
from enum import Enum, auto
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
    g = 9.80665
    eta_p = data.data['prop_efficiency']
    L_D = 15.6 #TODO: Will be added to the json file for each design later
    cp = data.data['prop_consumption']
    cj = data.data['jet_consumption']
    V = data.data['cruise_speed']

    W4_W5_harmonic = data.data['design']['MTOW'] / (data.data['design']['ZFW'] + (data.data['max_payload'] - data.data['design_payload'])*g)
    W4_W5_design = data.data['design']['MTOW'] / (data.data['design']['ZFW'])
    W4_W5_maxrange = (data.data['design']['MTOW']) / (data.data['design']['ZFW']- (data.data['design']['Fuel_max'] - data.data['design']['Fuel']))
    W4_W5_ferry = (data.data['design']['MTOW']- data.data['design_payload']*g) / (data.data['design']['MTOW']- data.data['design_payload']*g - data.data['design']['Fuel'])

    range_harmonic = range_equation(data.data['aircraft_type'], W4_W5_harmonic, data.data['prop_efficiency'], L_D, cp, cj, V, g)
    range_design = range_equation(data.data['aircraft_type'], W4_W5_design, data.data['prop_efficiency'], L_D, cp, cj, V, g)
    range_max = range_equation(data.data['aircraft_type'], W4_W5_maxrange, data.data['prop_efficiency'], L_D, cp, cj, V, g)
    range_ferry = range_equation(data.data['aircraft_type'], W4_W5_ferry, data.data['prop_efficiency'], L_D, cp, cj, V, g)

    print(f"Range (Harmonic): {range_harmonic} m")
    print(f"Range (Design): {range_design} m")
    print(f"Range (Ferry): {range_ferry} m")

    points = [(0, data.data['max_payload']), 
              (range_harmonic, data.data['max_payload']), 
              (range_design, data.data['design_payload']),
              (range_max, data.data['design_payload'] - (data.data['design']['Fuel_max'] - data.data['design']['Fuel'])/g), 
              (range_ferry, 0)]
    
    # Extract x and y coordinates from points
    x_coords, y_coords = zip(*points)

    # Plot the points and connect them in order
    plt.figure()
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', label='Payload-Range Curve')
    
    # Highlight the design point
    plt.plot(range_design, data.data['design_payload'], color='lime', label='Design Point', marker='o', markersize=8)
    
    plt.xlabel('Range (m)')
    plt.ylabel('Payload (kg)')
    plt.title('Payload-Range Diagram')
    plt.legend(loc='lower left')
    plt.grid(True)
    plt.show()

    