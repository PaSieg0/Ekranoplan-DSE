import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def compute_x_cg(
    x_cg_components,  # Dictionary of components and their x/cg values
    weight_components  # Dictionary of components and their weights
):
    total_weight = sum(weight_components.values())  # Calculate total weight as the sum of all component weights
    x_cg = sum(x_cg_components[comp] * weight_components[comp] / total_weight for comp in x_cg_components)
    return x_cg, total_weight


if __name__ == "__main__":
    aircraft_data = Data("design1.json")
    g = 9.81  # Acceleration due to gravity in m/s^2

    x_cg_components = {"OEW": 0.25, "Payload": 0.25, "Fuel": 0.5}
    mass_components = {"OEW": aircraft_data.data["design"]["OEW"]/g, 
                         "Payload": aircraft_data.data["design"]["design_payload"], 
                         "Fuel": aircraft_data.data["design"]["Fuel_used"]/g}
    
    x_cg_OEW, mass_OEW = x_cg_components["OEW"], mass_components["OEW"]
    x_cg_OEWpayload, mass_OEWpayload = compute_x_cg(
        {key: x_cg_components[key] for key in ["OEW", "Payload"]}, 
        {key: mass_components[key] for key in ["OEW", "Payload"]}
    )
    x_cg_OEWfuel, mass_OEWfuel = compute_x_cg(
        {key: x_cg_components[key] for key in ["OEW", "Fuel"]}, 
        {key: mass_components[key] for key in ["OEW", "Fuel"]}
    )
    x_cg_total, mass_total = compute_x_cg(
        x_cg_components, 
        mass_components
    )
    
    x_cgs = [x_cg_OEW, x_cg_OEWpayload, x_cg_OEWfuel, x_cg_total]
    weights = [mass_OEW, mass_OEWpayload, mass_OEWfuel, mass_total]

    # Scatter plot of CG positions with weight on y-axis and x/cg on x-axis
    labels = ["OEW", "OEW + Payload", "OEW + Fuel", "Total"]
    plt.figure(figsize=(10, 6))
    for i, label in enumerate(labels):
        plt.scatter(x_cgs[i], weights[i]/1000, label=label, s=100)  # s controls the size of the points
        
    plt.plot(x_cgs[0:2], [weights[i]/1000 for i in range(2)], linestyle='-', color='black', alpha=0.7)  # Connect 0 to 1
    plt.plot(x_cgs[1:4:2], [weights[i]/1000 for i in [1, 3]], linestyle='-', color='black', alpha=0.7)  # Connect 1 to 3
    plt.plot(x_cgs[0:3:2], [weights[i]/1000 for i in [0, 2]], linestyle='-', color='black', alpha=0.7)  # Connect 0 to 2
    plt.plot(x_cgs[2:4], [weights[i]/1000 for i in range(2, 4)], linestyle='-', color='black', alpha=0.7)  # Connect 2 to 3

    plt.xlabel("x/cg Position")
    plt.ylabel("Weight (tonnes)")
    plt.title("Center of Gravity Positions vs Weight for Different Configurations")
    plt.legend(loc='upper left')  # Move legend to the upper left
    plt.grid(linestyle='--', alpha=0.7)
    plt.show()