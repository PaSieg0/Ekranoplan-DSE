import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import Iteration
from ClassIWeightEstimation import MissionType, AircraftType
from utils import Data
from ISA_Class import ISA

def calculate_n(rho, CL, W, S, V):
    L = CL*1/2*rho*V**2*S
    return L/W

def calculante_n_limits(rho, CLmax, W, S, V, nmax, nmin, V_cruise, V_dive):
    if V < V_cruise or CLmax > 0:
        n=calculate_n(rho, CLmax, W, S, V)
        if n > nmax:
            return nmax
        elif n < nmin:
            return nmin
        else:
            return n
    else:
        return nmin + (0 - nmin) * (V - V_cruise) / (V_dive - V_cruise)
    
def max_n(W):
    W = W / 4.44822  # Convert weight from newtons to pounds
    n = 2.1 + (24000 / (W + 10000))
    if n < 2.5:
        return 2.5
    elif n > 3.8:
        return 3.8
    else:
        return n
    
def min_n():
    return -1.0

def plot_load_diagram(aircraft_data):
    rho = ISA(0).rho  # kg/m^3 (air density at sea level)
    g = aircraft_data.data["gravitational_acceleration"]  # m/s^2 (acceleration due to gravity)
    CLmax_clean = aircraft_data.data["CLmax_clean"]  # Maximum lift coefficient
    CLmax_flapped = aircraft_data.data["CLmax_landing"]  # Maximum lift coefficient during landing
    W = aircraft_data.data["MTOM"] * g  # Weight in N
    S = aircraft_data.data["MTOM"] * g / aircraft_data.data["WS"]  # Wing area in m^2
    nmax = max_n(W)  # Maximum load factor
    nmin = min_n()  # Minimum load factor
    V_cruise = aircraft_data.data["cruise_speed"]  # Cruise speed in m/s
    V_dive = aircraft_data.data["cruise_speed"]/0.8  # Minimum dive speed as stipulated by CS25
    V_flapped = 80  # Flapped speed as stipulated by CS25

    rho, CLmax_clean, CLmax_flapped, W, S, nmax, nmin, V_cruise, V_dive, V_flapped
    V_range = np.arange(0, V_dive, 0.1)  # Define a range of velocities
    n_positive = [calculante_n_limits(rho, CLmax_clean, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]  # Calculate positive load factor for each velocity
    n_negative = [calculante_n_limits(rho, -CLmax_clean, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]  # Calculate negative load factor for each velocity

    # V_range_flapped = np.arange(0, V_flapped, 0.1)  # Define a range of velocities
    # n_positive_flapped = [calculante_n_limits(rho, CLmax_flapped, W, S, V, nmax, nmin, V_cruise, V_flapped) for V in V_range_flapped]  # Calculate positive load factor for each velocity
    plt.figure(figsize=(10, 6))  # Set a larger figure size for better readability

    # Plot positive and negative load factors
    plt.plot(V_range, n_positive, color='blue', label='Positive Load Factor')
    plt.plot(V_range, n_negative, color='orange', label='Negative Load Factor')

    # plt.plot(V_range_flapped, n_positive_flapped, color='purple', label='Flapped Load Factor')

    # Highlight dive speed and cruise speed
    plt.plot([V_dive, V_dive], [0, nmax], color='red', linestyle='-', label='Dive Speed', linewidth=1.5)
    plt.axvline(x=V_cruise, color='green', linestyle='--', label='Cruise Speed', linewidth=1.5)
    # Find the velocity where n_positive is equal to 1
    for i, n in enumerate(n_positive):
        if n >= 1:
            V_at_n1 = V_range[i]
            plt.axvline(x=V_at_n1, color='magenta', linestyle='--', label='Stall Speed', linewidth=1.5)
            print(f"Stall Speed (V at n=1): {V_at_n1:.2f} m/s")
            break

    # Add horizontal lines for key load factors
    plt.axhline(y=0, color='black', linestyle='--', label='Zero Load Factor', linewidth=1)
    # plt.axhline(y=1, color='cyan', linestyle='--', label='Load Factor = 1', linewidth=1)
    # plt.axhline(y=nmax, color='purple', linestyle='--', label=f'Max Load Factor (nmax = {nmax:.2f})', linewidth=1)
    # plt.axhline(y=nmin, color='brown', linestyle='--', label=f'Min Load Factor (nmin = {nmin:.2f})', linewidth=1)

    # Add labels, title, and legend
    plt.xlabel('Velocity (V) [m/s]', fontsize=12)
    plt.ylabel('Load Factor (n)', fontsize=12)
    plt.title('Load Diagram', fontsize=14, fontweight='bold')
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)

    # Show the plot
    plt.tight_layout()  # Adjust layout to prevent overlap
    plt.show()

if __name__ == "__main__":
    
    aircraft_data = Data("design1.json")

    plot_load_diagram(aircraft_data)