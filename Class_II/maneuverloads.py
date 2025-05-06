import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import Iteration
from ClassIWeightEstimation import MissionType, AircraftType
from utils import Data

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

def plot_load_diagram(rho, CLmax, W, S, nmax, nmin, V_cruise, V_dive):
    V_range = np.arange(0, V_dive, 0.1)  # Define a range of velocities
    n_positive = [calculante_n_limits(rho, CLmax, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]  # Calculate positive load factor for each velocity
    n_negative = [calculante_n_limits(rho, -CLmax, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]  # Calculate negative load factor for each velocity
    plt.figure(figsize=(10, 6))  # Set a larger figure size for better readability

    # Plot positive and negative load factors
    plt.plot(V_range, n_positive, color='blue', label='Positive Load Factor')
    plt.plot(V_range, n_negative, color='orange', label='Negative Load Factor')

    # Highlight dive speed and cruise speed
    plt.axvline(x=V_dive, color='red', linestyle='-', label='Dive Speed', linewidth=1.5)
    plt.axvline(x=V_cruise, color='green', linestyle='-', label='Cruise Speed', linewidth=1.5)

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

    aircraft_type = AircraftType.PROP
    mission_type = MissionType.DESIGN
    cruise_speed = 225*0.51444
    jet_consumption = 19e-6
    prop_consumption = 90e-9
    prop_efficiency = 0.82
    Cd0 = 0.02
    e = 0.85
    A = 10
    tfo = 0.001
    reserve_fuel = 0
    k = 1
    n_engines = [4, 6, 8, 10]

    CLmax_clean=[1.5]
    CLmax_takeoff=[1.6, 1.8, 2.0, 2.2]
    CLmax_landing=[1.8, 1.9, 2.2]
    aspect_ratios=[A]
    stall_speed_clean=150*0.5144
    stall_speed_takeoff=120*0.5144
    stall_speed_landing=100*0.5144
    cruise_altitude=5
    high_altitude=10000*0.3048
    L=40
    r=3
    hull_surface=2*np.pi*L*r / 3
    rho_water=1000.0
    kinematic_viscosity=1.002e-6
    
    aircraft = Data("design1.json")

    final_MTOMS = []
    fuel_economy, MTOM_history, S_final = Iteration.iteration(
                        aircraft_type=aircraft_type,
                        mission_type=mission_type,
                        Range=2800*1.852*1000,
                        cruise_speed=cruise_speed,
                        jet_consumption=jet_consumption,
                        prop_consumption=prop_consumption,
                        prop_efficiency=prop_efficiency,
                        Cd0=Cd0,
                        e=e,
                        A=A,
                        tfo=tfo,
                        k=k,
                        n_engines=n_engines,
                        reserve_fuel=reserve_fuel,
                        CLmax_clean=CLmax_clean,
                        CLmax_takeoff=CLmax_takeoff,
                        CLmax_landing=CLmax_landing,
                        aspect_ratios=[A],
                        stall_speed_clean=stall_speed_clean,
                        stall_speed_takeoff=stall_speed_takeoff,
                        stall_speed_landing=stall_speed_landing,
                        cruise_altitude=cruise_altitude,
                        high_altitude=high_altitude,
                        hull_surface=hull_surface,
                        L=L,
                        rho_water=rho_water,
                        kinematic_viscosity=kinematic_viscosity
                        )

    
    # Example usage
    rho = 1.225  # kg/m^3 (air density at sea level)
    CLmax = 1.5  # Maximum lift coefficient
    W = MTOM_history[-1] * 9.81  # Weight in N
    S = S_final  # Wing area in m^2
    nmax = max_n(W)  # Maximum load factor
    nmin = min_n()  # Minimum load factor
    dive_speed = 300*0.51444  # Dive speed in m/s (example value)

    print(f"MTOM = {W/9.81/1000} kg", f"\nS = {S} m^2")

    plot_load_diagram(rho, CLmax, W, S, nmax, nmin, V_cruise=cruise_speed, V_dive=dive_speed)