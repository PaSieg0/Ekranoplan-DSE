import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import json
import numpy as np
import matplotlib.pyplot as plt
from math import radians, tan, degrees
from utils import Data
from Class_I.PrelimWingPlanformDesign import WingPlanform
from joinedloaddiagrams import plot_complete_load_diagram

def calculate_elliptical_chord(y, b, S):
    """Calculate elliptical chord distribution using the formula C(y) = (4S/πb)√(1-(2y/b)²)"""
    # Ensure y doesn't exceed b/2 (add small tolerance to avoid numerical issues)
    eps = 1e-10
    y = np.minimum(abs(y), b/2 - eps)
    return (4*S/(np.pi*b)) * np.sqrt(1 - (2*y/b)**2)

def plot_wing_planform():
    # Load data and create wing planform object
    data = Data("design3.json")
    wing = WingPlanform(data)
    wing.calculate()
    
    # Create figure with single subplot
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Get wing planform data without plotting
    x_points = wing.get_half_span_points()
    chord_distribution = []
    for x in x_points:
        # Linear interpolation between root and tip chord
        chord_distribution.append(wing.chord_root + (wing.chord_tip - wing.chord_root) * (x / (wing.b/2)))
    chord_distribution = np.array(chord_distribution)
    
    # Get wing parameters
    b = wing.b  # wingspan
    S = wing.S  # wing area
    MTOW = data.data['outputs']['max']['MTOW']  # maximum takeoff weight
    V_stall = data.data['requirements']['stall_speed_takeoff']  # stall speed

    Factor = MTOW / S
    Cl = Factor / (0.5 * 1.225 * (V_stall**2))
    print(f"Cl: {Cl:.2f}")
    print(f"Factor (W/S): {Factor:.2f} N/m²")

    # Calculate elliptical chord distribution
    c_elliptical = calculate_elliptical_chord(x_points, b, S)
    
    # Print array details
    print(f"Length of elliptical chord array: {len(x_points)} points")
    print(f"First x-coordinate (root): {x_points[0]:.1f} m")
    print(f"Last x-coordinate (tip): {x_points[-1]:.1f} m")
    
    # Calculate average chord
    average_chord = (chord_distribution + c_elliptical) / 2
    
    # Get load factors
    n_max, n_min = plot_complete_load_diagram(data, h=0, plot=False)  # Get load factors without plotting
    print(f"\nLoad factors: n_max = {n_max:.2f}, n_min = {n_min:.2f}")
      # Calculate lift distributions
    lift_per_unit_span = chord_distribution * Factor  # N/m
    lift_per_unit_span_elliptical = c_elliptical * Factor  # N/m
    lift_per_unit_span_average = average_chord * Factor  # N/m

    # Plot lift distributions
    ax.plot(x_points, lift_per_unit_span, 'b-', label='Actual lift distribution')
    ax.plot(x_points, lift_per_unit_span_elliptical, 'r--', label='Elliptical lift distribution')
    ax.plot(x_points, lift_per_unit_span_average, 'g-.', label='Average distribution', linewidth=1.5)

    # Calculate total lift by integrating the distribution
    total_lift_actual = 2 * np.trapz(lift_per_unit_span, x_points)  # N
    total_lift_elliptical = 2 * np.trapz(lift_per_unit_span_elliptical, x_points)  # N
    
    # Add lift information to the plot
    ax.text(0.02, 0.98, f"Total lift (actual): {total_lift_actual/1000:.2f} kN\nTotal lift (elliptical): {total_lift_elliptical/1000:.2f} kN", 
            transform=ax.transAxes, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Customize plot
    ax.set_title("Wing Lift Distribution")
    ax.set_xlabel("Spanwise Position (m)")
    ax.set_ylabel("Lift per unit span (N/m)")
    ax.grid(True)
    ax.legend()
    
    plt.tight_layout()
    plt.show()
    return lift_per_unit_span_average



if __name__ == "__main__":
    plot_wing_planform()

