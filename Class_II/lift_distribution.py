import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import json
import numpy as np
import matplotlib.pyplot as plt
from math import radians, tan, degrees
from utils import Data
from Class_I.PrelimWingPlanformDesign import WingPlanform

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
    
    # Create figure with subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    
    # Plot wing planform on top subplot
    x_points, chord_distribution = wing.plot_wing(ax=ax1)
    
    # Get wing parameters
    b = wing.b  # wingspan
    S = wing.S  # wing area
    MTOW = data.data['outputs']['max']['MTOW']  # maximum takeoff weight
    cr = wing.chord_root  # root chord
    X_LE = wing.X_LE  # leading edge position    
    V_cruise = data.data['requirements']['cruise_speed']  # cruise speed
    V_stall = data.data['requirements']['stall_speed_takeoff']  # stall speed

    Factor = MTOW / S
    Cl = Factor / (0.5 * 1.225 * (V_stall**2))
    print(f"Cl: {Cl:.2f}")
    print(f"Factor (W/S): {Factor:.2f} N/m²")

    # Calculate elliptical chord distribution
    c_elliptical = calculate_elliptical_chord(x_points, b, S)
    
    # Plot wing planform and elliptical distribution (top subplot)
    y_le = np.full_like(x_points, 0)  # Leading edge at y=0
    y_te = c_elliptical  # Trailing edge follows elliptical distribution
    ax1.plot(x_points, y_le, 'r--', linewidth=2, label='Elliptical LE')
    ax1.plot(x_points, y_te, 'r--', linewidth=2, label='Elliptical TE')
      # Print array details
    print(f"Length of elliptical chord array: {len(x_points)} points")
    print(f"First x-coordinate (root): {x_points[0]:.1f} m")
    print(f"Last x-coordinate (tip): {x_points[-1]:.1f} m")
    
    # Plot chord distributions (bottom subplot)
    ax2.plot(x_points, chord_distribution, 'b-', label='Actual chord distribution')
    ax2.plot(x_points, c_elliptical, 'r--', label='Elliptical chord distribution')
    
    # Calculate and plot average line
    average_chord = (chord_distribution + c_elliptical) / 2
    ax2.plot(x_points, average_chord, 'g-.', label='Average distribution', linewidth=1.5)
    
    # Calculate areas using trapezoidal integration
    actual_half_area = np.trapezoid(chord_distribution, x_points)
    elliptical_half_area = np.trapezoid(c_elliptical, x_points)
    
    # Double the areas to get full wing area (both sides)
    actual_area = 2 * actual_half_area
    elliptical_area = 2 * elliptical_half_area

    # Print the results
    print(f"\nArea calculations:")
    print(f"Actual wing area: {actual_area:.2f} m²")
    print(f"Elliptical wing area: {elliptical_area:.2f} m²")
    print(f"Area difference: {abs(actual_area - elliptical_area):.2f} m² ({abs(actual_area - elliptical_area)/actual_area*100:.1f}%)")
    
    # Add area information to the plot
    ax2.text(0.02, 0.98, f"Actual wing area: {actual_area:.2f} m²\nElliptical area: {elliptical_area:.2f} m²", 
             transform=ax2.transAxes, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Calculate lift distributions
    lift_per_unit_span = chord_distribution * Factor  # N/m
    lift_per_unit_span_elliptical = c_elliptical * Factor  # N/m
    lift_per_unit_span_average = average_chord * Factor  # N/m

    # Plot lift distributions (third subplot)
    ax3.plot(x_points, lift_per_unit_span, 'b-', label='Actual lift distribution')
    ax3.plot(x_points, lift_per_unit_span_elliptical, 'r--', label='Elliptical lift distribution')
    ax3.plot(x_points, lift_per_unit_span_average, 'g-.', label='Average lift distribution', linewidth=1.5)
    
    ax3.set_title("Lift Distribution")
    ax3.set_xlabel("Spanwise Position (m)")
    ax3.set_ylabel("Lift per unit span (N/m)")
    ax3.grid(True)
    ax3.legend()

    # Calculate total lift by integrating the distribution
    total_lift_actual = 2 * np.trapz(lift_per_unit_span, x_points)  # N
    total_lift_elliptical = 2 * np.trapz(lift_per_unit_span_elliptical, x_points)  # N
    
    # Add lift information to the plot
    ax3.text(0.02, 0.98, f"Total lift (actual): {total_lift_actual/1000:.2f} kN\nTotal lift (elliptical): {total_lift_elliptical/1000:.2f} kN", 
             transform=ax3.transAxes, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Customize plots
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_title("Wing Planform with Elliptical Distribution - Design 3")
    ax1.set_xlabel("Spanwise Direction (m)")
    ax1.set_ylabel("Chordwise Direction (m)")
    ax1.grid(True)
    ax1.legend()

    ax2.set_title("Chord Length Distribution")
    ax2.set_xlabel("Spanwise Position (m)")
    ax2.set_ylabel("Chord Length (m)")
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    plot_wing_planform()

