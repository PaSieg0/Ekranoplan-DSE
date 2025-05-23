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
    return (4*S/(np.pi*b)) * np.sqrt(1 - (2*y/b)**2)

def plot_wing_planform():
    # Load data and create wing planform object
    data = Data("design3.json")
    wing = WingPlanform(data)
    wing.calculate()
    
    # Create figure and plot
    fig, ax = plt.subplots(figsize=(12, 8))
    wing.plot_wing(ax=ax)
    
    # Get wing parameters
    b = wing.b  # wingspan
    S = wing.S  # wing area
    cr = wing.chord_root  # root chord
    X_LE = wing.X_LE  # leading edge position    # Calculate and plot elliptical chord distribution with same resolution as wing planform (0.1m spacing)
    x_points = np.arange(-b/2, b/2 + 0.1, 0.1)  # +0.1 to include the endpoint
    c_elliptical = calculate_elliptical_chord(abs(x_points), b, S)
    
    # Plot elliptical distribution
    y_le = np.full_like(x_points, 0)  # Leading edge at y=0
    y_te = c_elliptical  # Trailing edge follows elliptical distribution
    ax.plot(x_points, y_le, 'r--', linewidth=2, label='Elliptical LE')
    ax.plot(x_points, y_te, 'r--', linewidth=2, label='Elliptical TE')
      # Add annotation for elliptical chord
    x_mid = b/4
    c_mid = calculate_elliptical_chord(x_mid, b, S)
    ax.annotate(f'Elliptical chord at x={x_mid:.1f}m: {c_mid:.2f}m', 
                xy=(x_mid, c_mid/2),
                xytext=(x_mid + 2, c_mid/2 + 2),
                arrowprops=dict(facecolor='red', shrink=0.05))
    
    # Customize plot
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Wing Planform with Elliptical Chord Distribution - Design 3")
    ax.set_xlabel("Spanwise Direction (m)")
    ax.set_ylabel("Chordwise Direction (m)")
    ax.grid(True)
    ax.legend()
    plt.show()

if __name__ == "__main__":
    plot_wing_planform()

