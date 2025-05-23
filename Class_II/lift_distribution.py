import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import json
import numpy as np
import matplotlib.pyplot as plt
from math import radians, tan, degrees
from utils import Data

def load_wing_parameters():
    data = Data("design3.json")
    return data.data

def plot_wing_planform():
    # Load data from JSON
    data = load_wing_parameters()
    
    # Extract relevant parameters from wing_design
    wing_design = data['outputs']['wing_design']
    S = wing_design['S']  # wing area
    b = wing_design['b']  # wingspan
    MAC = wing_design['MAC']  # mean aerodynamic chord
    sweep_c4 = radians(wing_design['sweep_c_4'])  # quarter chord sweep angle
    sweep_xc = radians(wing_design['sweep_x_c'])  # sweep at x/c
    cr = wing_design['chord_root']  # root chord
    ct = wing_design['chord_tip']  # tip chord
    taper_ratio = wing_design['taper_ratio']  # taper ratio
    X_LE = wing_design['X_LE']  # leading edge position
    
    # Create wing planform coordinates
    y = np.array([0, b/2, b/2, 0, 0])
    x_le = np.array([X_LE, X_LE + tan(sweep_xc) * (b/2), X_LE + tan(sweep_xc) * (b/2), X_LE, X_LE])
    x_te = x_le + np.array([cr, ct, ct, cr, cr])
    
    # Plot the wing
    plt.figure(figsize=(10, 8))
    plt.plot(x_le, y, 'b-', linewidth=2, label='Leading edge')
    plt.plot(x_te, y, 'b-', linewidth=2, label='Trailing edge')
    plt.plot(-x_le + 2*X_LE, y, 'b-', linewidth=2)  # Mirror for other half
    plt.plot(-x_te + 2*X_LE, y, 'b-', linewidth=2)  # Mirror for other half
    
    # Add annotations
    plt.annotate(f'Wingspan (b) = {b:.2f} m', xy=(X_LE, b/2), xytext=(X_LE + cr, b/2),
                arrowprops=dict(facecolor='black', shrink=0.05))
    plt.annotate(f'Root chord = {cr:.2f} m', xy=(X_LE, 0), xytext=(X_LE, -b/8),
                arrowprops=dict(facecolor='black', shrink=0.05))
    plt.annotate(f'Tip chord = {ct:.2f} m', xy=(X_LE + tan(sweep_xc) * (b/2), b/2),
                xytext=(X_LE + tan(sweep_xc) * (b/2) - cr/2, b/2 + b/8),
                arrowprops=dict(facecolor='black', shrink=0.05))
    plt.annotate(f'Sweep = {degrees(sweep_xc):.1f}°', xy=(X_LE + cr/2, b/4),
                xytext=(X_LE + cr/2, b/3))
    
    plt.title('Wing Planform')
    plt.xlabel('X coordinate [m]')
    plt.ylabel('Y coordinate [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    plot_wing_planform()

