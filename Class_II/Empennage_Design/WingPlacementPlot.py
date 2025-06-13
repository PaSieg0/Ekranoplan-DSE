import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from Class_II.Empennage_Design.LoadingDiagram import LoadingDiagram
from Class_II.Empennage_Design.Scissor_plot import Tail_area
from utils import Data


def plot_wing_placement(loading_diagram: LoadingDiagram, PLOT: bool=True) -> None:
    mins = []
    maxs = []
    l_fus = loading_diagram.aircraft_data.data["outputs"]["fuselage_dimensions"]["l_fuselage"]
    placement_points = np.asarray(range(0, int(l_fus), 1))
    for place in placement_points:
        loading_diagram.X_LEMAC = place
        min_cg, max_cg = loading_diagram.determine_range()
        mins.append(min_cg)
        maxs.append(max_cg)

    if not PLOT:
        return mins, maxs
    fig, ax = plt.subplots()
    # Only show the horizontal line between the min and max CG for the selected value
    min_line, = ax.plot(mins, placement_points/l_fus, label='Min CG')
    max_line, = ax.plot(maxs, placement_points/l_fus, label='Max CG')
    ax.set_xlabel('CG (MAC)')
    ax.set_ylabel('Wing Placement (fraction of fuselage length)')
    ax.legend()

    # Add slider for horizontal line
    ax_slider = plt.axes([0.25, 0.02, 0.5, 0.03])
    slider = widgets.Slider(ax_slider, 'Wing Placement', 0, 1, valinit=0.5, valstep=0.01)
    # Initial horizontal line between min and max CG at the selected placement
    def get_cg_bounds(val):
        # Find the closest placement index for the slider value
        idx = (np.abs((placement_points/l_fus) - val)).argmin()
        return mins[idx], maxs[idx], placement_points[idx]/l_fus

    min_cg, max_cg, y_val = get_cg_bounds(slider.val)
    hline, = ax.plot([min_cg, max_cg], [y_val, y_val], color='red', linestyle='--', label='Selected Placement')
    vline_min = ax.axvline(min_cg, color='purple', linestyle=':', ymax=y_val, label='Min CG at Placement')
    vline_max = ax.axvline(max_cg, color='brown', linestyle=':', ymax=y_val, label='Max CG at Placement')
    min_text = ax.text(min_cg, 0, f"{min_cg:.2f}", color='purple', va='bottom', ha='center', fontsize=9, backgroundcolor='white')
    max_text = ax.text(max_cg, 0, f"{max_cg:.2f}", color='brown', va='bottom', ha='center', fontsize=9, backgroundcolor='white')
    value_text = ax.text(max_cg, y_val, f"{slider.val:.2f}", color='red', va='bottom', ha='right', fontsize=10, backgroundcolor='white')

    def update(val):
        min_cg, max_cg, y_val = get_cg_bounds(val)
        hline.set_xdata([min_cg, max_cg])
        hline.set_ydata([y_val, y_val])
        vline_min.set_xdata([min_cg, min_cg])
        vline_min.set_ydata([0, y_val])
        vline_max.set_xdata([max_cg, max_cg])
        vline_max.set_ydata([0, y_val])
        min_text.set_x(min_cg)
        min_text.set_y(0)
        min_text.set_text(f"{min_cg:.2f}")
        max_text.set_x(max_cg)
        max_text.set_y(0)
        max_text.set_text(f"{max_cg:.2f}")
        value_text.set_x(max_cg)
        value_text.set_y(y_val)
        value_text.set_text(f"{val:.2f}")
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()
    return mins, maxs

def plot_wing_placement_basic(loading_diagram: LoadingDiagram) -> None:
    mins = []
    maxs = []
    l_fus = loading_diagram.aircraft_data.data["outputs"]["fuselage_dimensions"]["l_fuselage"]
    placement_points = np.asarray(np.arange(0, l_fus, 0.1))
    for place in placement_points:
        loading_diagram.X_LEMAC = place
        min_cg, max_cg = loading_diagram.determine_range()
        mins.append(min_cg)
        maxs.append(max_cg)
    fig, ax = plt.subplots()
    ax.plot(mins, placement_points/l_fus, label='Min CG', color='tab:blue')
    ax.plot(maxs, placement_points/l_fus, label='Max CG', color='tab:orange')
    ax.set_xlabel('CG (MAC)')
    ax.set_xlim(-0.2, 1)
    ax.set_ylim(0.1, 0.5)
    ax.set_ylabel('Wing Placement (fraction of fuselage length)')
    ax.legend()
    plt.show()

def plot_wing_placement_and_scissor(loading_diagram, tail, Xcg_range):
    # Get min/max CG envelopes from wing placement
    mins = []
    maxs = []
    l_fus = loading_diagram.aircraft_data.data["outputs"]["fuselage_dimensions"]["l_fuselage"]
    placement_points = np.asarray(np.arange(0, l_fus, 0.1))
    for place in placement_points:
        loading_diagram.X_LEMAC = place
        min_cg, max_cg = loading_diagram.determine_range()
        mins.append(min_cg)
        maxs.append(max_cg)
    # Get scissor plot curves
    Sh_S_stability_vals = [tail.Sh_S_stability(xcg) for xcg in Xcg_range]
    Sh_S_control_vals = [tail.Sh_S_controllability(xcg) for xcg in Xcg_range]
    X_cg_bar = tail.get_normalised_cg()
    X_cg_fwd_bar = X_cg_bar[0]
    X_cg_aft_bar = X_cg_bar[1]
    # Plot both on the same axes
    fig, ax = plt.subplots(figsize=(10, 6))
    # Wing placement envelopes
    ax.plot(mins, placement_points/l_fus, label='Min CG (Wing Placement)', color='orange', linewidth=2)
    ax.plot(maxs, placement_points/l_fus, label='Max CG (Wing Placement)', color='brown', linewidth=2)
    # Scissor plot curves (Sh/S vs Xcg)
    ax2 = ax.twinx()
    ax2.plot(Xcg_range, Sh_S_stability_vals, label='Sh/S (Stability)', color='red', linewidth=2)
    ax2.plot(Xcg_range, Sh_S_control_vals, label='Sh/S (Controllability)', color='blue', linestyle='--', linewidth=2)
    # Add vertical lines for CG range
    ax2.axvline(x=X_cg_fwd_bar, color='green', linestyle='-.', linewidth=1.5, label='X_cg (min)')
    ax2.axvline(x=X_cg_aft_bar, color='purple', linestyle='-.', linewidth=1.5, label='X_cg (max)')
    # Axis labels
    ax.set_xlabel('CG (MAC) / Xcg')
    ax.set_ylabel('Wing Placement (fraction of fuselage length)')
    ax2.set_ylabel('Sh/S')
    # Legends
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    ax.set_xlim(-0.2, 1)
    ax2.set_ylim(0, 1)
    ax.set_ylim(0.1, 0.5)
    ax.grid(True)
    plt.title('Wing Placement Envelope and Scissor Plot Overlap')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    slider_val = 0.322
    loading_diagram = LoadingDiagram(aircraft_data=aircraft_data, wing_placement=slider_val)
    l_fus = loading_diagram.aircraft_data.data["outputs"]["fuselage_dimensions"]["l_fuselage"]
    X_LEMAC = slider_val * l_fus
    loading_diagram.X_LEMAC = X_LEMAC
    min_cg, max_cg = loading_diagram.determine_range()
    print(f"Slider value: {slider_val}")
    print(f"Wing placement (X_LEMAC): {X_LEMAC:.3f} m")
    print(f"Min CG (MAC): {min_cg:.4f}")
    print(f"Max CG (MAC): {max_cg:.4f}")
    # Optionally, still show the interactive plot
    # plot_wing_placement(loading_diagram)
    plot_wing_placement_basic(loading_diagram)
    tail_class = Tail_area(aircraft_data=loading_diagram.aircraft_data, fwd_cg=min_cg, aft_cg=max_cg)
    # plot_wing_placement_and_scissor(loading_diagram, tail_class, np.linspace(-0.5, 1.2, 200))