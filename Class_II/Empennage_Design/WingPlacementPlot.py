import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from Class_II.Empennage_Design.LoadingDiagram import LoadingDiagram
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

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    loading_diagram = LoadingDiagram(aircraft_data=aircraft_data)
    l_fus = loading_diagram.aircraft_data.data["outputs"]["fuselage_dimensions"]["l_fuselage"]
    slider_val = 0.3346999999999851
    X_LEMAC = slider_val * l_fus
    loading_diagram.X_LEMAC = X_LEMAC
    min_cg, max_cg = loading_diagram.determine_range()
    print(f"Slider value: {slider_val}")
    print(f"Wing placement (X_LEMAC): {X_LEMAC:.3f} m")
    print(f"Min CG (MAC): {min_cg:.4f}")
    print(f"Max CG (MAC): {max_cg:.4f}")
    # Optionally, still show the interactive plot
    plot_wing_placement(loading_diagram)