import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_II.LoadingDiagram import LoadingDiagram
from utils import Data


def plot_wing_placement(loading_diagram: LoadingDiagram) -> None:
    mins = []
    maxs = []
    l_fus = loading_diagram.aircraft_data.data["outputs"]["general"]["l_fuselage"]
    placement_points = np.asarray(range(0, int(l_fus), 1))
    for place in placement_points:
        loading_diagram.X_LEMAC = place
        min_cg, max_cg = loading_diagram.determine_range()
        mins.append(min_cg)
        maxs.append(max_cg)
    plt.plot(mins, placement_points/l_fus, label='Min CG')
    plt.plot(maxs, placement_points/l_fus, label='Max CG')
    plt.show()

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    loading_diagram = LoadingDiagram(aircraft_data=aircraft_data)
    plot_wing_placement(loading_diagram)