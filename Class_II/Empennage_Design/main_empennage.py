import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from Class_II.Empennage_Design.LoadingDiagram import LoadingDiagram
from Class_II.Empennage_Design.Scissor_plot import Tail_area
from Class_II.Empennage_Design.WingPlacementPlot import plot_wing_placement
from utils import Data

def plot(aircraft_data: Data) -> float:
    diffs = []
    l_fus = aircraft_data.data["outputs"]["general"]["l_fuselage"]
    placements = np.arange(0.2, 0.5, 0.0001)
    loading_diagram = LoadingDiagram(aircraft_data=aircraft_data)
    # Use tqdm for a progress bar
    for wing_placement in tqdm(placements, desc="Wing Placement Sweep", unit="placement"):
        wing_placement = wing_placement * l_fus
        loading_diagram.X_LEMAC = wing_placement
        mins, maxs = loading_diagram.determine_range()
        tail_area = Tail_area(aircraft_data=aircraft_data, fwd_cg=mins, aft_cg=maxs)
        tail_stab, tail_cont = tail_area.get_tail_area()
        diffs.append((wing_placement/l_fus, abs(tail_stab - tail_cont))) if tail_stab > 0 and tail_cont > 0 else diffs.append((wing_placement/l_fus, 10000))


    idx = np.argmin([d[1] if isinstance(d, tuple) else d for d in diffs])
    best_placement, _ = diffs[idx] if isinstance(diffs[idx], tuple) else (None, diffs[idx])
    print(best_placement)
    return best_placement

if __name__ == "__main__":
    design_file = "design3.json"
    aircraft_data = Data(design_file)
    best_placement = plot(aircraft_data=aircraft_data)
    print(f"Best wing placement for tail area: {best_placement}")
