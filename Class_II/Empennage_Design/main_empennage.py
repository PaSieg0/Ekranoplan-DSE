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

class EmpennageOptimizer:
    def __init__(self, aircraft_data):
        self.aircraft_data = aircraft_data
        self.l_fus = aircraft_data.data["outputs"]['fuselage_dimensions']["l_fuselage"]
        self.placements = np.arange(0.2, 0.5, 0.001)
        self.diffs = []
        self.area = []
        self.best_placement = None

    def run(self):
        from tqdm import tqdm
        # Use tqdm for a progress bar
        for wing_placement in self.placements:
            loading_diagram = LoadingDiagram(aircraft_data=self.aircraft_data, wing_placement=wing_placement)
            mins, maxs = loading_diagram.determine_range()
            tail_area = Tail_area(aircraft_data=self.aircraft_data, fwd_cg=mins, aft_cg=maxs)
            tail_stab, tail_cont = tail_area.get_tail_area()
            S_h = max(tail_stab, tail_cont)
            self.area.append(S_h)
            if tail_stab > 0 and tail_cont > 0:
                self.diffs.append((wing_placement, abs(tail_stab - tail_cont), S_h, maxs, mins))
            else:
                self.diffs.append((wing_placement, 10000))
        self.select_best_placement()
        self.update_parameters()
        self.save_design()
        return self.best_placement

    def select_best_placement(self):
        filtered_diffs = [d for d in self.diffs if d[1] != 10000]
        if not filtered_diffs:
            self.best_placement = None
            return
        idx = np.argmin([d[1] for d in filtered_diffs])
        self.best_placement, diff, self.S_h, self.most_aft_cg, self.most_forward_cg = filtered_diffs[idx]

    def update_parameters(self):
        if self.best_placement is not None:
            self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] = self.best_placement * self.l_fus
            self.aircraft_data.data['outputs']['wing_design']['X_LE'] = self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] - self.aircraft_data.data['outputs']['wing_design']['y_MAC']*np.tan(np.deg2rad(self.aircraft_data.data['outputs']['wing_design']['sweep_x_c']))
            self.aircraft_data.data['outputs']['cg_range']['most_aft_cg_mac'] = self.most_aft_cg
            self.aircraft_data.data['outputs']['cg_range']['most_forward_cg_mac'] = self.most_forward_cg
            self.aircraft_data.data['outputs']['cg_range']['most_aft_cg'] = self.most_aft_cg * self.aircraft_data.data['outputs']['wing_design']['MAC'] + self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
            self.aircraft_data.data['outputs']['cg_range']['most_forward_cg'] = self.most_forward_cg * self.aircraft_data.data['outputs']['wing_design']['MAC'] + self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
            # self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos'] = 0.9*self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] - 0.25* self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['MAC']
            self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v'] = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos'] + 0.25*self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['MAC'] - self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] = self.S_h
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos'] = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['quarter_tip'] - 0.25*self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h'] = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos'] + 0.25*self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root'] - self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height'] = self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage'] + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'] 
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b'] = np.sqrt(self.S_h * self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['aspect_ratio'])
            # self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper'] = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip'] / self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
            ct_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
            b = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
            taper = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper']
            w = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']

            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root'] = ct_v*b / (2*(taper-1)*w/2 + b)
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip'] = taper * self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['MAC'] = (2 / 3) * self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root'] * ((1 + self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper'] + self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper']**2) / (1 + self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper']))
            self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['y_MAC'] = (self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b'] / 6) * (1 + 2 * self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper']) / (1 + self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper'])
            loading_diagram = LoadingDiagram(aircraft_data=self.aircraft_data, wing_placement=self.best_placement)
            loading_diagram.determine_range()


    def save_design(self):
        self.aircraft_data.save_design(design_file=f"design{self.aircraft_data.data['design_id']}.json")


if __name__ == "__main__":
    design_file = "design3.json"
    aircraft_data = Data(design_file)
    optimizer = EmpennageOptimizer(aircraft_data)
    best_placement = optimizer.run()
    print(f"Best wing placement for tail area: {best_placement}")
