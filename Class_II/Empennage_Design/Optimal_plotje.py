import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from utils import Data
from Class_II.Empennage_Design.main_empennage import EmpennageOptimizer

def main():

    forebody_lengths = np.arange(20.5, 23.13, 0.1)
    diffs = []
    aft_cgs = []
    step_dists = []
    found = False
    for forebody_length in forebody_lengths:
        print(f"Running optimization for forebody length: {forebody_length}")
        aircraft_data = Data('design3.json')
        aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] = forebody_length
        aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']
        optimizer = EmpennageOptimizer(aircraft_data)
        best_placement = optimizer.run()
        aft_cg = aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        step_dist = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        diff = aft_cg - step_dist
        diffs.append(diff)
        aft_cgs.append(aft_cg)
        step_dists.append(step_dist)
        if not found and diff <= -0.5:
            print(f"First forebody length where diff <= -0.5: {forebody_length:.2f} (diff={diff:.3f})")
            chosen_length = forebody_length
            aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] = round(chosen_length, 2)
            aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']
            found = True
            break
        if aft_cg < step_dist:
            print(f"LETS GOOOOOOOOOOO {aft_cg}, {step_dist}.")
            print(best_placement)
    return forebody_lengths, diffs, aft_cgs, step_dists, chosen_length if found else None

if __name__ == "__main__":
    forebody_lengths, diffs, aft_cgs, step_dists, chosen_length = main()
