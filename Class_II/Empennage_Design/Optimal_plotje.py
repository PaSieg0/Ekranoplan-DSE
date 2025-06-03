import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from utils import Data
from Class_II.Empennage_Design.main_empennage import EmpennageOptimizer

def main():
    diffs = []
    for forebody_length in np.arange(16.12, 23.12, 1):
        print(f"Running optimization for forebody length: {forebody_length}")
        aircraft_data = Data('design3.json')
        aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] = forebody_length
        aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']
        optimizer = EmpennageOptimizer(aircraft_data)
        best_placement = optimizer.run()
        aft_cg = aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        step_dist = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        diffs.append(aft_cg-step_dist)
        if aft_cg < step_dist:
            print(f"LETS GOOOOOOOOOOO {aft_cg}, {step_dist}.")
            print(best_placement)
    return diffs

if __name__ == "__main__":
    diffs = main()
    plt.plot(np.arange(16.12, 23.12, 1), diffs)
    plt.grid()
    plt.xlabel('Forebody length [m]')
    plt.ylabel('Difference between aft_cg and step_position')
    plt.title('- is good + is bad')
    plt.show()