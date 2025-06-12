import os
import copy
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType
from Class_I.Fuselage import Fuselage
from Class_I.PrelimWingPlanformDesign import WingPlanform
from Class_I.cgRange import CGRange
from Class_II.ClassII_weight_estimation import ClassII
from Class_II.Small_Iteration import SmallIteration
from Class_II.Modified_CD0 import Cd0Estimation
from Class_II.Empennage_Design.Vertical_tail import vertical_tail_sizing
from Class_II.Empennage_Design.main_empennage import EmpennageOptimizer
from Class_II.FuselageThickness import FuselageThickness
from Class_II.StressAnalysis_Wing import main as wing_stress_analysis
from Class_II.AileronHLD import AileronHLD
from Class_II.ElevatorRudder import ElevatorRudder
from Aero_stab.Derivatives_datcom_sym import DerivativesDatcom_sym
from Aero_stab.Derivatives_datcom_asym  import DerivativesDatcom_asym
from Optimum_Performance.Optimum_speeds import OptimumSpeeds


class FinalIteration:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_file = f"design{self.aircraft_data.data['design_id']}.json"

        self.max_iterations = 100

    def main(self, mission_type: MissionType):
        self.prev_json = copy.deepcopy(self.aircraft_data.data.copy())
        iteration_number = 0
        self.MTOW = []
        self.S = []
        self.fuel_economy = []    
        while True:
            # Run iteration lil broski
            iteration_number += 1
            print(f"Iteration {iteration_number} started...")

            # Run Class II
            print('Class II + I')
            class_ii = ClassII(aircraft_data=self.aircraft_data)
            class_ii.main()

            # Run Class I (Small iteration)
            iteration = SmallIteration(aircraft_data=self.aircraft_data, mission_type=mission_type, class_ii_OEW=class_ii.OEW)
            iteration.run_iteration()

            # Wing planform
            print('Wing planform')
            wing_planform = WingPlanform(aircraft_data = self.aircraft_data)
            wing_planform.calculate()

            # CD0
            print('Cd0')
            Cd0_est = Cd0Estimation(aircraft_data = self.aircraft_data, mission_type = mission_type, class_ii_OEW=class_ii.OEW)
            Cd0_est.mainloop()

            # Cruise Speed
            optimum_speeds = OptimumSpeeds(self.aircraft_data, mission_type)
            optimum_speeds.update_json(self.aircraft_data.data['inputs']['cruise_altitude'])

            # Stability derivatives
            stab_coeff_sym = DerivativesDatcom_sym(self.aircraft_data)
            stab_coeff_sym.update_json()
            stab_coeff_asym = DerivativesDatcom_asym(self.aircraft_data)
            stab_coeff_asym.update_json()

            # EmpennageOptimizer (Vertical Tail, main_empennage)
            print('Empennage shits')
            empennage_optimizer = EmpennageOptimizer(self.aircraft_data)
            empennage_optimizer.run()

            # Vertical Tail Sizing
            vertical_tail = vertical_tail_sizing(self.aircraft_data)
            vertical_tail.get_vertical_tail_size()
            
            # EmpennageOptimizer (Vertical Tail, main_empennage)
            empennage_optimizer = EmpennageOptimizer(self.aircraft_data)
            empennage_optimizer.run()

            empennage_optimizer = EmpennageOptimizer(self.aircraft_data)
            empennage_optimizer.run()
            
            # AileronHLD 
            print('Control surfaces')
            aileron_hld = AileronHLD(aircraft_data=self.aircraft_data)
            aileron_hld.main()

            # ElevatorRudder
            elevator_rudder = ElevatorRudder(aircraft_data=self.aircraft_data)
            elevator_rudder.main()

            # FuselageThickness
            print('Fuselage')
            fuselage = FuselageThickness(aircraft_data=self.aircraft_data)
            fuselage.main()
            
            # StressAnalysisWing
            print('Wing Stress')
            critical_margins = wing_stress_analysis()

            if critical_margins != 0:
                print('The wing box is weak bro 😔')
                break
            
            stop_condition = compare_dicts(self.aircraft_data.data, self.prev_json, tolerance=0.01) or iteration_number >= self.max_iterations

            print(self.aircraft_data.data['outputs']['design']['MTOM'], self.prev_json['outputs']['design']['MTOM'])

            if stop_condition:
                self.aircraft_data.save_design(self.design_file)
                print("Final iteration completed successfully. LET'S GOOOOO BABY! Time to put the blinds up. 😎")
                break
            
            self.prev_json = copy.deepcopy(self.aircraft_data.data.copy())


def compare_dicts(dict1, dict2, tolerance=0.01):
    """
    Compares two dictionaries recursively with a tolerance for value differences.
    
    :param dict1: First dictionary.
    :param dict2: Second dictionary.
    :param tolerance: Tolerance level (percentage or absolute difference) for values to be considered "close".
    :return: True if dictionaries are the same within the tolerance, False otherwise.
    """
    if dict1.keys() != dict2.keys():
        return False

    for key in dict1:
        if key == 'requirements':
            continue
        val1 = dict1[key]
        val2 = dict2[key]

        # Check if both values are dictionaries, and recursively compare them
        if isinstance(val1, dict) and isinstance(val2, dict):
            if not compare_dicts(val1, val2, tolerance):
                print(f"Difference found in key '{key}': {val1} != {val2}\n")
                return False
        else:
            # Compare values within the given tolerance
            if not is_close(val1, val2, tolerance):
                print(f"Difference found in key '{key}': {val1} != {val2}\n")
                return False
    return True


def is_close(val1, val2, tolerance):
    """
    Checks if two values are close within a specified tolerance.
    
    :param val1: First value.
    :param val2: Second value.
    :param tolerance: Tolerance (absolute or relative difference) allowed.
    :return: True if values are within the tolerance, False otherwise.
    """
    if isinstance(val1, (int, float)) and isinstance(val2, (int, float)):
        # You can define your own tolerance logic here (percentage or absolute)
        if abs(val1 - val2) <= tolerance:
            return True
        if max(val1, val2) > 0:  # Allow percentage-based comparison if the numbers are large enough
            if abs(val1 - val2) / max(abs(val1), abs(val2)) <= tolerance:
                return True
    if isinstance(val1, list) and isinstance(val2, list):
        if len(val1) != len(val2):
            return False
        for v1, v2 in zip(val1, val2):
            if not is_close(v1, v2, tolerance):
                return False
        return True
    return val1 == val2  # Fallback for non-numeric values

if __name__ == "__main__":
    # Assuming the Data class is properly
    # aircraft_data = Data('design3.json')
    # aircraft_data2 = copy.deepcopy(aircraft_data.data)
    # aircraft_data2['outputs']['max']['MTOW'] += 4000
    # print(aircraft_data.data['outputs']['max']['MTOW'])
    # print(aircraft_data2['outputs']['max']['MTOW'])
    # print(compare_dicts(aircraft_data.data, aircraft_data2, tolerance=0.01))

    aircraft_data = Data('design3.json')
    final_iteration = FinalIteration(aircraft_data=aircraft_data)
    final_iteration.main(mission_type=MissionType.DESIGN)
    