import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType
from Class_II.ClassII_weight_estimation import ClassII
from Class_II.Small_Iteration import SmallIteration
from Class_I.Fuselage import Fuselage
from Class_I.PrelimWingPlanformDesign import WingPlanform
from Class_I.Cd0Estimation import Cd0Estimation
from Class_I.cgRange import CGRange
from Class_I.empennage import Empennage

class BigIteration:
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"
        self.mission_type = mission_type
        self.class_ii = ClassII(aircraft_data=self.aircraft_data)
        self.class_ii.main()

        self.iteration_number = 0
        self.max_iterations = 20
        self.tolerance = 0.01


    def main(self) -> None:
        for i in range(1, 5):
            print(f"Running iteration for design {i}...")
            file_path = f"design{i}.json"
            self.aircraft_data = Data(file_path)
            
            fuselage = Fuselage(aircraft_data=self.aircraft_data)
            fuselage.CalcFuseLen()

            for mission in MissionType:
                print(f"Running iteration for mission type {mission.name}...")
                self.prev_S = self.aircraft_data.data['outputs']['wing_design']['S']
                self.prev_MTOM = self.aircraft_data.data['outputs']['max']['MTOM']
                self.prev_CD0 = self.aircraft_data.data['inputs']['Cd0']
                self.main_iteration()


    def main_iteration(self):
        self.class_ii.main()
        iteration = SmallIteration(aircraft_data=self.aircraft_data, mission_type=self.mission_type, class_ii_OEW=self.class_ii.OEW)
        iteration.run_iteration()
        wing_planform = WingPlanform(aircraft_data=self.aircraft_data)
        wing_planform.calculate()

        cg_range = CGRange(aircraft_data=self.aircraft_data)
        cg_range.calculate_cg_range()

        emp = Empennage(aircraft_data=self.aircraft_data)
        emp.run_iteration()

        Cd0_est = Cd0Estimation(
            aircraft_data=self.aircraft_data,
            mission_type=self.mission_type
        )
        Cd0_est.mainloop()

        S = self.aircraft_data.data['outputs']['wing_design']['S']
        MTOM = self.aircraft_data.data['outputs']['max']['MTOM']
        Cd0 = self.aircraft_data.data['inputs']['Cd0']

        stop_condition = (abs(self.prev_S - S)/self.prev_S < self.tolerance and abs(self.prev_MTOM - MTOM)/self.prev_MTOM < self.tolerance and abs(self.prev_CD0 - Cd0)/self.prev_CD0 < self.tolerance) or self.iteration_number >= self.max_iterations
        self.iteration_number += 1

        self.prev_CD0 = Cd0
        self.prev_MTOM = MTOM
        self.prev_S = S

        if stop_condition:
            print(self.aircraft_data.data['outputs']['max']['MTOM'])
            self.aircraft_data.save_design(self.design_file)
            return
        else:
            return self.main_iteration()
        
if __name__ == "__main__":
    # Assuming the Data class is properly
    iteration = BigIteration(
        aircraft_data=Data('design3.json'),
        mission_type=MissionType.DESIGN
    )
    iteration.main()