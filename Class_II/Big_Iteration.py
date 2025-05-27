import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType
from Class_II.ClassII_weight_estimation import ClassII
from Class_II.Small_Iteration import SmallIteration
from Class_I.Fuselage import Fuselage
from Class_I.PrelimWingPlanformDesign import WingPlanform
from Class_II.Modified_CD0 import Cd0Estimation
from Class_I.cgRange import CGRange
from Class_I.empennage import Empennage

class BigIteration:
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"
        self.class_ii = ClassII(aircraft_data=self.aircraft_data)
        self.class_ii.main()

        self.iteration_number = 0
        self.max_iterations = 100
        self.tolerance = 0.0000001


    def main(self) -> None:
        for i in range(3, 4):
            file_path = f"design{i}.json"
            self.aircraft_data = Data(file_path)
            
            fuselage = Fuselage(aircraft_data=self.aircraft_data)
            fuselage.CalcFuseLen()

            for mission in MissionType:
                self.iteration_number = 0
                self.prev_S = self.aircraft_data.data['outputs']['wing_design']['S']
                self.prev_MTOM = self.aircraft_data.data['outputs'][mission.name.lower()]['MTOM']
                self.prev_CD0 = self.aircraft_data.data['inputs']['Cd0']
                self.prev_OEW = self.aircraft_data.data['outputs'][mission.name.lower()]['OEW']
                self.main_iteration(mission)


    def main_iteration(self, mission_type: MissionType):
        while True:
            self.class_ii.main()
            self.aircraft_data.data['outputs'][mission_type.name.lower()]['OEW'] = self.class_ii.OEW*9.81
            iteration = SmallIteration(aircraft_data=self.aircraft_data, mission_type=mission_type, class_ii_OEW=self.class_ii.OEW)
            iteration.run_iteration()


            wing_planform = WingPlanform(aircraft_data=self.aircraft_data)
            wing_planform.calculate()

            cg_range = CGRange(aircraft_data=self.aircraft_data)
            cg_range.calculate_cg_range()

            emp = Empennage(aircraft_data=self.aircraft_data)
            emp.run_iteration()

            Cd0_est = Cd0Estimation(
                aircraft_data=self.aircraft_data,
                mission_type=mission_type,
                class_ii_OEW=self.class_ii.OEW
            )
            Cd0_est.mainloop()

            S = self.aircraft_data.data['outputs']['wing_design']['S']
            MTOM = self.aircraft_data.data['outputs'][mission_type.name.lower()]['MTOM']
            Cd0 = self.aircraft_data.data['inputs']['Cd0']
            OEW = self.aircraft_data.data['outputs'][mission_type.name.lower()]['OEW']

            stop_condition = (
    abs(self.prev_S - S) / self.prev_S < self.tolerance and
    abs(self.prev_MTOM - MTOM) / self.prev_MTOM < self.tolerance and
    abs(self.prev_CD0 - Cd0) / self.prev_CD0 < self.tolerance and
    abs(self.prev_OEW - OEW) / self.prev_OEW < self.tolerance
) or self.iteration_number >= self.max_iterations

            self.iteration_number += 1
            self.prev_CD0 = Cd0
            self.prev_MTOM = MTOM
            self.prev_S = S
            self.prev_OEW = OEW

            if stop_condition:
                self.aircraft_data.save_design(self.design_file)
                break

        
if __name__ == "__main__":
    # Assuming the Data class is properly
    while True:
        iteration = BigIteration(
            aircraft_data=Data('design3.json'),
            mission_type=MissionType.DESIGN
        )
        prev_MTOM = iteration.aircraft_data.data['outputs']['max']['MTOM']
        iteration.main()
        MTOM = iteration.aircraft_data.data['outputs']['max']['MTOM']
        if abs(prev_MTOM - MTOM) / prev_MTOM < iteration.tolerance:
            print(f"Convergence achieved with MTOM: {MTOM:,.0f} kg")
            break
        prev_MTOM = MTOM