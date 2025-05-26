import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType
from Class_II.Modified_Class_I import ModifiedClassI
from Class_II.ClassII_weight_estimation import ClassII
from Class_I.WingLoading import WingLoading
from Class_I.__main__ import main
from Class_I.PrelimWingPlanformDesign import WingPlanform
from Class_I.Iteration import AircraftIteration

class MainIteration:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"
        self.class_ii = ClassII(aircraft_data=self.aircraft_data)
        self.class_ii.main()
        self.class_i = ModifiedClassI(aircraft_data=self.aircraft_data, mission_type=MissionType.DESIGN, class_ii_OEW=self.class_ii.OEW)
        self.prev_OEW = self.aircraft_data.data['outputs']['max']['OEW']/9.81
        self.curr_OEW = self.class_ii.OEW

    def determine_subsystems(self):
        # Make all subsystems calculations run with the new MTOW
        WS = self.aircraft_data.data['outputs']['max']['WS']
        S = self.class_i.MTOW / WS
        WingPlanform(aircraft_data=self.aircraft_data).calculate()

    
    def run_iteration(self):
        while True:
            self.class_ii.main()
            self.class_i.OEW = self.class_ii.OEW
            self.class_i.main()

            self.aircraft_data.data['outputs']['max']['MTOW'] = self.class_i.MTOW
            self.determine_subsystems()

            print(self.prev_OEW, self.curr_OEW)

            stop_condition = abs(self.prev_OEW - self.curr_OEW)/self.prev_OEW < 0.01
            if stop_condition:
                self.aircraft_data.save_design(self.design_file)
                break




if __name__ == "__main__":
    # Assuming the Data class is properly
    iteration = MainIteration(
        aircraft_data=Data('design3.json')   
    )
    iteration.run_iteration()