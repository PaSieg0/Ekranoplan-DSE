import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Class_I.Iteration import AircraftIteration
from Class_I.ClassIWeightEstimation import MissionType
from Class_I.bar_graph import plot_bar_graph
from utils import design_json_to_excel,Data, generate_df
from Class_I.Fuselage import Fuselage
from Class_I.PrelimWingPlanformDesign import WingPlanform
from Class_I.Cd0Estimation import Cd0Estimation
from Class_I.cgRange import CGRange
from Class_I.empennage import Empennage
from Optimum_Performance.Optimum_speeds import OptimumSpeeds
from Fuselage import Fuselage
from PrelimWingPlanformDesign import WingPlanform
from Cd0Estimation import Cd0Estimation
from cgRange import CGRange
from empennage import Empennage
from engine_height import EngineHeight


def main(create_excel: bool = False) -> None:
    for i in range(3, 4):
        print(f"Running iteration for design {i}...")
        file_path = f"design{i}.json"
        aircraft_data = Data(file_path)
        
        # fuselage = Fuselage(aircraft_data=aircraft_data)
        # fuselage.CalcFuseLen()

        for mission in MissionType:
            print(f"Running iteration for mission type {mission.name}...")
            main_iteration(
                iteration_number=1,
                aircraft_data=aircraft_data,
                mission=mission,
                file_path=file_path,
                create_excel=create_excel,
                )


def main_iteration(
        aircraft_data: Data,
        mission: MissionType,
        file_path: str,
        tolerance=0.01,
        max_iterations=20,
        prev_S=300,
        prev_MTOM=100000,
        prev_CD0=0.02,
        prev_V_cruise=115,
        iteration_number = 1,
        create_excel: bool = False
    ):
    
    iteration = AircraftIteration(
                aircraft_data=aircraft_data,
                mission_type=mission
    )
    iteration.run_iteration()
    wing_planform = WingPlanform(aircraft_data=aircraft_data)
    wing_planform.calculate()

    cg_range = CGRange(aircraft_data=aircraft_data)
    cg_range.calculate_cg_range()

    emp = Empennage(aircraft_data=aircraft_data)
    emp.run_iteration()

    Cd0_est = Cd0Estimation(
        aircraft_data=aircraft_data,
        mission_type=mission
    )
    Cd0_est.mainloop()

    opt = OptimumSpeeds(
        aircraft_data=aircraft_data,
        mission_type=mission
    )
    opt.update_cruise_speed(aircraft_data.data['inputs']['cruise_altitude'])
    engine_height = EngineHeight(data=aircraft_data)
    engine_height.calculate_engine_positions()

    S = aircraft_data.data['outputs']['wing_design']['S']
    MTOM = aircraft_data.data['outputs']['max']['MTOM']
    Cd0 = aircraft_data.data['inputs']['Cd0']
    V_cruise = aircraft_data.data['requirements']['cruise_speed']

    stop_condition = ((abs(prev_S - S)/prev_S < tolerance and
                      abs(prev_MTOM - MTOM)/prev_MTOM < tolerance and 
                      abs(prev_CD0 - Cd0)/prev_CD0 < tolerance and
                      abs(prev_V_cruise - V_cruise)/prev_V_cruise < tolerance) or 
                      iteration_number >= max_iterations)
    iteration_number += 1

    if stop_condition:
        #aircraft_data.save_design(file_path)
        if create_excel:
            #design_json_to_excel(file_path,'concepts.xlsx')
            return
    else:
        return main_iteration(
            aircraft_data=aircraft_data,
            mission=mission,
            prev_S=S,
            prev_MTOM=MTOM,
            prev_CD0=Cd0,
            prev_V_cruise=V_cruise,
            iteration_number=iteration_number,
            file_path=file_path,
            create_excel=create_excel
            )
    



if __name__ == "__main__":
    main(create_excel=False)
    ''' df = generate_df()
    plot = plot_bar_graph(df, 'take_off_power')
    # print(df.columns)
    print(df[['design_id', 'mission_type', 'MTOM', 'fuel_economy', 'aspect_ratio', 'S', 'b', 'MAC']])
    plot.show()'''