import os
from utils import Data, generate_df
from Iteration import AircraftIteration
from ClassIWeightEstimation import MissionType
from bar_graph import plot_bar_graph
from Json_to_excel import design_json_to_excel
from Fuselage import Fuselage
from PrelimWingPlanformDesign import WingPlanform
from Cd0Estimation import Cd0Estimation
from cgRange import CGRange
from empennage import Empennage


def main(create_excel: bool = False) -> None:
    for i in range(1, 5):
        print(f"Running iteration for design {i}...")
        file_path = f"design{i}.json"
        aircraft_data = Data(file_path)
        
        fuselage = Fuselage(aircraft_data=aircraft_data)
        fuselage.CalcFuseLen()

        for mission in MissionType:
            print(f"Running iteration for mission type {mission.name}...")
            main_iteration(
                iteration_number=1,
                aircraft_data=aircraft_data,
                mission=mission,
                file_path=file_path
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
        iteration_number = 1,
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

    S = aircraft_data.data['outputs']['wing_design']['S']
    MTOM = aircraft_data.data['outputs']['max']['MTOM']
    Cd0 = aircraft_data.data['inputs']['Cd0']

    stop_condition = (abs(prev_S - S)/prev_S < tolerance and abs(prev_MTOM - MTOM)/prev_MTOM < tolerance and abs(prev_CD0 - Cd0)/prev_CD0 < tolerance) or iteration_number >= max_iterations
    iteration_number += 1

    if stop_condition:
        #aircraft_data.save_design(file_path)
        return
    else:
        return main_iteration(
            aircraft_data=aircraft_data,
            mission=mission,
            prev_S=S,
            prev_MTOM=MTOM,
            prev_CD0=Cd0,
            iteration_number=iteration_number,
            file_path=file_path
            )
    



if __name__ == "__main__":
    main(create_excel=True)
    ''' df = generate_df()
    plot = plot_bar_graph(df, 'take_off_power')
    # print(df.columns)
    print(df[['design_id', 'mission_type', 'MTOM', 'fuel_economy', 'aspect_ratio', 'S', 'b', 'MAC']])
    plot.show()'''