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
            iteration = AircraftIteration(
                aircraft_data=aircraft_data,
                mission_type=mission)
            iteration.run_iteration()
            aircraft_data.save_design(file_path)
            if create_excel:
                design_json_to_excel(file_path, f"Concept_Data.xlsx")

        wing_planform = WingPlanform(aircraft_data=aircraft_data)
        wing_planform.calculate()

        cg_range = CGRange(aircraft_data=aircraft_data)
        cg_range.calculate_cg_range()

        emp = Empennage(aircraft_data=aircraft_data)
        emp.calculate_tail_areas()
        
        for mission in MissionType:
            cd0_est = Cd0Estimation(
                aircraft_data=aircraft_data,
                mission_type=mission
            )
            cd0_est.mainloop()

if __name__ == "__main__":
    main(create_excel=True)
    ''' df = generate_df()
    plot = plot_bar_graph(df, 'take_off_power')
    # print(df.columns)
    print(df[['design_id', 'mission_type', 'MTOM', 'fuel_economy', 'aspect_ratio', 'S', 'b', 'MAC']])
    plot.show()'''