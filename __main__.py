import os
from utils import Data
from Iteration import AircraftIteration
from ClassIWeightEstimation import MissionType
from bar_graph import generate_df, plot_bar_graph
from Json_to_excel import json_to_excel


def main(create_excel: bool = False) -> None:
    for i in range(1, 5):
        print(f"Running iteration for design {i}...")
        file_path = f"design{i}.json"
        print(file_path)
        if create_excel:
            json_to_excel(file_path, f"Concept_Data.xlsx")
        aircraft_data = Data(file_path)
        aircraft_data.load_design(file_path)
        for mission in MissionType:
            print(f"Running iteration for mission type {mission.name}...")
            iteration = AircraftIteration(
                aircraft_data=aircraft_data,
                mission_type=mission)
            iteration.run_iteration()
            aircraft_data.save_design(file_path)


if __name__ == "__main__":
    main(create_excel=True)
    df = generate_df()
    plot = plot_bar_graph(df, 'MTOM')
    # print(df.columns)
    print(df[['design_id', 'mission_type', 'MTOM', 'fuel_economy', 'aspect_ratio', 'S', 'b', 'MAC']])
    plot.show()