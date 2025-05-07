import os
from utils import Data
from Iteration import AircraftIteration
from ClassIWeightEstimation import MissionType
from bar_graph import generate_df, plot_bar_graph


def main():
    for i in range(1, 6):
        print(f"Running iteration for design {i}...")
        file_path = f"design{i}.json"
        print(file_path)
    
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
    main()
    df = generate_df()
    plot = plot_bar_graph(df, 'fuel_economy')
    plot.show()