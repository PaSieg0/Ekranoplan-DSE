import os
from utils import Data
from Iteration import AircraftIteration
from ClassIWeightEstimation import MissionType


def main():
    for i in range(1, 6):
        print(f"Running iteration for design {i}...")
        file_path = os.path.join(os.path.dirname(__file__), "Data", f"design{i}.json")
        if not os.path.exists(file_path):
            print(f"File {file_path} does not exist. Skipping.")
            continue
    
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