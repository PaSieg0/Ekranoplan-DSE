import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Class_I.ClassIWeightEstimation import ClassI
from utils import Data, ISA, MissionType, plt

if __name__ == "__main__":
    # Create an instance of ClassI
    C130 = Data('designC130.json')
    for mission in MissionType:
        if mission == MissionType.ALTITUDE:
            continue
        class_i = ClassI(aircraft_data=C130, mission_type=mission)
        class_i.main()
        print(f"Class I Weight Estimation for {mission.name} mission completed.")
        # print(class_i.MTOM)
        # print(class_i.LD)
        # print(class_i.Mff)
        # print(class_i.design_file)