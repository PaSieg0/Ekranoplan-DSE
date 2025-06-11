import numpy as np
import matplotlib.pyplot as plt
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Optimum_Performance.Range_speed import RangeAnalyzer
from utils import Data, MissionType

if __name__ == "__main__":
    file_path = "design3.json"
    mission_type = MissionType.DESIGN
    analyzer = RangeAnalyzer(file_path, mission_type)
    range_m, time1, time2 = analyzer.calculate_range_variable(vfunc_leg1=None, 
                                                      vfunc_leg2=None, 
                                                      payload_leg1=None,
                                                      payload_leg2=0, 
                                                      h=10,
                                                      numerical=True)
    print(f"Range: {range_m/1852/2:.2f} nmi, Time leg 1: {time1/3600:.2f} h, Time leg 2: {time2/3600:.2f} h")