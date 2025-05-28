import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
from Class_I.ClassIWeightEstimation import ClassI, MissionType
from Class_I.PayloadRange import RangeCalculator

if __name__ == "__main__":
    # Example usage
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.ALTITUDE  # Example mission type

    range_calculator = RangeCalculator(data_object=aircraft_data, mission_type=mission_type)
    
    # Perform analysis and plot
    ranges_nm, points = range_calculator.analyze_and_plot(show=True)
    
    # Print summary
    print(range_calculator.get_results_summary())
