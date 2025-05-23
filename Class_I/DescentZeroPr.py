import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_I.AltitudeVelocity import AltitudeVelocity
from utils import Data, MissionType, ISA
from Optimum_speeds import OptimumSpeeds

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)

    V_cruise = aircraft_data.data['requirements']['cruise_speed']
    h_WIG = 5
    h_WOG = 3048
    RoC_req = 1000/196.85

    RoC, V_max_roc = altitude_velocity.calculate_max_RoC(h_WIG)
    AoC, V_max_aod = altitude_velocity.calculate_max_AoC(h_WIG)
    RoD, V_min_rod = altitude_velocity.calculate_min_RoD(h_WIG)
    AoD, V_min_aod = altitude_velocity.calculate_min_AoD(h_WIG)

    print("===============Optimum speeds==================")
    print(f"RoC: {V_max_roc:.2f} m/s")
    print(f"AoC: {V_max_aod:.2f} m/s")
    print(f'RoD: {V_min_rod:.2f} m/s')
    print(f'AoD: {V_min_aod:.2f} m/s')

    altitude_velocity.plot_force_curve([h_WIG, h_WOG])