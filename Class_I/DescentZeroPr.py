import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_I.AltitudeVelocity import AltitudeVelocity
from utils import Data, MissionType

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)

    V_cruise = aircraft_data.data['requirements']['cruise_speed']
    h_WIG = 5
    h_WOG = 3048
    RoC_req = 1000/196.85
    _, V_max_roc_alt = altitude_velocity.calculate_max_RoC(h_WOG)
    _, V_max_aoc_alt = altitude_velocity.calculate_max_AoC(h_WOG)
    RoD = -altitude_velocity.calculate_power_required(V_max_roc_alt, h_WOG, 0)/altitude_velocity._mtow * 196.85
    AoD = np.arcsin(-altitude_velocity.calculate_power_required(V_max_aoc_alt, h_WOG, 0)/V_max_aoc_alt/altitude_velocity._mtow) * 180/np.pi
    print(f'Min RoD for Pr=0W at h=3048m: {RoD} ft/min at V = {V_max_roc_alt} m/s')
    print(f'Min AoD for Pr=0W at h=3048m: {AoD} deg at V = {V_max_aoc_alt} m/s')

