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
    RoD, V_min_rod_alt = altitude_velocity.calculate_min_RoD(h_WOG)
    AoD, V_min_aod_alt = altitude_velocity.calculate_min_AoD(h_WOG)

    altitude_velocity.plot_force_curve([h_WIG, h_WOG])

    print(f'Min RoD for Pr=0W at h=3048m: {RoD*196.85} ft/min at V = {V_min_rod_alt} m/s')
    print(f'Min AoD for Pr=0W at h=3048m: {AoD*180/np.pi} deg at V = {V_min_aod_alt} m/s')

