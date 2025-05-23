import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_I.AltitudeVelocity import AltitudeVelocity
from utils import Data, MissionType, ISA

class Optimum_speeds(AltitudeVelocity):
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        self.data = aircraft_data
        self.mission_type = mission_type

        self.dive_speed = self.data.data['requirements']['cruise_speed'] / 0.6
        
        # Cache frequently used values to avoid repeated lookups
        self._mtow = self.data.data['outputs']['design']['MTOW']
        self._S = self.data.data['outputs']['design']['S']
        self._Cd0 = self.data.data['inputs']['Cd0']
        self._AR = self.data.data['inputs']['aspect_ratio']
        self._e = self.data.data['inputs']['oswald_factor']
        self._CLmax = self.data.data['inputs']['CLmax_clean']
        self._engine_power = self.data.data['inputs']['engine_power']
        self._sea_level_density = ISA(0).rho
        self._k = 1 / (np.pi * self._AR * self._e)  # Induced drag factor
        self.velocity_steps = 2000  # Number of velocity steps for calculations
        self.height_steps = 2000  # Number of height steps for calculations
    
    def v_range(self, h: float) -> float:
        """
        Calculate the maximum range velocity at a given altitude.
        """
        V_stall = self.calculate_stall_speed(h)
        velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps)

        # Compute drag for each velocity
        drag = np.array([self.calculate_drag(v, h) for v in velocity_range])

        # Find velocity at which drag is minimized
        min_drag_index = np.argmin(drag)
        v_max_range = velocity_range[min_drag_index]

        return v_max_range
    
    def v_endurance(self, h: float) -> float:
        """
        Calculate the maximum endurance velocity at a given altitude.
        """
        V_stall = self.calculate_stall_speed(h)
        velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps)

        # Compute drag for each velocity
        Pr = np.array([self.calculate_power_required(v, h) for v in velocity_range])

        # Find velocity at which drag is minimized
        min_drag_index = np.argmin(Pr)
        v_max_endurance = velocity_range[min_drag_index]

        return v_max_endurance
    

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)
    optimum_speeds = Optimum_speeds(aircraft_data, mission_type)

    V_cruise = aircraft_data.data['requirements']['cruise_speed']
    h_WIG = 5
    h_WOG = 3048
    RoC_req = 1000/196.85

    RoC, V_max_roc = optimum_speeds.calculate_max_RoC(h_WIG)
    AoC, V_max_aod = optimum_speeds.calculate_max_AoC(h_WIG)
    RoD, V_min_rod = optimum_speeds.calculate_min_RoD(h_WIG)
    AoD, V_min_aod = optimum_speeds.calculate_min_AoD(h_WIG)

    print("===============Optimum speeds==================")
    print(f"Range: {optimum_speeds.v_range(h_WIG):.2f} m/s")
    print(f"Endurance: {optimum_speeds.v_endurance(h_WIG):.2f} m/s")
    print(f"RoC: {V_max_roc:.2f} m/s")
    print(f"AoC: {V_max_aod:.2f} m/s")
    print(f'RoD: {V_min_rod:.2f} m/s')
    print(f'AoD: {V_min_aod:.2f} m/s')

    altitude_velocity.plot_force_curve([h_WIG, h_WOG])