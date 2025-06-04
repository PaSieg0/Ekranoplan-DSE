import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType
from Class_I.AltitudeVelocity import AltitudeVelocity


class OptimumSpeeds(AltitudeVelocity):
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        super().__init__(aircraft_data, mission_type)

        if mission_type == MissionType.FERRY:
            self._mtow = self.data.data['outputs']['design']['MTOW']-self.data.data['requirements']['design_payload']*self.g
            self._oew = self.data.data['outputs']['design']['OEW']
            self._zfw = self.data.data['outputs']['design']['ZFW']-self.data.data['requirements']['design_payload']*self.g
        else:
            self._mtow = self.data.data['outputs']['design']['MTOW']
            self._oew = self.data.data['outputs']['design']['OEW']
            self._zfw = self.data.data['outputs']['design']['ZFW']

        self.fuel_weight_per_metre = (self._mtow - self._zfw) / self.data.data['requirements']['design_range']  # kg/m

        self._current_weight = self._mtow
    
    def step_weight(self, dw) -> None:
        """
        Update the current weight of the aircraft.
        """
        self._current_weight += dw

    def v_range(self, h: float) -> float:
        """
        Calculate the maximum range velocity at a given altitude.
        """
        V_stall = self.calculate_stall_speed(h)
        velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps*5)

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
        velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps*5)

        # Compute drag for each velocity
        Pr = np.array([self.calculate_power_required(v, h) for v in velocity_range])

        # Find velocity at which drag is minimized
        min_pr_index = np.argmin(Pr)
        v_max_endurance = velocity_range[min_pr_index]

        return v_max_endurance
    
    def L_over_D(self, h: float, v: float) -> float:
        """
        Calculate the lift-to-drag ratio at a given altitude.
        """

        # Compute drag for each velocity
        drag = self.calculate_drag(v, h)
        lift = self._current_weight

        # Calculate L/D ratio
        L_over_D = lift / drag

        return L_over_D
    
    def update_cruise_speed(self, h: float) -> None:
        """
        Update the cruise speed based on the current altitude.
        """
        self.data.data['requirements']['cruise_speed'] = self.v_range(h)

if __name__ == "__main__":
    # Example usage
    aircraft_data = Data("design3.json")
    mission_type = MissionType.DESIGN  # or any other mission type
    optimum_speeds = OptimumSpeeds(aircraft_data, mission_type)

    h = h_WIG = 10  # Example altitude in meters

    optimum_speeds._current_weight = 1839082
    v = optimum_speeds.v_range(h)
    ld = optimum_speeds.L_over_D(h, v)
    print(f"{v:.2f}\t\t{optimum_speeds._current_weight:.2f}\t\t{ld:.2f}")

    v_range = optimum_speeds.v_range(h)
    v_endurance = optimum_speeds.v_endurance(h)
    _, v_max_roc = optimum_speeds.calculate_max_RoC(h)
    _, v_max_aod = optimum_speeds.calculate_max_AoC(h)
    _, v_min_rod = optimum_speeds.calculate_min_RoD(h)
    _, v_min_aod = optimum_speeds.calculate_min_AoD(h)

    print(f"Optimum range speed: {v_range:.2f} m/s")
    print(f"Optimum endurance speed: {v_endurance:.2f} m/s")
    print(f"Maximum rate of climb speed: {v_max_roc:.2f} m/s")
    print(f"Maximum angle of climb speed: {v_max_aod:.2f} m/s")
    print(f"Minimum rate of descent speed: {v_min_rod:.2f} m/s")
    print(f"Minimum angle of descent speed: {v_min_aod:.2f} m/s")
