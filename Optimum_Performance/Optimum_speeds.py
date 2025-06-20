import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType
from Class_I.AltitudeVelocity import AltitudeVelocity
from scipy.optimize import minimize_scalar


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

    def v_range(self, h: float, W: float = None) -> float:
        """
        Calculate the maximum range velocity at a given altitude.
        """
        if W is None:
            W = self._current_weight
            
        V_stall = self.calculate_stall_speed(h)
        def drag_func(V):
            return self.calculate_drag(V, h, W)

        res = minimize_scalar(
            drag_func,
            bounds=(V_stall, self.dive_speed),
            method='bounded',
            options={'xatol': 1e-6}
        )
        v_max_range = res.x
        return v_max_range
    
    def v_endurance(self, h: float, W: float = None) -> float:
        """
        Calculate the maximum endurance velocity at a given altitude.
        """
        if W is None:
            W = self._current_weight

        V_stall = self.calculate_stall_speed(h)
        def power_func(V):
            return self.calculate_power_required(V, h)

        res = minimize_scalar(
            power_func,
            bounds=(V_stall, self.dive_speed),
            method='bounded',
            options={'xatol': 1e-6}
        )
        v_max_endurance = res.x
        return v_max_endurance
    
    def v_max(self, h: float, W: float = None) -> float:
        """
        Calculate the maximum velocity at a given altitude (where rate of climb = 0).
        """
        if W is None:
            W = self._current_weight

        V_stall = self.calculate_stall_speed(h)
        
        def roc_func(V):
            roc = self.calculate_RoC(V, h)
            return abs(roc)  # Minimize absolute value to find where RoC = 0

        res = minimize_scalar(
            roc_func,
            bounds=(V_stall, self.dive_speed*2),
            method='bounded',
            options={'xatol': 1e-6}
        )
        v_max = res.x
        return v_max
    
    def v_stall(self, h: float, W: float = None) -> float:
        """
        Calculate the stall speed at a given altitude.
        """
        if W is None:
            W = self._current_weight
        v_stall = self.calculate_stall_speed(h, W=W)
        return v_stall
    
    def L_over_D(self, V: float, h: float, W: float) -> float:
        """
        Calculate the lift-to-drag ratio at a given altitude.
        """

        # Compute drag for each velocity
        drag = self.calculate_drag(V, h, W)
        lift = W

        # Calculate L/D ratio
        L_over_D = lift / drag

        return L_over_D
    
    def update_json(self, h: float) -> None:
        """
        Update the cruise speed based on the current altitude.
        """
        self.data.data['requirements']['cruise_speed'] = self.v_range(h)
        self.data.data['outputs']['optimum_speeds']['range'] = self.v_range(h)
        self.data.data['outputs']['optimum_speeds']['endurance'] = self.v_endurance(h)
        self.data.data['outputs']['optimum_speeds']['max'] = self.v_max(h)
        self.data.data['outputs']['optimum_speeds']['max_roc'] = self.calculate_max_RoC(h)[1]
        self.data.data['outputs']['optimum_speeds']['max_aoc'] = self.calculate_max_AoC(h)[1]
        self.data.data['outputs']['optimum_speeds']['min_rod'] = self.calculate_min_RoD(h)[1]
        self.data.data['outputs']['optimum_speeds']['min_aod'] = self.calculate_min_AoD(h)[1]
        self.data.save_design("design3.json")

if __name__ == "__main__":
    # Example usage
    aircraft_data = Data("design3.json")
    mission_type = MissionType.DESIGN  # or any other mission type
    optimum_speeds = OptimumSpeeds(aircraft_data, mission_type)

    h = h_WIG = 10  # Example altitude in meters

    optimum_speeds._current_weight = optimum_speeds._mtow  # Set current weight to MTOW for calculations
    optimum_speeds.update_json(h)

    v_range = optimum_speeds.v_range(h)
    v_range_10k = optimum_speeds.v_range(3048)  # Example altitude at 10,000 feet
    v_endurance = optimum_speeds.v_endurance(h)
    v_max = optimum_speeds.v_max(h)
    roc_0, v_max_roc = optimum_speeds.calculate_max_RoC(h)
    roc_10k, v_max_roc_10k = optimum_speeds.calculate_max_RoC(3048)
    _, v_max_aod = optimum_speeds.calculate_max_AoC(h)
    _, v_min_rod = optimum_speeds.calculate_min_RoD(h)
    _, v_min_aod = optimum_speeds.calculate_min_AoD(h)

    pr_cruise = optimum_speeds.calculate_power_required(v_range, h)
    pr_cruise_10k = optimum_speeds.calculate_power_required(v_range_10k, 3048)
    pr_climb = optimum_speeds.calculate_power_required(v_max_roc, h, roc_0)
    pr_climb_high = optimum_speeds.calculate_power_required(v_max_roc_10k, 3048, roc_10k)

    print(f"Speeds at {h}m altitude:")
    print(f"Range speed: {v_range:.2f} m/s")
    print(f"Range speed at 10k ft: {v_range_10k:.2f} m/s")
    print(f"Max RoC speed: {v_max_roc:.2f} m/s")
    print(f"Max RoC speed at 10k ft: {v_max_roc_10k:.2f} m/s")

    print(f"\nPower Requirements:")
    print(f"Cruise power (at range speed): {pr_cruise:.2f} W")
    print(f"Cruise power at 10k ft: {pr_cruise_10k:.2f} W")
    print(f"Climb power: {pr_climb:.2f} W")
    print(f"Climb power at high altitude: {pr_climb_high:.2f} W")
