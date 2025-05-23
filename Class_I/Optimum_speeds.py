import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType
from AltitudeVelocity import AltitudeVelocity


class OptimumSpeeds(AltitudeVelocity):
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        super().__init__(aircraft_data, mission_type)

        if mission_type == MissionType.FERRY:
            self._mtow = self.data.data['outputs']['design']['MTOW']-self.data.data['requirements']['design_payload']*9.81
            self._oew = self.data.data['outputs']['design']['OEW']
            self._zfw = self.data.data['outputs']['design']['ZFW']-self.data.data['requirements']['design_payload']*9.81
        else:
            self._mtow = self.data.data['outputs']['design']['MTOW']
            self._oew = self.data.data['outputs']['design']['OEW']
            self._zfw = self.data.data['outputs']['design']['ZFW']

        self.fuel_weight_per_metre = (self._mtow - self._zfw) / self.data.data['requirements']['design_range']  # kg/m

        self._current_weight = self._mtow

        self.ddist = 100000.0  # distance step in meters
    
    def step_weight(self) -> None:
        """
        Update the current weight of the aircraft.
        """
        dfuel_weight = -self.fuel_weight_per_metre*self.ddist
        self._current_weight += dfuel_weight

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

if __name__ == "__main__":
    # Example usage
    aircraft_data = Data("design3.json")
    mission_type = MissionType.DESIGN  # or any other mission type
    optimum_speeds = OptimumSpeeds(aircraft_data, mission_type)
    
    for i in np.arange(0, 3704000, optimum_speeds.ddist):
        v_range = optimum_speeds.v_range(0)  # Example altitude
        # Call methods as needed
        print(optimum_speeds.L_over_D(0, v_range), optimum_speeds.calculate_drag(v_range, 0))  # Example altitude and velocity

        optimum_speeds.step_weight()
