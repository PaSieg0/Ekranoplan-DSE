import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType
from Class_I.AltitudeVelocity import AltitudeVelocity
from tqdm import tqdm

class OptimumTurns(AltitudeVelocity):
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        super().__init__(aircraft_data, mission_type)
        self.h = aircraft_data.data['inputs']['cruise_altitude']
        self.span = aircraft_data.data['outputs']['design']['b']
        self.dihedral = aircraft_data.data['outputs']['wing_design']['dihedral']

    def max_n_turn_points(self, h):
        """
        Calculate the steepest turn points based on the aircraft's performance data.
        
        Parameters:
        airspeed_type (str): Type of airspeed ('true' or 'indicated').
        altitude_units (str): Units for altitude ('meters' or 'feet').
        
        Returns:
        tuple: Zero points and stall points for the steepest turn.
        """
        n_range = np.arange(1.02, 3.5, 0.001)
        V_stall = self.calculate_stall_speed(h)
        v_range = np.linspace(V_stall, self.dive_speed*2, self.velocity_steps)

        zero_points = np.array([])

        for n in n_range:
            self._current_weight = self._mtow * n
            extra_acceleration = self.calculate_RoC_vectorized(v_range, h)/v_range

            # Find where RoC crosses RoC_limit
            diff_sign = np.diff(np.sign(extra_acceleration))
            zero_crossings = np.where(diff_sign != 0)[0]
        
            # Interpolate to find more accurate crossing velocities
            for idx in zero_crossings:
                v1, v2 = v_range[idx], v_range[idx + 1]
                acc1, acc2 = extra_acceleration[idx], extra_acceleration[idx + 1]
                if acc2 != acc1:
                    v_zero = v1 + (0 - acc1) * (v2 - v1) / (acc2 - acc1)
                else:
                    v_zero = v1

                zero_points = np.append(zero_points, [[v_zero, n]], axis=0) if zero_points.size else np.array([[v_zero, n]])

        idx = np.argsort(zero_points[:, 0])
        zero_points = zero_points[idx]

        self._current_weight = self._mtow

        return zero_points
    
    def plot_vs_velocity(self, points, y_axis_name="Value", ax=None, **kwargs):
        """
        Plot any value vs velocity using points as returned by calculations.

        Parameters:
        points (np.ndarray): Array with columns [velocity, y_value].
        y_axis_name (str): Label for the y-axis.
        ax (matplotlib.axes.Axes, optional): Matplotlib axis to plot on.
        kwargs: Additional keyword arguments for plt.plot.
        """
        if ax is None:
            fig, ax = plt.subplots()
        ax.plot(points[:, 0], points[:, 1], label=f'{y_axis_name} vs Velocity', **kwargs)
        ax.set_xlabel('Velocity [m/s]')
        ax.set_ylabel(y_axis_name)
        ax.set_title(f'{y_axis_name} vs Velocity')
        ax.grid(True)
        ax.legend()
        plt.show()
    
    def max_bank_angle_points(self, zero_points):
        max_bank_angle = np.arccos(1/zero_points[:, 1]) * (180 / np.pi)
        points = np.column_stack((zero_points[:, 0], max_bank_angle))
        return points

    def bank_angle(self, V, n):
        return np.arccos(1/n) * (180 / np.pi)

    def min_turn_radius_points(self, zero_points):
        min_R_points = self.turn_radius(zero_points[:, 0], zero_points[:, 1])
        points = np.column_stack((zero_points[:, 0], min_R_points))
        return points
    
    def turn_radius(self, V, n):
        return V**2 / (self.g * np.sqrt(n**2 - 1))
    
    def min_time_turn_points(self, zero_points):
        min_t_points = self.turn_time(zero_points[:, 0], zero_points[:, 1])
        points = np.column_stack((zero_points[:, 0], min_t_points))
        return points
    
    def turn_time(self, V, n):
        R = self.turn_radius(V, n)
        return 2 * np.pi * R / V
    
    def sea_bank_limit(self, h):
        """
        Calculate the sea level bank angle limit based on altitude.

        Parameters:
        h (float): Altitude in meters.

        Returns:
        float: Sea level bank angle limit in degrees.
        """
        dihedral_rad = (self.dihedral) * (np.pi / 180)

        half_span = self.span / 2

        if h/half_span >= 1:
            return  np.pi /2 * (180 / np.pi)  # 90 degrees if the altitude is greater than half the span	
        else:
            return (np.arcsin(h / half_span) + dihedral_rad) * (180 / np.pi)
        
    def update_json(self, file_path, h_max=None):
        """
        Update the JSON file with the calculated values.

        Parameters:
        file_path (str): Path to the JSON file.
        """
        data = self.data.data
        zero_points = optimum_turns.max_n_turn_points(h=h)
        
        zero_points_limited = zero_points.copy()
        if h_max:
            max_n = 1/np.cos(np.radians(optimum_turns.sea_bank_limit(h)))
            zero_points_limited[zero_points_limited[:, 1] > max_n, 1] = max_n

        data['outputs']['general']['max_n_turn'] = float(np.max(zero_points_limited[:, 1]))
        data['outputs']['general']['max_bank_angle'] = float(np.max(self.max_bank_angle_points(zero_points_limited)[:, 1]))
        data['outputs']['general']['min_turn_radius'] = float(np.min(self.min_turn_radius_points(zero_points_limited)[:, 1]))
        data['outputs']['general']['min_time_turn'] = float(np.min(self.min_time_turn_points(zero_points_limited)[:, 1]))
        
        self.data.save_design(file_path)
    
if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN
    optimum_turns = OptimumTurns(aircraft_data, mission_type)
    h = aircraft_data.data['inputs']['cruise_altitude']

    optimum_turns.update_json(file_path, h_max=None)

    # zero_points = optimum_turns.max_n_turn_points(h=h)
    # zero_points_limited = zero_points.copy()

    # max_n = 1/np.cos(np.radians(optimum_turns.sea_bank_limit(h)))
    # zero_points_limited[zero_points_limited[:, 1] > max_n, 1] = max_n

    # turn_radii = optimum_turns.min_turn_radius_points(zero_points_limited)
    # max_bank_angles = optimum_turns.max_bank_angle_points(zero_points_limited)
    # min_time_turns = optimum_turns.min_time_turn_points(zero_points_limited)

    # optimum_turns.plot_vs_velocity(zero_points_limited, y_axis_name="Load Factor [-]", color='blue')
    # optimum_turns.plot_vs_velocity(turn_radii, y_axis_name="Min Turn Radius [m]", color='blue')
    # optimum_turns.plot_vs_velocity(max_bank_angles, y_axis_name="Max Bank Angle [deg]", color='blue')
    # optimum_turns.plot_vs_velocity(min_time_turns, y_axis_name="Min Turn Time [s]", color='blue')