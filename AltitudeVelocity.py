import numpy as np
import matplotlib.pyplot as plt
from utils import Data
from ClassIWeightEstimation import ClassI, MissionType
from ISA_Class import ISA

class AltitudeVelocity:
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        self.data = aircraft_data
        self.mission_type = mission_type
        self.dive_speed = self.data.data['requirements']['cruise_speed'] / 0.8

    def calculate_power_required(self, V: float, h: float) -> float:
        """
        Calculate the power required for the aircraft at different velocities.
        """
        qS = 0.5 * ISA(h).rho * V**2 * self.data.data['outputs']['design']['S']
        Cl = self.data.data['outputs']['design']['MTOW'] / (qS)
        Cd = self.data.data['inputs']['Cd0'] + Cl**2 / (np.pi * self.data.data['inputs']['aspect_ratio'] * self.data.data['inputs']['oswald_factor'])

        # print(f"Cl: {Cl}, Cd: {Cd}, q: {q}, Prequired: {Cd * q * V}")
        return Cd*qS * V

    def calculate_power_availabe(self, h: float) -> float:
        """
        Calculate the power available for the aircraft at different velocities.
        Assume power available is constant for propeller engines.
        """
        return self.data.data['inputs']['engine_power'] * ISA(h).rho/ISA(0).rho * 4 # self.data.data['inputs']['n_engines']
    
    def calculate_stall_speed(self, h: float) -> float:
        """
        Calculate stall speed depending on altitude
        """
        denom = 0.5 * ISA(h).rho * self.data.data['inputs']['CLmax_clean'] * self.data.data['outputs']['design']['S']
        V_stall = np.sqrt(self.data.data['outputs']['design']['MTOW']/denom)

        return V_stall
    
    def plot_power_curve(self, h_list: np.array) -> None:
        """
        Plot the power required and available curves.
        """
        plt.figure(figsize=(10, 6))
        color_list = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        for i, h in enumerate(h_list):
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, 100)

            power_required = [self.calculate_power_required(v, h) for v in velocity_range]
            power_available = [self.calculate_power_availabe(h) for _ in velocity_range]

            plt.plot(velocity_range, power_required, label=f'Power Required at {h} m', color=color_list[i])
            plt.plot(velocity_range, power_available, label=f'Power Available at {h} m', color=color_list[i])

        plt.title(f'Power Required vs Power Available at {h} m')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Power (W)')
        plt.legend()
        plt.grid()
        plt.show()

    def calculate_RoC(self, V: float, h: float) -> float:

        RoC = (self.calculate_power_availabe(h) - self.calculate_power_required(V, h))/self.data.data['outputs']['design']['MTOW']
        return RoC
    
    def calculate_max_RoC(self, h: float) -> float:
        
        V_stall = self.calculate_stall_speed(h)
        velocity_range = np.linspace(V_stall, self.dive_speed, 100)

        mps2fpm = 196.85 # metres per second to feet per minute
        RoC = [self.calculate_RoC(v, h) for v in velocity_range]

        max_roc_idx = np.argmax(RoC)
        velocity_at_max_roc = velocity_range[max_roc_idx]

        return max(RoC), velocity_at_max_roc
    
    def plot_RoC_line(self, h_list: np.array) -> None:
        plt.figure(figsize=(10, 6))

        for h in h_list:
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, 100)

            mps2fpm = 196.85 # metres per second to feet per minute
            RoC = [self.calculate_RoC(v, h)*mps2fpm for v in velocity_range]
            
            plt.plot(velocity_range, RoC, label=f'RoC at {h} m')

        plt.title(f'Rate of Climb at {h} m')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('RoC (ft/min)')
        plt.legend()
        plt.grid()
        plt.show()

    def calculate_h_max(self, height_resolution:int=10, RoC_limit:float=0) -> tuple:
        """
        Calculate the maximum height at which the aircraft can maintain climb.
        
        Args:
            height_resolution: Step size in feet for altitude calculations
            RoC_limit: The minimum rate of climb threshold in ft/min
            
        Returns:
            tuple: (h_max, v_optimal)
                h_max: The maximum altitude in feet where RoC first reaches the limit
                v_optimal: The optimal velocity at h_max in knots
        """
        # Binary search approach to find h_max more efficiently
        h_min = 0
        h_max = 20000
        
        # First check if aircraft can climb at all
        roc_min, v_min = self.calculate_max_RoC(h_min)
        if roc_min <= RoC_limit:
            return h_min, v_min
        
        # Check if aircraft can climb beyond our max range
        roc_max, v_max = self.calculate_max_RoC(h_max)
        if roc_max > RoC_limit:
            # Either extend range or return with a note
            return h_max, v_max  # Could raise a warning here
        
        # Binary search to find the zero crossing point
        v_at_h_min = v_min  # Initialize velocity at h_min
        
        while h_max - h_min > height_resolution:
            h_mid = (h_min + h_max) // 2
            roc_at_mid, v_at_mid = self.calculate_max_RoC(h_mid)
            
            if roc_at_mid > RoC_limit:
                h_min = h_mid
                v_at_h_min = v_at_mid  # Update velocity at new h_min
            else:
                h_max = h_mid
        
        # Get the final velocity at h_min
        final_roc, final_velocity = self.calculate_max_RoC(h_min)
        
        return h_min, final_velocity  # Return both altitude and velocity
    
    def calculate_limit_points(self, height_resolution:int=1, RoC_limit:float=0) -> np.array:

        h_max, _ = self.calculate_h_max(height_resolution=height_resolution)
        h_list = np.arange(0, h_max, height_resolution)

        zero_points = []
        stall_points = []
        for h in h_list:
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, 100)
            RoC = np.array([self.calculate_RoC(v, h) for v in velocity_range])
            # Find where RoC crosses RoC_limit (not just zero)
            diff_sign = np.diff(np.sign(RoC - RoC_limit))
            zero_crossings = np.where(diff_sign != 0)[0]
            # Interpolate to find more accurate crossing velocities
            for idx in zero_crossings:
                v1, v2 = velocity_range[idx], velocity_range[idx + 1]
                roc1, roc2 = RoC[idx], RoC[idx + 1]
                if roc2 != roc1:
                    v_zero = v1 + (RoC_limit - roc1) * (v2 - v1) / (roc2 - roc1)
                else:
                    v_zero = v1
                zero_points.append((v_zero, h))

            stall_points.append((V_stall, h))  # Add stall speed as a zero point

        if zero_points:
            min_zero_velocity = min([v for v, _ in zero_points])
            stall_points = [pt for pt in stall_points if pt[0] < min_zero_velocity]

        return zero_points, stall_points
    
    def calculate_Vy(self) -> float:
        """
        
        """
        
        max_Vy_points = []
        for RoC_limit in np.arange(0, 10, 0.02):
            h_max, v_opt = self.calculate_h_max(height_resolution=1, RoC_limit=RoC_limit)
            max_Vy_points.append((RoC_limit, h_max, v_opt))

        return max_Vy_points

    def plot_limit_points(self, height_resolution:int=1) -> None:
        zero_points, stall_points = self.calculate_limit_points(height_resolution=height_resolution, RoC_limit=0)
        zero_points = np.array(zero_points)
        stall_points = np.array(stall_points)
        # Find the point with the maximum altitude among both lists
        max_zero = zero_points[np.argmax(zero_points[:, 1])] if len(zero_points) > 0 else None
        max_stall = stall_points[np.argmax(stall_points[:, 1])] if len(stall_points) > 0 else None

        # Determine which point has the highest altitude
        if max_zero is not None and (max_stall is None or max_zero[1] >= max_stall[1]):
            h_max_point = max_zero
        else:
            h_max_point = max_stall

        zero_points = zero_points[zero_points[:, 0].argsort()]
        stall_points = stall_points[stall_points[:, 0].argsort()]

        plt.figure(figsize=(10, 6))
        plt.plot(zero_points[:, 0], zero_points[:, 1], color='k', label='Thrust limit')
        plt.plot(stall_points[:, 0], stall_points[:, 1], color='r', label='Stall limit')

        if h_max_point is not None:
            plt.scatter(h_max_point[0], h_max_point[1], color='b', s=10, marker='o')
            plt.annotate(
                f"h_max = {h_max_point[1]:.0f} m",
                (h_max_point[0], h_max_point[1]),
                textcoords="offset points",
                xytext=(30, -30),  # bottom right
                ha='left',
                va='top',
                color='b',
                fontsize=10,
                arrowprops=dict(arrowstyle="->", color='b')
            )
            
        max_Vy_points = self.calculate_Vy()
        # Extract velocities and altitudes for plotting the Vy curve
        vy_velocities = [point[2] for point in max_Vy_points]
        vy_altitudes = [point[1] for point in max_Vy_points]
        plt.plot(vy_velocities, vy_altitudes, color='g', linestyle='--', label='Vy curve')

        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Altitude (m)')
        plt.legend()
        plt.grid()
        plt.show()


if __name__ == "__main__":
    # Example usage
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)
    
    h = np.array([0, 3048])  # Example altitude in meters
    # altitude_velocity.plot_power_curve(h)
    # altitude_velocity.plot_RoC_line(h)
    altitude_velocity.plot_limit_points(height_resolution=1)

    print(f"Max RoC at h = 0 is {altitude_velocity.calculate_max_RoC(0)[0] * 196.85} ft/min")
    print(f"Max RoC at h = 3048 is {altitude_velocity.calculate_max_RoC(3048)[0] * 196.85} ft/min")
    # print(f"Service Ceiling: {altitude_velocity.calculate_h_max(height_resolution=1)} metres")