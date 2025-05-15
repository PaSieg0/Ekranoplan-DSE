import numpy as np
import matplotlib.pyplot as plt
from utils import Data
from ClassIWeightEstimation import ClassI, MissionType
from ISA_Class import ISA
from functools import lru_cache

class AltitudeVelocity:
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        self.data = aircraft_data
        self.mission_type = mission_type
        self.dive_speed = self.data.data['requirements']['cruise_speed'] / 0.8
        
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
        
        # Pre-calculate the mps to fpm conversion factor
        self.mps2fpm = 196.85  # metres per second to feet per minute

    @lru_cache(maxsize=128)
    def _get_density(self, h: float) -> float:
        """Cached method to get air density at a given altitude."""
        return ISA(h).rho

    def calculate_power_required(self, V: float, h: float) -> float:
        """
        Calculate the power required for the aircraft at different velocities.
        """
        rho = self._get_density(h)
        q = 0.5 * rho * V**2
        qS = q * self._S
        Cl = self._mtow / qS
        Cd = self._Cd0 + Cl**2 * self._k
        
        return Cd * qS * V

    def calculate_power_availabe(self, h: float) -> float:
        """
        Calculate the power available for the aircraft at different velocities.
        Assume power available is constant for propeller engines.
        """
        return self._engine_power * (self._get_density(h) / self._sea_level_density)**0.75 * 4

    @lru_cache(maxsize=128)
    def calculate_stall_speed(self, h: float) -> float:
        """
        Calculate stall speed depending on altitude
        """
        denom = 0.5 * self._get_density(h) * self._CLmax * self._S
        return np.sqrt(self._mtow / denom)
    
    def plot_power_curve(self, h_list: np.array) -> None:
        """
        Plot the power required and available curves.
        """
        plt.figure(figsize=(10, 6))
        color_list = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        
        for i, h in enumerate(h_list):
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, 100)
            
            # Vectorized calculations instead of list comprehensions
            power_required = np.array([self.calculate_power_required(v, h) for v in velocity_range])
            power_available = np.full_like(velocity_range, self.calculate_power_availabe(h))
            
            plt.plot(velocity_range, power_required, label=f'Power Required at {h} m', color=color_list[i % len(color_list)])
            plt.plot(velocity_range, power_available, label=f'Power Available at {h} m', color=color_list[i % len(color_list)])

        plt.title(f'Power Required vs Power Available')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Power (W)')
        plt.legend()
        plt.grid()
        plt.show()

    def calculate_RoC(self, V: float, h: float) -> float:
        """Calculate Rate of Climb at a given velocity and altitude."""
        return (self.calculate_power_availabe(h) - self.calculate_power_required(V, h)) / self._mtow
    
    def calculate_RoC_vectorized(self, velocities: np.ndarray, h: float) -> np.ndarray:
        """Vectorized version of calculate_RoC for multiple velocities at once."""
        power_available = self.calculate_power_availabe(h)
        power_required = np.array([self.calculate_power_required(v, h) for v in velocities])
        return (power_available - power_required) / self._mtow
    
    def calculate_max_RoC(self, h: float) -> tuple:
        """Find the maximum Rate of Climb and corresponding velocity at a given altitude."""
        V_stall = self.calculate_stall_speed(h)
        velocity_range = np.linspace(V_stall, self.dive_speed, 100)
        
        RoC_values = self.calculate_RoC_vectorized(velocity_range, h)
        max_roc_idx = np.argmax(RoC_values)
        
        return RoC_values[max_roc_idx], velocity_range[max_roc_idx]
    
    def plot_RoC_line(self, h_list: np.array) -> None:
        plt.figure(figsize=(10, 6))

        for h in h_list:
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, 100)
            
            # Vectorized calculation
            RoC = self.calculate_RoC_vectorized(velocity_range, h) * self.mps2fpm
            
            plt.plot(velocity_range, RoC, label=f'RoC at {h} m')

        plt.title('Rate of Climb vs Velocity')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('RoC (ft/min)')
        plt.legend()
        plt.grid()
        plt.show()

    def calculate_h_max(self, height_resolution:float=1, RoC_limit:float=0) -> tuple:
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
        while h_max - h_min > height_resolution:
            h_mid = (h_min + h_max) // 2
            roc_at_mid, v_at_mid = self.calculate_max_RoC(h_mid)
            
            if roc_at_mid > RoC_limit:
                h_min = h_mid
            else:
                h_max = h_mid
        
        # Get the final velocity at h_min
        final_roc, final_velocity = self.calculate_max_RoC(h_min)
        
        return h_min, final_velocity
    
    def calculate_limit_points(self, height_resolution:float=1, RoC_limit:float=0) -> tuple:
        """Calculate zero-RoC points and stall points for plotting the flight envelope."""
        # First determine maximum altitude to avoid unnecessary calculations
        h_max, _ = self.calculate_h_max(height_resolution=height_resolution, RoC_limit=RoC_limit)
        h_list = np.arange(0, h_max + height_resolution, height_resolution)

        zero_points = []
        stall_points = []
        
        for h in h_list:
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, 100)
            
            # Use vectorized calculation
            RoC = self.calculate_RoC_vectorized(velocity_range, h)
            
            # Find where RoC crosses RoC_limit
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

            stall_points.append((V_stall, h))

        # Convert to numpy arrays for faster processing if there are points
        if zero_points:
            zero_points = np.array(zero_points)
            min_zero_velocity = np.min(zero_points[:, 0]) if len(zero_points) > 0 else float('inf')
            stall_points = np.array([(v, h) for v, h in stall_points if v < min_zero_velocity])
        else:
            stall_points = np.array(stall_points)

        return zero_points, stall_points
    
    def calculate_Vy(self, h_max:float, height_resolution:float=1) -> list:
        """Calculate best climb speeds (Vy) at different altitude points."""
        # Implementing a more efficient approach
        h_limits = np.arange(0, h_max, height_resolution)
        max_Vy_points = []
        
        for h in h_limits:
            RoC, v_opt = self.calculate_max_RoC(h)
            max_Vy_points.append((RoC, h, v_opt))  # Append altitude, max RoC, and velocity

        return max_Vy_points

    def plot_limit_points(self, height_resolution:float=1) -> None:
        """Plot the flight envelope with thrust limit, stall limit, and Vy curve."""
        zero_points, stall_points = self.calculate_limit_points(height_resolution=height_resolution, RoC_limit=0)
        
        plt.figure(figsize=(10, 6))
        
        # Convert to numpy arrays if not already
        if isinstance(zero_points, list) and zero_points:
            zero_points = np.array(zero_points)
        if isinstance(stall_points, list) and stall_points:
            stall_points = np.array(stall_points)
        
        # Plot only if points exist
        if len(zero_points) > 0:
            # Sort by velocity for proper line plotting
            idx = np.argsort(zero_points[:, 0])
            zero_points = zero_points[idx]
            plt.plot(zero_points[:, 0], zero_points[:, 1], color='k', label='Thrust limit')
            
            # Find max altitude point on the zero RoC curve
            max_zero_idx = np.argmax(zero_points[:, 1])
            max_zero = zero_points[max_zero_idx]
        else:
            max_zero = None
            
        if len(stall_points) > 0:
            # Sort by velocity for proper line plotting
            idx = np.argsort(stall_points[:, 0])
            stall_points = stall_points[idx]
            plt.plot(stall_points[:, 0], stall_points[:, 1], color='r', label='Stall limit')
            
            # Find max altitude point on the stall curve
            max_stall_idx = np.argmax(stall_points[:, 1])
            max_stall = stall_points[max_stall_idx]
        else:
            max_stall = None

        # Determine which point has the highest altitude
        if max_zero is not None and (max_stall is None or max_zero[1] >= max_stall[1]):
            h_max_point = max_zero
        elif max_stall is not None:
            h_max_point = max_stall
        else:
            h_max_point = None

        # Annotate the maximum altitude point
        if h_max_point is not None:
            plt.scatter(h_max_point[0], h_max_point[1], color='b', s=10, marker='o')
            plt.annotate(
                f"h_max = {h_max_point[1]:.0f} m",
                (h_max_point[0], h_max_point[1]),
                textcoords="offset points",
                xytext=(30, -30),
                ha='left',
                va='top',
                color='b',
                fontsize=10,
                arrowprops=dict(arrowstyle="->", color='b')
            )
        
        # Calculate and plot the Vy curve    
        max_Vy_points = self.calculate_Vy(h_max=h_max_point[1], height_resolution=height_resolution)
        if max_Vy_points:
            # Extract velocities and altitudes for plotting
            vy_velocities = [point[2] for point in max_Vy_points]
            vy_altitudes = [point[1] for point in max_Vy_points]
            plt.plot(vy_velocities, vy_altitudes, color='g', linestyle='--', label='Vy curve')

        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Altitude (m)')
        plt.title('Aircraft Flight Envelope')
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