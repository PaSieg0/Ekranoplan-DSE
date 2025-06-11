import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType, plt
from functools import lru_cache
from tqdm import tqdm
from scipy.optimize import minimize_scalar

class AltitudeVelocity:
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        self.data = aircraft_data
        self.mission_type = mission_type
        self.dive_speed = self.data.data['requirements']['cruise_speed'] / 0.6
        
        # Cache frequently used values to avoid repeated lookups
        if mission_type == MissionType.FERRY:
            self._mtow = self.data.data['outputs']['design']['MTOW']-self.data.data['requirements']['design_payload']*9.81
        else:
            self._mtow = self.data.data['outputs']['design']['MTOW']
        self._current_weight = self._mtow

        self._S = self.data.data['outputs']['design']['S']
        self._Cd0 = self.data.data['inputs']['Cd0']
        self._AR = self.data.data['inputs']['aspect_ratio']
        self._e = self.data.data['inputs']['oswald_factor']
        self._CLmax = self.data.data['inputs']['CLmax_clean']
        self._engine_power = self.data.data['inputs']['engine']['engine_power']
        self._prop_efficiency = self.data.data['inputs']['prop_efficiency']
        self._sea_level_density = ISA(0).rho
        self._k = 1 / (np.pi * self._AR * self._e)  # Induced drag factor
        self.velocity_steps = 2000  # Number of velocity steps for calculations
        self.height_steps = 7000  # Number of height steps for calculations
        self.g = 9.81  # Gravitational acceleration in m/s^2
        
        # Pre-calculate the mps to fpm conversion factor
        self.mps2fpm = 196.85  # metres per second to feet per minute

    @lru_cache(maxsize=128)
    def _get_density(self, h: float) -> float:
        """Cached method to get air density at a given altitude."""
        return ISA(h).rho
    
    def equivalent_air_speed(self, V_tas: float, h: float) -> float:
        """
        Calculate the equivalent air speed at a given altitude.
        """
        rho = self._get_density(h)
        return V_tas * np.sqrt(rho/self._sea_level_density)

    def true_air_speed(self, V_ias: float, h: float) -> float:
        """
        Calculate the true air speed at a given altitude.
        """
        rho = self._get_density(h)
        return V_ias * np.sqrt(self._sea_level_density/rho)

    def calculate_power_required(self, V: float, h: float, RoC: float=0, W=None) -> float:
        """
        Calculate the power required for the aircraft at different velocities.
        """
        if W is None:
            W = self._current_weight

        rho = self._get_density(h)
        q = 0.5 * rho * V**2
        qS = q * self._S
        Cl = W / qS
        Cd = self._Cd0 + Cl**2 * self._k
        
        return Cd * qS * V + W * RoC

    def calculate_power_available(self, h: float) -> float:
        """
        Calculate the power available for the aircraft at different velocities.
        Assume power available is constant for propeller engines.
        """
        return self._engine_power * (self._get_density(h) / self._sea_level_density)**0.70 * 6 * self._prop_efficiency
    
    def calculate_drag(self, V: float, h: float, W: float = None) -> float:
        return self.calculate_power_required(V, h, W=W)/V

    def calculate_thust(self, V: float, h: float) -> float:
        return self.calculate_power_available(h)/V
    
    def calculate_stall_speed(self, h: float) -> float:
        """
        Calculate stall speed depending on altitude
        """
        denom = 0.5 * self._get_density(h) * self._CLmax * self._S
        return np.sqrt(self._current_weight / denom)
    
    def plot_power_curve(self, h_list: np.array) -> None:
        """
        Plot the power required and available curves.
        """
        plt.figure(figsize=(10, 6))
        color_list = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        
        for i, h in enumerate(h_list):
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps)
            
            # Vectorized calculations instead of list comprehensions
            power_required = np.array([self.calculate_power_required(v, h) for v in velocity_range])
            power_available = np.full_like(velocity_range, self.calculate_power_available(h))
            
            plt.plot(velocity_range, power_required, label=f'Power Required at {h} m', color=color_list[i % len(color_list)])
            plt.plot(velocity_range, power_available, label=f'Power Available at {h} m', color=color_list[i % len(color_list)])

        plt.title(f'Power Required and Power Available vs Velocity')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Power (W)')
        plt.legend()
        plt.grid()
        plt.show()

    def plot_force_curve(self, h_list: np.array, n_list: float = [1], show=True) -> None:
        """
        Plot the drag and thrust available curves for different load factors.
        """
        plt.figure(figsize=(10, 6))
        color_list = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        initial_weight = self._current_weight  # Store initial weight

        for n_idx, n in enumerate(n_list):
            self._current_weight = initial_weight * n  # Adjust weight based on load factor

            for i, h in enumerate(h_list):
                V_stall = self.calculate_stall_speed(h)
                velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps)

                drag = np.array([self.calculate_drag(v, h) for v in velocity_range])
                thrust = np.array([self.calculate_thust(v, h) for v in velocity_range])

                label_drag = f'Drag at {h} m, n={n}'
                label_thrust = f'Thrust Available at {h} m, n={n}'
                color = color_list[(i + n_idx * len(h_list)) % len(color_list)]

                plt.plot(velocity_range, drag, label=label_drag, color=color, linestyle='-')
                plt.plot(velocity_range, thrust, label=label_thrust, color=color, linestyle='--')

        self._current_weight = initial_weight  # Reset weight to original value

        plt.title('Thrust and Drag vs Velocity')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Force (N)')
        plt.legend()
        plt.grid()
        if show:
            plt.show()

    def calculate_RoC(self, V: float, h: float, W: float = None) -> float:
        """Calculate Rate of Climb at a given velocity and altitude."""
        return (self.calculate_power_available(h) - self.calculate_power_required(V, h, W=W)) / self._current_weight
    
    def calculate_AoC(self, V: float, h: float, W: float = None) -> float:
        """Calculate Angle of Climb at a given velocity and altitude."""
        return np.arcsin(self.calculate_RoC(V, h, W=W) / V)
    
    def calculate_RoC_vectorized(self, velocities: np.ndarray, h: float) -> np.ndarray:
        """Vectorized version of calculate_RoC for multiple velocities at once."""
        power_available = self.calculate_power_available(h)
        power_required = np.array([self.calculate_power_required(v, h) for v in velocities])
        return (power_available - power_required) / self._current_weight
    
    def calculate_AoC_vectorized(self, velocities: np.ndarray, h: float) -> np.ndarray:
        """Vectorized version of calculate_AoC for multiple velocities at once."""
        thrust_available = np.array([self.calculate_power_available(h)/v for v in velocities])
        thrust_required = np.array([self.calculate_power_required(v, h)/v for v in velocities])
        return np.arcsin((thrust_available - thrust_required) / self._current_weight)
    
    def calculate_max_RoC(self, h: float) -> tuple:
        """Find the maximum Rate of Climb and corresponding velocity at a given altitude."""
        V_stall = self.calculate_stall_speed(h)

        k1 = 0
        k2 = self._k
        
        Cl_opt = (k1 + np.sqrt(k1**2+12*k2*self._Cd0))/(2*k2) # Optimal lift coefficient
        V_opt = np.sqrt(2 * self._current_weight / (self._get_density(h) * self._S * Cl_opt))  # Optimal velocity

        if V_opt < V_stall:
            V_opt = V_stall
        elif V_opt > self.dive_speed:
            V_opt = self.dive_speed
        
        RoC = self.calculate_RoC(V_opt, h)
        return RoC, V_opt
    
    def calculate_max_AoC(self, h: float) -> tuple:
        """Find the maximum Angle of Climb and corresponding velocity at a given altitude."""

        V_stall = self.calculate_stall_speed(h)

        def negative_aoc(V):
            thrust_available = self.calculate_power_available(h) / V
            thrust_required = self.calculate_power_required(V, h) / V
            return -np.arcsin((thrust_available - thrust_required) / self._current_weight)

        result = minimize_scalar(negative_aoc, bounds=(V_stall, self.dive_speed), method='bounded')
        max_aoc_idx = None  # Not needed with minimize_scalar
        AoC_values = [-result.fun]  # Convert back to positive value
        velocity_range = [result.x]  # Optimal velocity
        max_aoc_idx = 0  # Index for the single optimal point
        
        return AoC_values[max_aoc_idx], velocity_range[max_aoc_idx]
    
    def calculate_min_RoD(self, h: float) -> tuple:
        """Find the min Rate of Descent and corresponding velocity at a given altitude."""
        V_stall = self.calculate_stall_speed(h)

        k1 = 0
        k2 = self._k
        
        Cl_opt = (k1 + np.sqrt(k1**2+12*k2*self._Cd0))/(2*k2) # Optimal lift coefficient
        V_opt = np.sqrt(2 * self._current_weight / (self._get_density(h) * self._S * Cl_opt))  # Optimal velocity

        if V_opt < V_stall:
            V_opt = V_stall
        elif V_opt > self.dive_speed:
            V_opt = self.dive_speed
           
        RoD = -self.calculate_power_required(V_opt, h, RoC=0)/self._current_weight
        
        return RoD, V_opt
    
    def calculate_min_AoD(self, h: float) -> tuple:
        """Find the maximum Angle of Climb and corresponding velocity at a given altitude."""
        V_stall = self.calculate_stall_speed(h)
        velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps)
        
        AoD_values = [np.arcsin(-self.calculate_power_required(V, h, RoC=0)/V/self._current_weight) for V in velocity_range]
        max_aoc_idx = np.argmax(AoD_values)
        
        return AoD_values[max_aoc_idx], velocity_range[max_aoc_idx]
    
    def plot_RoC_line(self, h_list: np.array) -> None:
        plt.figure(figsize=(10, 6))

        for h in h_list:
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed, self.velocity_steps)
            
            # Vectorized calculation
            RoC = self.calculate_RoC_vectorized(velocity_range, h) * self.mps2fpm
            
            plt.plot(velocity_range, RoC, label=f'RoC at {h} m')

        plt.title('Rate of Climb vs Velocity')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('RoC (ft/min)')
        plt.legend()
        plt.grid()
        plt.show()

    def calculate_h_max(self, RoC_limit:float=0) -> tuple:
        """
        Calculate the maximum height at which the aircraft can maintain climb.
        
        Args:
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
        while h_max - h_min > 1/self.height_steps:
            h_mid = (h_min + h_max) / 2
            roc_at_mid, v_at_mid = self.calculate_max_RoC(h_mid)
            
            if roc_at_mid > RoC_limit:
                h_min = h_mid
            else:
                h_max = h_mid
        
        # Get the final velocity at h_min
        final_roc, final_velocity = self.calculate_max_RoC(h_min)
        
        return h_min, final_velocity
    
    def calculate_Vy(self, h_max:float) -> list:
        """Calculate best climb speeds (Vy) at different altitude points."""
        # Implementing a more efficient approach
        h_limits = np.linspace(0, h_max, self.height_steps)
        max_Vy_points = []
        
        for h in h_limits:
            RoC, v_opt = self.calculate_max_RoC(h)
            max_Vy_points.append((RoC, h, v_opt))  # Append altitude, max RoC, and velocity

        return max_Vy_points
    
    def calculate_Vx(self, h_max:float) -> list:
        """Calculate best cruise speeds (Vx) at different altitude points."""
        # Implementing a more efficient approach
        h_limits = np.linspace(0, h_max, self.height_steps)
        max_Vx_points = []
        
        for h in h_limits:
            AoC, v_opt = self.calculate_max_AoC(h)
            max_Vx_points.append((AoC, h, v_opt))

        return max_Vx_points
    
    def calculate_limit_points(self, airspeed_type='true', altitude_units='meters') -> tuple:
        # First determine maximum altitude to avoid unnecessary calculations
        h_max, _ = self.calculate_h_max(RoC_limit=0)
        h_list = np.linspace(0, h_max, self.height_steps)

        zero_points = []
        stall_points = []
        
        # Conversion factor for altitude
        self.m_to_ft = 3.28084 if altitude_units == 'feet' else 1.0

        for h in tqdm(h_list, desc="Calculating envelope"):
            V_stall = self.calculate_stall_speed(h)
            velocity_range = np.linspace(V_stall, self.dive_speed*2, self.velocity_steps)

            # Use vectorized calculation
            RoC = self.calculate_RoC_vectorized(velocity_range, h)

            # Find where RoC crosses RoC_limit
            diff_sign = np.diff(np.sign(RoC - 0))
            zero_crossings = np.where(diff_sign != 0)[0]

            # Interpolate to find more accurate crossing velocities
            for idx in zero_crossings:
                v1, v2 = velocity_range[idx], velocity_range[idx + 1]
                roc1, roc2 = RoC[idx], RoC[idx + 1]
                if roc2 != roc1:
                    v_zero = v1 + (0 - roc1) * (v2 - v1) / (roc2 - roc1)
                else:
                    v_zero = v1
                
                # Convert to equivalent airspeed if requested
                if airspeed_type == 'equivalent':
                    v_zero = self.equivalent_air_speed(v_zero, h)
                
                zero_points.append((v_zero, h * self.m_to_ft))

            # Convert stall speed to equivalent airspeed if requested
            if airspeed_type == 'equivalent':
                V_stall = self.equivalent_air_speed(V_stall, h)
            

            stall_points.append((V_stall, h * self.m_to_ft))

        # Convert to numpy arrays for faster processing if there are points
        if zero_points:
            zero_points = np.array(zero_points)
            min_zero_velocity = np.min(zero_points[:, 0]) if len(zero_points) > 0 else float('inf')
            stall_points = np.array([(v, h) for v, h in stall_points if v < min_zero_velocity])
        else:
            stall_points = np.array(stall_points)

        # Sort by velocity for proper line plotting
        idx = np.argsort(zero_points[:, 0])
        zero_points = zero_points[idx]
        idx = np.argsort(stall_points[:, 0])
        stall_points = stall_points[idx]

        return zero_points, stall_points

    def plot_limit_points(
        self, 
        zero_points: np.ndarray, 
        stall_points: np.ndarray,
        airspeed_type='true',
        altitude_units='meters',
        plot=('thrust_limit', 'stall_limit', 'Vy', 'Vx', 'max_h', 'max_RoC', 'max_AoC', 'energy_height', 'roc_contours')
    ) -> None:
        """
        Plot the flight envelope with thrust limit, stall limit, Vy/Vx/Vz curves, and RoC contours.

        Args:
            airspeed_type (str): Type of airspeed to plot - 'true' for true airspeed (default) 
                                 , 'equivalent' for equivalent airspeed
                                 , 'energy' for energy height airspeed (x-axis = V^2/(2g))
            altitude_units (str): Units for altitude - 'meters' (default) or 'feet'
            plot (tuple): Which lines/points to plot. Options include:
                'thrust_limit', 'stall_limit', 'Vy', 'Vx', 'max_h', 'max_RoC', 'max_AoC', 'energy_height', 'roc_contours'
        """
        plt.figure(figsize=(10, 7))

        # Helper for x-axis transform
        def x_transform(V):
            if airspeed_type == 'energy':
                return np.array(V)**2 / (2 * self.g)
            return V

        # Plot thrust limit (zero RoC curve)
        if 'thrust_limit' in plot and len(zero_points) > 0:
            plt.plot(x_transform(zero_points[:, 0]), zero_points[:, 1], color='k', label='Thrust limit')
            max_zero_idx = np.argmax(zero_points[:, 1])
            max_zero = zero_points[max_zero_idx]
        else:
            max_zero = None

        # Plot stall limit
        if 'stall_limit' in plot and len(stall_points) > 0:
            plt.plot(x_transform(stall_points[:, 0]), stall_points[:, 1], color='r', label='Stall limit')
            max_stall_idx = np.argmax(stall_points[:, 1])
            max_stall = stall_points[max_stall_idx]
        else:
            max_stall = None

        # Fill between curves for envelope
        if 'thrust_limit' in plot and len(zero_points) > 0:
            idx = np.argsort(zero_points[:, 0])
            zero_points_sorted = zero_points[idx]
            plt.fill_between(
                x_transform(zero_points_sorted[:, 0]),
                0,
                zero_points_sorted[:, 1],
                color='lightgreen',
                alpha=0.3,
                edgecolor='none'
            )
        if 'stall_limit' in plot and len(stall_points) > 0:
            idx = np.argsort(stall_points[:, 0])
            stall_points_sorted = stall_points[idx]
            plt.fill_between(
                x_transform(stall_points_sorted[:, 0]),
                0,
                stall_points_sorted[:, 1],
                color='lightgreen',
                alpha=0.3,
                edgecolor='none'
            )

        # Determine which point has the highest altitude
        if max_zero is not None and (max_stall is None or max_zero[1] >= max_stall[1]):
            h_max_point = max_zero
        elif max_stall is not None:
            h_max_point = max_stall
        else:
            h_max_point = None

        # Annotate the maximum altitude point
        if 'max_h' in plot and h_max_point is not None:
            plt.scatter(x_transform(h_max_point[0]), h_max_point[1], color='b', s=10, marker='o')
            altitude_label = f"h_max = {h_max_point[1]:.0f} {'ft' if altitude_units == 'feet' else 'm'}"
            plt.annotate(
                altitude_label,
                (x_transform(h_max_point[0]), h_max_point[1]),
                textcoords="offset points",
                xytext=(30, -30),
                ha='left',
                va='top',
                color='b',
                fontsize=10,
                arrowprops=dict(arrowstyle="->", color='b')
            )

        # Calculate and plot the Vx curve    
        if 'Vx' in plot and h_max_point is not None:
            print(f"Calculating Vx...")
            max_Vx_points = self.calculate_Vx(h_max=h_max_point[1] / self.m_to_ft)
            if max_Vx_points:
                vx_velocities = [point[2] for point in max_Vx_points]
                vx_altitudes = [point[1] * self.m_to_ft for point in max_Vx_points]
                if airspeed_type == 'equivalent':
                    vx_velocities = [self.equivalent_air_speed(v, h/self.m_to_ft) for v, h in zip(vx_velocities, vx_altitudes)]
                plt.plot(x_transform(vx_velocities), vx_altitudes, color='b', linestyle='--', label='Vx curve')

        # Calculate and plot the Vy curve    
        if 'Vy' in plot and h_max_point is not None:
            print(f"Calculating Vy...")
            max_Vy_points = self.calculate_Vy(h_max=h_max_point[1] / self.m_to_ft)
            if max_Vy_points:
                vy_velocities = [point[2] for point in max_Vy_points]
                vy_altitudes = [point[1] * self.m_to_ft for point in max_Vy_points]
                if airspeed_type == 'equivalent':
                    vy_velocities = [self.equivalent_air_speed(v, h/self.m_to_ft) for v, h in zip(vy_velocities, vy_altitudes)]
                plt.plot(x_transform(vy_velocities), vy_altitudes, color='g', linestyle='--', label='Vy curve')

        # Plot contours of energy height
        if len(zero_points) > 0 and ('thrust_limit' in plot or 'stall_limit' in plot) and 'energy_height' in plot:
            v_min = min(np.min(zero_points[:, 0]), np.min(stall_points[:, 0]))
            v_max = max(np.max(zero_points[:, 0]), np.max(stall_points[:, 0]))
            h_min = 0
            h_max_plot = max(np.max(zero_points[:, 1]), np.max(stall_points[:, 1]))
            v_grid = np.linspace(v_min, v_max, 100)
            h_grid = np.linspace(h_min, h_max_plot, 100)
            V, H = np.meshgrid(v_grid, h_grid)
            H_m = H / self.m_to_ft if altitude_units == 'feet' else H
            E = self.energy_height(V, H_m)
            if airspeed_type == 'energy':
                X = V**2 / (2 * self.g)
                cs = plt.contour(X, H, E, levels=10, colors='gray', linestyles='dotted', linewidths=1)
            else:
                cs = plt.contour(x_transform(V), H, E, levels=10, colors='gray', linestyles='dotted', linewidths=1)
            plt.clabel(cs, inline=True, fontsize=8, fmt="%.0f")

        # Plot contours of RoC (only for RoC > 0)
        if len(zero_points) > 0 and ('roc_contours' in plot):
            v_min = min(np.min(zero_points[:, 0]), np.min(stall_points[:, 0]))
            v_max = max(np.max(zero_points[:, 0]), np.max(stall_points[:, 0]))
            h_min = 0
            h_max_plot = max(np.max(zero_points[:, 1]), np.max(stall_points[:, 1]))
            v_grid = np.linspace(v_min, v_max, self.velocity_steps)
            h_grid = np.linspace(h_min, h_max_plot, self.height_steps)
            V, H = np.meshgrid(v_grid, h_grid)
            H_m = H / self.m_to_ft if altitude_units == 'feet' else H
            # Compute RoC at each grid point
            RoC = np.zeros_like(V)
            for i in tqdm(range(H.shape[0]), desc="Calculating RoC contours"):
                for j in range(H.shape[1]):
                    RoC[i, j] = self.calculate_RoC(V[i, j], H_m[i, j]) * self.mps2fpm  # ft/min
            if airspeed_type == 'energy':
                X = V**2 / (2 * self.g)
                cs_roc = plt.contour(X, H, RoC, levels=10, colors='blue', linestyles='solid', linewidths=1)
            else:
                cs_roc = plt.contour(x_transform(V), H, RoC, levels=10, colors='blue', linestyles='solid', linewidths=1)
            plt.clabel(cs_roc, inline=True, fontsize=8, fmt="%.0f ft/min")

        # Annotate key points, max RoC at h = 0 and max AoC at h = 0
        max_roc, v_max_roc = self.calculate_max_RoC(0)
        max_aoc, v_max_aoc = self.calculate_max_AoC(0)
        ax = plt.gca()
        if 'max_RoC' in plot:
            ax.scatter(x_transform(v_max_roc), 0, color='orange', s=10, marker='o')
            ax.annotate(
                f"Max RoC:\n {max_roc * self.mps2fpm:.0f} ft/min",
                (x_transform(v_max_roc), 0),
                textcoords="offset points",
                xytext=(60, 10),
                ha='center',
                fontsize=11,
                arrowprops=dict(arrowstyle="->", color='orange', lw=1)
            )
        if 'max_AoC' in plot:
            ax.scatter(x_transform(v_max_aoc), 0, color='purple', s=10, marker='o')
            ax.annotate(
                f"Max AoC:\n {max_aoc * 180/np.pi:.1f} degrees",
                (x_transform(v_max_aoc), 0),
                textcoords="offset points",
                xytext=(50, 30),
                ha='center',
                fontsize=11,
                arrowprops=dict(arrowstyle="->", color='purple', lw=1)
            )

        # Plot horizontal line at h=10000 ft with velocity range
        if altitude_units == 'feet':
            h_10000 = 10000
        else:
            h_10000 = 10000 / self.m_to_ft  # Convert to meters if needed
        
        # Calculate stall speed at 10000 ft
        V_stall_10k = self.calculate_stall_speed(h_10000 / self.m_to_ft if altitude_units == 'feet' else h_10000)
        
        # Find thrust limited speed at 10000 ft (where RoC = 0)
        h_actual = h_10000 / self.m_to_ft if altitude_units == 'feet' else h_10000
        velocity_range = np.linspace(V_stall_10k, self.dive_speed, self.velocity_steps)
        RoC_values = self.calculate_RoC_vectorized(velocity_range, h_actual)
        
        # Find where RoC crosses zero (thrust limited speed)
        zero_crossings = np.where(np.diff(np.sign(RoC_values)))[0]
        if len(zero_crossings) > 0:
            # Interpolate to find accurate thrust limited speed
            idx = zero_crossings[-1]  # Take the last crossing (max speed)
            v1, v2 = velocity_range[idx], velocity_range[idx + 1]
            roc1, roc2 = RoC_values[idx], RoC_values[idx + 1]
            V_max_10k = v1 + (0 - roc1) * (v2 - v1) / (roc2 - roc1) if roc2 != roc1 else v1
        else:
            V_max_10k = self.dive_speed  # Fallback to dive speed if no zero crossing found
        
        # Convert to equivalent airspeed if needed
        if airspeed_type == 'equivalent':
            V_stall_10k = self.equivalent_air_speed(V_stall_10k, h_actual)
            V_max_10k = self.equivalent_air_speed(V_max_10k, h_actual)
        
        # Plot horizontal line from stall speed to thrust limited speed at 10000 ft
        plt.plot([x_transform(V_stall_10k), x_transform(V_max_10k)], [h_10000, h_10000], 
            color='gray', linestyle='--', alpha=0.7, linewidth=2, label='h = 10,000 ft')

        # Set appropriate axis labels based on airspeed type and altitude units
        if airspeed_type == 'equivalent':
            plt.xlabel('Equivalent Airspeed (m/s)')
        elif airspeed_type == 'energy':
            plt.xlabel('Energy Height Airspeed $V^2/(2g)$ (m)')
        else:
            plt.xlabel('True Airspeed (m/s)')
        if altitude_units == 'feet':
            plt.ylabel('Altitude (ft)')
        else:
            plt.ylabel('Altitude (m)')
        plt.legend()
        plt.grid(True, which='both')
        plt.show()
    
    def calculate_t_climb(self, h_start: float, h_end: float, V_ias: float=None) -> float:
        """
        Calculate the time to climb to a given altitude.
        """
        # Euler integration using max RoC from h_start to h_end
        dt = 0.01  # time step in seconds
        h = h_start
        t = 0.0
        while h < h_end:
            if V_ias is None:
                roc, V_tas = self.calculate_max_RoC(h)
            else:
                V_tas = self.true_air_speed(V_ias, h)
                roc = self.calculate_RoC(V_tas, h)
            if roc <= 0:
                return np.inf
            dh = roc * dt  # roc in m/s
            h += dh
            t += dt
        return t

    def calculate_distance_climb(self, h_start: float, h_end: float) -> float:
        """
        Calculate the horizontal distance to climb to a given altitude.
        """
        # Euler integration using max AoC from h_start to h_end
        dt = 1.0  # time step in seconds
        h = h_start
        distance = 0.0
        
        while h < h_end:
            aoc, V = self.calculate_max_AoC(h)
            roc = self.calculate_RoC(V, h)
            
            if roc <= 0:
                break  # cannot climb further
                
            # Calculate horizontal speed component
            # At angle of climb (aoc), horizontal speed = V * cos(aoc)
            horizontal_speed = V * np.cos(aoc)
            
            # Increment horizontal distance
            dx = horizontal_speed * dt
            distance += dx
            
            # Increment altitude (same as time to climb function)
            dh = roc * dt  # roc in m/s
            h += dh
            
        return distance

    def calculate_true_RoC(self, V: float, h: float) -> float:
        """
        Calculate the true Rate of Climb at a given altitude.
        """
        RoCs = self.calculate_RoC(V, h)
        V_ias = self.equivalent_air_speed(V, h)
        dH = 0.1
        dV_dH = V_ias * np.sqrt(self._get_density(0)) * (np.sqrt(1/self._get_density(h + dH)) - np.sqrt(1/self._get_density(h)))/ dH
        RoC = RoCs / (1 + V/self.g * dV_dH)
        return RoC
    
    def energy_height(self, V: float, h: float) -> float:
        """
        Calculate the energy height at a given altitude.
        """
        return h + V**2 / (2 * self.g)  # h in meters, V in m/s, energy height in meters


if __name__ == "__main__":
    # Example usage
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)
    
    h = np.array([0, 3048])  # Example altitude in meters
    # altitude_velocity.plot_power_curve(h)
    # altitude_velocity.plot_RoC_line(h)
    
    cruise_speed = aircraft_data.data['requirements']['cruise_speed']
    power_required_cruise = altitude_velocity.calculate_power_required(cruise_speed, h=0)
    power_available_cruise = altitude_velocity.calculate_power_available(h=0)
    print(f"Power required to cruise at h=0 and V={cruise_speed} m/s: {power_required_cruise/10**6:.2f} MW")
    print(f"Power required / Power available at cruise: {power_required_cruise / power_available_cruise:.2f}")

    print(f"Max RoC at h = 0 is {altitude_velocity.calculate_max_RoC(0)[0] * 196.85} ft/min")
    print(f"Max AoC at h = 0 is {altitude_velocity.calculate_max_AoC(0)[0] * 180/np.pi} degrees")
    print(f"Max true RoC at h = 0 is {altitude_velocity.calculate_true_RoC(altitude_velocity.calculate_max_RoC(0)[1], 0) * 196.85} ft/min")

    zero_points, stall_points = altitude_velocity.calculate_limit_points(airspeed_type='true', altitude_units='feet')
    altitude_velocity.plot_limit_points(zero_points, stall_points, 
                                        airspeed_type='true', 
                                        altitude_units='feet', 
                                        plot=('thrust_limit', 'stall_limit', 'Vy', 'Vx'))