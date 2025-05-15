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
    
    def plot_power_curve(self, h_list: np.array):
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
    
    def plot_RoC_line(self, h_list: np.array):
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

    def calculate_h_max(self, height_resolution = 10):
        # Set h range from 0 to max

        initial_h_range = np.arange(0, 20000, height_resolution)
        RoC_V_list=  [self.calculate_max_RoC(h) for h in initial_h_range]

        # Find where RoC first reaches 0 (i.e., aircraft can no longer climb)
        RoC_values = [roc for roc, _ in RoC_V_list]
        zero_crossings = np.where(np.array(RoC_values) <= 0)[0]
        if len(zero_crossings) == 0:
            raise ValueError("RoC never reaches zero in the given altitude range.")
        h_max_idx = zero_crossings[0]
        h_max = initial_h_range[h_max_idx]

        return h_max


if __name__ == "__main__":
    # Example usage
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.DESIGN  # Example mission type
    altitude_velocity = AltitudeVelocity(aircraft_data, mission_type)
    
    h = np.array([0, 3048])  # Example altitude in meters
    altitude_velocity.plot_power_curve(h)
    altitude_velocity.plot_RoC_line(h)

    print(f"Max RoC at h = 0 is {altitude_velocity.calculate_max_RoC(0)[0] * 196.85} ft/min")
    print(f"Max RoC at h = 3048 is {altitude_velocity.calculate_max_RoC(3048)[0] * 196.85} ft/min")
    print(f"Service Ceiling: {altitude_velocity.calculate_h_max()} metres")