import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType, plt
from Class_I.AltitudeVelocity import AltitudeVelocity
from tqdm import tqdm

class OptimumClimb(AltitudeVelocity):
    def __init__(self, aircraft_data: Data, mission_type: MissionType):
        super().__init__(aircraft_data, mission_type)
        self._LD = self.data.data['outputs']['design']['LD']
        self._LD_g = self.data.data['outputs']['general']['LD_g']
        self.prop_consumption = self.data.data['inputs']['prop_consumption']
        self.prop_efficiency = self.data.data['inputs']['prop_efficiency']

    def fuel_burnt(self, range: float, WIG: bool) -> float:
        """
        Calculate the fuel burnt during the climb phase.
        """
        LD = self._LD if WIG else self._LD_g
        range_fraction = np.exp(-range*self.prop_consumption*self.g/self.prop_efficiency * LD**-1)
        return range_fraction * self._current_weight
    
    def energy_to_climb(self, h_start: float, h_end: float, V_ias: float=None) -> float:
        """
        Calculate the energy (in joules) required to climb from h_start to h_end.
        Optionally at a constant indicated airspeed (V_ias). Returns energy and time taken.
        """
        # Euler integration using max RoC from h_start to h_end
        dt = 0.1  # time step in seconds
        h = h_start
        t = 0.0
        energy = 0.0
        V_prev = self.true_air_speed(V_ias, h) if V_ias is not None else self.calculate_max_RoC(h)[1]
        while h < h_end:
            if V_ias is None:
                roc, V_tas = self.calculate_max_RoC(h)
            else:
                V_tas = self.true_air_speed(V_ias, h)
                roc = self.calculate_RoC(V_tas, h)
            if roc <= 0:
                return np.inf
            
            dh = roc * dt  # roc in m/s
            de = self.calculate_power_required(V_tas, h, RoC=roc)*dt
            dKE = 0.5 * self._current_weight/self.g * (V_tas**2 - V_prev**2)  # Kinetic energy change
            energy += de + dKE  # Total energy change
            h += dh
            t += dt

            V_prev = V_tas
            if dh <= 1e-4:
                print(" \nWarning: Climb rate is zero or negative, stopping integration.")
                break
        return energy, t
    
    def plot_max_roc_path(self, h_start: float, h_end: float, V_ias: float=None):
        """
        Plot the path followed when using the maximum rate of climb strategy.
        Shows altitude vs horizontal distance.
        """
        # Initialize arrays to store the path data
        altitudes = []
        horizontal_distances = []
        
        # Euler integration using max RoC from h_start to h_end
        dt = 0.1  # time step in seconds
        h = h_start
        horizontal_distance = 0.0
        
        while h < h_end:
            if V_ias is None:
                roc, V_tas = self.calculate_max_RoC(h)
            else:
                V_tas = self.true_air_speed(V_ias, h)
                roc = self.calculate_RoC(V_tas, h)
            
            if roc <= 0:
                print(f"Warning: Climb rate is zero or negative at altitude {h:.1f}m, stopping.")
                break
            
            # Store current values
            altitudes.append(h)
            horizontal_distances.append(horizontal_distance)
            
            # Update for next iteration
            dh = roc * dt
            dx = V_tas * dt  # horizontal distance traveled
            h += dh
            horizontal_distance += dx
            
            if dh <= 1e-4:
                print("Warning: Climb rate is very small, stopping integration.")
                break
        
        # Convert to numpy arrays for easier manipulation
        altitudes_roc = np.array(altitudes)
        horizontal_distances_roc = np.array(horizontal_distances)
        
        # Now calculate max AoC strategy
        altitudes_aoc = []
        horizontal_distances_aoc = []
        
        h = h_start
        horizontal_distance = 0.0
        
        while h < h_end:
            if V_ias is None:
                aoc, V_tas = self.calculate_max_AoC(h)
                roc = self.calculate_RoC(V_tas, h)
            else:
                V_tas = self.true_air_speed(V_ias, h)
                roc = self.calculate_RoC(V_tas, h)
            
            if roc <= 0:
                print(f"Warning: Climb rate is zero or negative at altitude {h:.1f}m, stopping AoC.")
                break
            
            # Store current values
            altitudes_aoc.append(h)
            horizontal_distances_aoc.append(horizontal_distance)
            
            # Update for next iteration
            dh = roc * dt
            dx = V_tas * dt  # horizontal distance traveled
            h += dh
            horizontal_distance += dx
            
            if dh <= 1e-4:
                print("Warning: Climb rate is very small, stopping AoC integration.")
                break
        
        # Convert to numpy arrays for easier manipulation
        altitudes_aoc = np.array(altitudes_aoc)
        horizontal_distances_aoc = np.array(horizontal_distances_aoc)
        
        # Plot both strategies
        plt.plot(horizontal_distances_roc, altitudes_roc, color='tab:blue', linestyle=':', alpha=0.7, label='Max Rate of Climb')
        plt.plot(horizontal_distances_aoc, altitudes_aoc, color='tab:orange', linestyle=':', alpha=0.7, label='Max Angle of Climb')
        
        # Plot 100m margin below flight paths
        margin = 100  # meters
        # Only plot where altitude with margin is above 0
        roc_valid = (altitudes_roc - margin) > 0
        aoc_valid = (altitudes_aoc - margin) > 0
        
        plt.plot(horizontal_distances_roc[roc_valid], (altitudes_roc - margin)[roc_valid], color='tab:blue', linestyle='-', linewidth=2, label='Max RoC - 100m margin')
        plt.plot(horizontal_distances_aoc[aoc_valid], (altitudes_aoc - margin)[aoc_valid], color='tab:orange', linestyle='-', linewidth=2, label='Max AoC - 100m margin')
        
        plt.xlabel('Horizontal Distance (m)')
        plt.ylabel('Altitude (m)')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        print(f"Max RoC - Total horizontal distance: {horizontal_distances_roc[-1]:.1f} meters")
        print(f"Max RoC - Final altitude reached: {altitudes_roc[-1]:.1f} meters")
        print(f"Max AoC - Total horizontal distance: {horizontal_distances_aoc[-1]:.1f} meters")
        print(f"Max AoC - Final altitude reached: {altitudes_aoc[-1]:.1f} meters")
        
        return (horizontal_distances_roc, altitudes_roc), (horizontal_distances_aoc, altitudes_aoc)


if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    mission_type = MissionType.ALTITUDE  # Example mission type
    optimum_climb = OptimumClimb(aircraft_data, mission_type)

    h_start = 10  # Starting altitude in meters
    h_end = 3048  # Ending altitude in meters
    # V_ias = 86.05860123201985  # Indicated airspeed in m/s
    
    opt_time = optimum_climb.calculate_t_climb(h_start=h_start, h_end=h_end, V_ias=None)
    optimum_climb.plot_max_roc_path(h_start=h_start, h_end=h_end, V_ias=None)

    # times = []  # Initialize an empty array to store times
    # energies = []  # Initialize an empty array to store energies
    # V_ias_list = np.arange(70, 115, 1)  # Example range of V_ias values from 50 m/s to 200 m/s
    # for V_ias in tqdm(V_ias_list, desc="Calculating times for different V_ias"):
    #     energy, time = optimum_climb.energy_to_climb(h_start=h_start, h_end=h_end, V_ias=V_ias)
    #     times.append(time)
    #     energies.append(energy)

    # min_energy_idx = np.argmin(energies)
    # min_energy = energies[min_energy_idx]
    # min_energy_ias = V_ias_list[min_energy_idx]
    # print(f"Minimum energy to climb: {min_energy/10**9:.2f} GJ at IAS = {min_energy_ias:.2f} m/s")
    # print(f"Time to climb minimum energy: {times[min_energy_idx]:.2f} seconds")

    # times = np.array(times)
    # energies = np.array(energies)

    # # Plotting the results
    # fig, ax1 = plt.subplots()

    # color = 'tab:blue'
    # ax1.set_xlabel('Indicated Airspeed (m/s)')
    # ax1.set_ylabel('Time to Climb (seconds)', color=color)
    # ax1.plot(V_ias_list, times, marker='o', color=color, label='Time to Climb')
    # ax1.axhline(opt_time, color='r', linestyle='--', label=f'Optimum Time ({opt_time:.2f} s)')
    # ax1.tick_params(axis='y', labelcolor=color)
    # ax1.legend(loc='upper left')

    # ax2 = ax1.twinx()
    # color = 'tab:green'
    # ax2.set_ylabel('Energy to Climb (Joules)', color=color)
    # ax2.plot(V_ias_list, energies, marker='s', color=color, label='Energy to Climb')
    # ax2.tick_params(axis='y', labelcolor=color)
    # ax2.legend(loc='upper right')

    # plt.title('Time and Energy to Climb vs Indicated Airspeed')
    # plt.grid()
    # plt.show()
    