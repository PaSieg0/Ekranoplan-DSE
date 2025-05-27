import sys
import os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

from utils import Data
import pandas as pd
import numpy as np
from enum import Enum, auto
import matplotlib.pyplot as plt

class EmpType(Enum):
    CRUCIFORM = auto()
    T_TAIL = auto()
    CONVENTIONAL = auto()
    H_TAIL = auto()
    NONE = auto()



class Take_off_mechanics:


    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.tail_type = EmpType[aircraft_data.data['inputs']['tail_type']]
        if self.tail_type == EmpType.NONE:
            return

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']
        self.hull_surface = self.aircraft_data.data['outputs']['general']['hull_surface']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']

        self.CLmax_takeoff = self.aircraft_data.data['inputs']['CLmax_takeoff']
        self.CL_hydro = self.aircraft_data.data['inputs']['CL_hydro']
        self.Cd = self.aircraft_data.data['outputs']['general']['Cd']
        self.Cd_water = self.aircraft_data.data['outputs']['general']['Cd_water']

        self.MTOW = self.aircraft_data.data['outputs']['design']['MTOW']

        self.thrust = 110000*self.aircraft_data.data['inputs']['n_engines']

        #hardcoded variables for now, change these later on to be self-iterating
        self.rho_air = 1.225  # kg/m³ at sea level, 15°C
        self.P0 = 101325  # Pa
        self.Temp_air = 288.15  # K (15°C)
        self.rho_water = 1025  # kg/m³ at sea level
        self.g = 9.80665
        self.AOA = 5.0
        self.ThrustVectorAngle = 3.0  # im just capping everything here
        self.HydroLiftAngle = 0.0
        self.HydroDragAngle = 0.0

        #hull shaped fuselage sizing
        self.triangle_height = 3.0
        self.width = 6.0
        self.length = 60.0

    #solves for submerged volume given a submerged height
    def submerged_volume(self, h):
        #Piecewise volume as function of immersion depth h (m)
        #Considering hull shape
        if h <= self.triangle_height:
            return 0.5 * self.width * h * self.length
        else:
            triangle_vol = 0.5 * self.width * self.triangle_height * self.length
            rect_vol = self.width * (h - self.triangle_height) * self.length
            return triangle_vol + rect_vol

    #solves for initial submerged volume and depth at MTOW when at rest
    def get_initial_submerged_condition(self):
      
        # Solve V such that buoyant force = weight
        V0 = self.MTOW / (self.rho_water * self.g)

        # Initial guesses
        h_low = 0
        h_high = 5
        for _ in range(100):
            h_mid = 0.5 * (h_low + h_high)
            vol = self.submerged_volume(h_mid)
            if vol > V0:
                h_high = h_mid
            else:
                h_low = h_mid
        return self.submerged_volume(h_mid), h_mid

    #iteratively compute the bouyancy force
    def update_bouyancy_force(self, h):
        V = self.submerged_volume(h)
        Fx = 0
        Fy = self.rho_water * self.g * V
        return np.array([Fx, Fy])

    @staticmethod
    def get_fluid_force(coeff, rho, v, S, theta):
        v_mag = np.linalg.norm(v)

        if np.isfinite(v_mag):
            print(f"Warning: Invalid v_mag from velocity {v}")
        if np.isfinite(theta):
            print(f"Warning: Invalid theta: {theta}")

        F_mag = 0.5 * coeff * rho * v_mag**2 * S

        Fx = F_mag * np.cos(theta)
        Fy = F_mag * np.sin(theta)

        return np.array([Fx, Fy])


    def simulate_takeoff(self):
        dt = 0.1
        t_max = 600

        time = [0]
        velocity = [np.array([0.0, 0.0])]
        distance = [np.array([0.0, 0.0])]

        h = self.get_initial_submerged_condition()[1]
        v = np.array([0.0, 0.0])
        x = np.array([0.0, 0.0])
        t = 0
        g = self.g
        m = self.MTOW/g
        T = np.array([self.thrust,0])
        L = 0
        D = 0
        L_hydro = 0
        D_hydro = 0
        F_bouy = np.array([0,self.MTOW])

        while t < t_max:

            L = self.get_fluid_force(self.CLmax_takeoff, self.rho_air, v, self.S, np.deg2rad(self.AOA))
            D = self.get_fluid_force(self.Cd, self.rho_air, v, self.S, np.deg2rad(self.AOA))
            L_hydro = self.get_fluid_force(self.CL_hydro, self.rho_water, v, self.hull_surface, np.deg2rad(self.HydroLiftAngle))
            D_hydro = self.get_fluid_force(self.Cd_water, self.rho_water, v, self.hull_surface, np.deg2rad(self.HydroDragAngle))
            F_bouy = self.update_bouyancy_force(h)
            
            #V = self.update_submerged_volume
            F = L-D+L_hydro-D_hydro+F_bouy+T
            if not np.all(np.isfinite(F)):
                print(f"Invalid total force F: {F}")
            a = F/m

            v += a * dt
            x += v * dt
            h += v[0] * dt
            t += dt
            #V = self.update_submerged_volume(h-x[1])

            time.append(t)
            velocity.append(v)
            distance.append(x)



        return time, velocity, distance

    def run_simulation(self, plot=True):
        time, velocity, distance = self.simulate_takeoff()

        if plot:

        # Ensure velocity is a NumPy array for easy slicing
            velocity = np.array(velocity)
            distance = np.array(distance)

            plt.figure(figsize=(12, 6))

        # Plot Vx and Vy separately
            plt.subplot(2, 1, 1)
            plt.plot(time, velocity[:, 0], label="Vx (m/s)")
            plt.plot(time, velocity[:, 1], label="Vy (m/s)")
            plt.ylabel("Velocity Components (m/s)")
            plt.grid(True)
            plt.legend()

            plt.subplot(2, 1, 2)
            plt.plot(time, distance[:, 0], label="Distance (m)", color="orange")
            plt.plot(time, distance[:, 1], label="Distance (m)", color="orange")
            plt.xlabel("Time (s)")
            plt.ylabel("Distance (m)")
            plt.grid(True)
            plt.legend()

            plt.tight_layout()
            plt.show()

        return time, velocity, distance



if __name__ == "__main__":
    data = Data("design3.json")
    model = Take_off_mechanics(data)
    time, velocity, distance = model.run_simulation()
