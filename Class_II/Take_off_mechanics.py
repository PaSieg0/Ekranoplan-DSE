import os
import pandas as pd
import numpy as np
from utils import Data
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

        self.CLmax_takeoff = self.aircraft_data.data['outputs']['inputs']['CLmax_takeoff']
        self.CL_hydro = self.aircraft_data.data['outputs']['inputs']['CL_hydro']
        self.Cd = self.aircraft_data.data['outputs']['general']['Cd']
        self.Cd_water = self.aircraft_data.data['outputs']['general']['Cd_water']

        self.MTOW = self.aircraft_data.data['outputs']['design']['MTOW']

        self.thrust = 110000*self.aircraft_data.data['outputs']['inputs']['n_engines']

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

        #hull shaped fuselage initial sizing
        self.triangle_height = 3
        self.width = 6
        self.length = 60


    def immersed_volume(self, h):
        """Piecewise volume as function of immersion depth h (m)"""
        if h <= self.triangle_height:
            return 0.5 * self.width * h * self.length
        else:
            triangle_vol = 0.5 * self.width * self.triangle_height * self.length
            rect_vol = self.width * (h - self.triangle_height) * self.length
            return triangle_vol + rect_vol

    def get_initial_submerged_volume(self):
        """Return submerged volume required to balance MTOW"""
        # Solve V such that buoyant force = weight
        target_volume = self.MTOW / (self.rho_water * self.g)

        # Use bisection method to find h
        h_low = 0
        h_high = 5
        for _ in range(100):
            h_mid = 0.5 * (h_low + h_high)
            vol = self.immersed_volume(h_mid)
            if vol > target_volume:
                h_high = h_mid
            else:
                h_low = h_mid
        return self.immersed_volume(h_mid), h_mid

    def update_bouyancy_force(self, h):
        """Returns buoyancy force for a given immersion depth h (m)"""
        V = self.immersed_volume(h)
        Fx = 0
        Fy = self.rho_water * self.g * V
        return np.array([Fx, Fy])

    def get_new_submerged_volume(self, h):
        """Returns volume submerged at depth h"""
        return self.immersed_volume(h)

    def get_fluid_force(coeff, rho, v, S, theta):
        F_mag = 0.5 * coeff * rho * v ** 2 * S
        Fx = F_mag * np.cos(theta)
        Fy = F_mag * np.sin(theta)
        return np.array([Fx, Fy])


    def simulate_takeoff(self):
        dt = 0.1
        t_max = 600

        time = [0]
        velocity = [0]
        distance = [0]

        v = 0
        x = 0
        t = 0
        g = self.g
        m = self.MTOW/g
        L = self.get_fluid_force(self.CLmax_takeoff, self.rho_air, v, self.S, np.deg2rad(self.AOA))
        D = self.get_fluid_force(self.Cd, self.rho_air, v, self.S, np.deg2rad(self.AOA))
        L_hydro = self.get_fluid_force(self.CL_hydro, self.rho_water, v, self.hull_surface, np.deg2rad(self.HydroLiftAngle))
        D_hydro = self.get_fluid_force(self.Cd_water, self.rho_water, v, self.hull_surface, np.deg2rad(self.HydroDragAngle))
        F_bouy = self.update_bouyancy_force(x)

        while t < t_max:
            F = L-D+L_hydro-D_hydro+F_bouy

            a = F/m

            v += a * dt
            x += v * dt
            t += dt

            time.append(t)
            velocity.append(v)
            distance.append(x)


        return time, velocity, distance

    def run_simulation(self, plot=True):
        time, velocity, distance = self.simulate_takeoff()

        if plot:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(12, 6))

            plt.subplot(2, 1, 1)
            plt.plot(time, velocity, label="Velocity (m/s)")
            plt.ylabel("Velocity (m/s)")
            plt.grid(True)
            plt.legend()

            plt.subplot(2, 1, 2)
            plt.plot(time, distance, label="Distance (m)", color="orange")
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
