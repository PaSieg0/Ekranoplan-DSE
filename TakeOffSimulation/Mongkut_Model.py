import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
import numpy as np
import matplotlib.pyplot as plt


class Simulation:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_file = f"design{self.aircraft_data.data['design_id']}.json"

        self.t_end = 100
        self.dt = 0.1
        self.t = 0.0


        self.C_delta_0 = self.calculate_C_delta_0()
        self.h = -3.1

    def calculate_C_delta_0(self):
        MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        rho_water = self.aircraft_data.data['rho_water']
        g = 9.81
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        return (MTOW) / (rho_water * g * B**3)
    
    def calculate_wetted_width(self):
        h = self.h
        if h <= -1.6:
            return self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        elif -1.6 < h <+ 0:
            return -self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'] * h / 1.6
        elif h >= 0:
            print("Returning 0")
            return 0
        
    
    def update_C_delta(self):
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'] #self.calculate_wetted_width()
        # print(f"Wetted width (B): {B}")
        if B < 0:
            raise ValueError("Wetted width cannot be negative. Check the height (h) value.")
        if B == 0:
            print("Returning infinity for C_delta due to zero wetted width.")
            return float('inf')
        self.C_delta = 1/(B**3)

    def update_C_v(self):
        v = self.v
        g = 9.81
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        self.C_v = (v) / (np.sqrt(g * B))

    def update_C_R(self):
        C_V = self.C_v
        self.C_R = 0.05789 * np.exp(-((C_V - 1.907) / (0.7561))**2) + 0.0273 * np.exp(-((C_V - 1.347) / (0.2536))**2) - 0.3322 * np.exp(-((C_V - 23.41) / (11.74))**2) + 0.07924 * np.exp(-((C_V - 3.227) / (1.951))**2)

    def update_C_R_delta(self):
        print(f"C_R: {self.C_R}, C_delta: {self.C_delta}, C_delta_0: {self.C_delta_0}")
        self.C_R_delta = self.C_R * (self.C_delta / self.C_delta_0)

    def update_R(self):
        rho_water = self.aircraft_data.data['rho_water']
        g = 9.81
        self.R = self.C_R_delta * rho_water * g * self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']**3

        

    def update_all_states(self):
        self.update_C_delta()
        self.update_C_v()
        self.update_C_R()
        self.update_C_R_delta()
        self.update_R()
        # print(f"Height: {self.h}, Velocity: {self.v}, C_R_delta: {self.C_R_delta}, R: {self.R}")
        # print(f"C_delta: {self.C_delta}, C_v: {self.C_v}, C_R: {self.C_R}")

        
    def run_simulation(self):
        for v in np.arange(0, 70, 0.1):
            self.h += 1
            self.v = v
            self.update_all_states()

            plt.plot(v, self.R, 'ro')

        plt.show()

if __name__ == "__main__":
    aircraft_data = Data('design3.json')
    simulation = Simulation(aircraft_data)
    simulation.run_simulation()