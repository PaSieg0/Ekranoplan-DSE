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
        self.width = 6.43
        self.length = 39.0


        self.t = 0
        self.dt = 0.1
        self.t_end = 10

    def submerged_volume(self, h):
        #Piecewise volume as function of immersion depth h (m)
        #Considering hull shape
        if h <= self.triangle_height:
            return 0.5 * self.width * h * self.length
        else:
            triangle_vol = 0.5 * self.width * self.triangle_height * self.length
            rect_vol = self.width * (h - self.triangle_height) * self.length
            return triangle_vol + rect_vol
        
    def get_L_earo(self):
        self.gamma = np.arcsin(self.V[1] / self.V[0])  # flight path angle
        # TODO: Implement alpha with pitch angle for now assume gamma=alpa
        self.alpha = self.gamma # angle of attack

        self.CL = self.alpha_0 + self.CL_alpha * (self.alpha - self.alpha_0)





        
    def run_simulation(self):

        self.t_history = []
        self.Vx_history = []
        self.Vy_history = []
        self.X_history = []
        self.Y_history = []

        while self.t <= self.t_end:

            self.L_aero = self.get_L_earo()
            self.L_hydro = self.get_L_hydro()

            self.D_aero = self.get_D_aero()
            self.D_hydro = self.get_D_hydro()

            self.F_buoyancy = self.get_F_buoyancy()
            self.F_gravity = self.get_F_gravity()

            self.F_thrust = self.get_F_thrust()

            self.F_net = self.L_aero + self.L_hydro + self.D_aero + self.D_hydro + self.F_buoyancy + self.F_gravity + self.F_thrust

            self.a = self.F_net / self.MTOM

            self.V += self.a*self.dt

            self.pos += self.V*self.dt

            self.t += self.dt

        return 



if __name__ == "__main__":
    data = Data("design3.json")
    model = Take_off_mechanics(data)
    time, Vx, Vy, X, Y = model.run_simulation()
        