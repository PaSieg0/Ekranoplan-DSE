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

class EnvironmentData:
    def __init__(self):
        self.rho_air = 1.225       # kg/m³ at sea level, 15°C
        self.P0 = 101325     # Pa
        self.Temp_air = 288.15  # K (15°C)
        self.rho_water = 1025   # kg/m³ at sea level
        self.g = 9.80665

class Angles:
    def deg2rad(self, deg):
        return deg * np.pi / 180

    def __init__(self):
        self.AOA = self.deg2rad(5.0)
        self.ThrustVectorAngle = self.deg2rad(3.0) #im just capping everything here
        self.HydroLiftAngle = self.deg2rad(0.0)
        self.HydroDragAngle = self.deg2rad(0.0)

class Take_off_dynamic_model:
    def __init__(self, aircraft_data: Data, env: EnvironmentData, angles: Angles):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.tail_type = EmpType[aircraft_data.data['inputs']['tail_type']]

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']
        self.hull_surface = self.aircraft_data.data['outputs']['general']['hull_surface']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']

        self.CLmax_takeoff = self.aircraft_data.data['outputs']['inputs']['CLmax_takeoff']
        self.CL_hydro = self.aircraft_data.data['outputs']['inputs']['CL_hydro']
        self.Cd = self.aircraft_data.data['outputs']['general']['Cd']
        self.Cd_water = self.aircraft_data.data['outputs']['general']['Cd_water']

        self.MTOW = self.aircraft_data.data['outputs']['design']['MTOW']

        self.thrust = 110000*self.aircraft_data.data['outputs']['inputs']['n_engines']

        self.env = env
        self.angles = angles


        def get_fluid_force(coeff, rho, v, S, theta):
            F_mag = 0.5 * coeff * rho * v ** 2 * S
            Fx = F_mag * np.cos(theta)
            Fy = F_mag * np.sin(theta)
            return np.array([Fx, Fy])

        def get_bouyancy_force():
            Fx = 0
            Fy =
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
            g = self.env.g
            m = self.MTOW/g
            L = get_fluid_force(self.CLmax_takeoff, self.env.rho, v, self.S,angles.AOA, 1)
            D = get_fluid_force(self.Cd, self.env.rho, v, self.S, angles.AOA, 0)
            L_hydro = get_fluid_force(self.Cl_hydro, self.env.rho_water, v, self.hull_surface, angles.HydroLiftAngle, 1)
            D_hydro = get_fluid_force(self.Cd_water, self.env.rho_water, v, self.hull_surface, angles.HydroDragAngle,0)
            F_bouy = get_bouyancy_force()

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
