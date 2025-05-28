import sys
import os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

from utils import Data
import pandas as pd
import numpy as np
from enum import Enum, auto
import matplotlib.pyplot as plt

from aero.lift_curve import lift_curve

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
        self.MTOM = self.aircraft_data.data['outputs']['max']['MTOM']  # Maximum Take-Off Mass

        self.max_thrust = 110000*self.aircraft_data.data['inputs']['n_engines'] * self.aircraft_data.data['inputs']['prop_efficiency']  # Maximum thrust from all engines combined

        #hardcoded variables for now, change these later on to be self-iterating
        self.rho_air = 1.225  # kg/m³ at sea level, 15°C
        self.P0 = 101325  # Pa
        self.Temp_air = 288.15  # K (15°C)
        self.rho_water = 1025  # kg/m³ at sea level
        self.g = 9.80665
        self.ThrustVectorAngle = 3.0  # im just capping everything here
        self.HydroLiftAngle = 0.0
        self.HydroDragAngle = 0.0

        #hull shaped fuselage sizing
        # TODO: CHANGE THESE VALUES SO THEY TAKE VALUES FROM JSON
        self.triangle_height = 1.6
        self.width = 7.5
        # self.length = 39.0
        self.hull_length = self.aircraft_data.data['outputs']['general']['l_bottom']
        self.lambda_ = 4
        self.deadrise_angle = np.deg2rad(25)  # Deadrise angle in radians
        # TODO: CHECK IF AFT CG OR FORWARD CG
        self.ksi = self.aircraft_data.data['outputs']['general']['l_bottom'] - self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        # TODO CHANGE BEAM LENGTH OF PLANNING SURFACE
        self.B = self.aircraft_data.data['outputs']['general']['d_fuselage']  # Beam of the hull

        self.Ls = 1
        self.Lb = 2
        self.kb = 1.2

        # TODO: CHANGE THIS WHEN GOING TO RIGID BODY
        self.theta = 0


        self.lift_curve = lift_curve()


        # initial conditions
        self.a = np.array([0.0, 0.0])  # [Ax, Ay]
        self.V = np.array([0.0, 0.0])  # [Vx, Vy]
        self.pos = np.array([0.0, self.get_initial_height()])  # [X, Y]

        self.V_mag = np.linalg.norm(self.V)
        self.gamma = np.arctan2(self.V[1], self.V[0]) if self.V[0] != 0 else 0  # flight path angle
        # TODO: Implement alpha with pitch angle for now assume gamma=alpa
        self.alpha = -self.gamma

        print(self.pos)


        self.t = 0
        self.dt = 0.1
        self.t_end = 10

        
    def get_initial_height(self):
        V_disp = self.MTOM / self.rho_water
        A_disp = V_disp / self.hull_length
        # TODO: CHANGE THIS SO IT TAKES VALUES FROM JSON
        A_triangle = 1/2*self.triangle_height * self.width

        if A_disp <= A_triangle:
            return -np.sqrt(4*A_disp*self.triangle_height/self.width)
        else: 
            return -((A_disp - A_triangle) / self.width + self.triangle_height)
        
    def calculate_submerged_area(self):
        h = self.pos[1]
        if h >= 0:
            return 0
        elif h < 0 and h > -self.triangle_height:
            return 0.5 * self.width * -h
        elif h <= -self.triangle_height:
            return self.width * (-h - self.triangle_height) + 0.5 * self.width * self.triangle_height
        
    def get_L_aero(self):
        print(self.V)
        print(self.alpha)
        self.CL = self.lift_curve.interpolate_Cl(np.rad2deg(self.alpha))

        L_aero = 0.5 * self.rho_air * np.linalg.norm(self.V)**2 * self.S * self.CL
        L = np.array([L_aero * np.sin(np.deg2rad(self.alpha)), L_aero * np.cos(np.deg2rad(self.alpha))])
        return L
    
    def get_D_aero(self):
        self.Cd0 = self.aircraft_data.data['inputs']['Cd0']

        self.Cd = self.Cd0 + self.CL**2 / (np.pi * self.aircraft_data.data['outputs']['wing_design']['aspect_ratio'] * self.aircraft_data.data['inputs']['oswald_factor'])  # induced drag coefficient


        D_aero = 0.5 * self.rho_air * -np.linalg.norm(self.V)**2 * self.S * self.Cd
        D = np.array([D_aero * np.cos(np.deg2rad(self.alpha)), D_aero * np.sin(np.deg2rad(self.alpha))])
        return D
    
    def calculate_lambda_(self):
        # TODO: CHECK IF THIS IS CORRECT
        if self.pos[1] <= 0:
            return self.hull_length / self.aircraft_data.data['outputs']['general']['d_fuselage']
        else:
            return 0
    
    def calculate_lambda_1(self):

        lambda_ = self.calculate_lambda_()
        print(f"Lambda: {lambda_}, h: {self.pos[1]}")
        print(f"V: {self.V}, V_mag: {self.V_mag}")
        
        if 0 <= lambda_ <= 1:
            return 1.6*lambda_ - 0.3*lambda_**2
        elif 1 < lambda_ <= 4:
            return lambda_ + 0.3
        else:
            raise ValueError("Lambda must be between 0 and 4 for this calculation.")
        
    def calculate_Froude_number(self):
        V_mag = np.linalg.norm(self.V)
        if V_mag == 0:
            return 0.0001
        
        return V_mag / np.sqrt(self.g * self.hull_length)

    
    def calculate_lambda_beta(self):
        Froude_number = self.calculate_Froude_number()
        lambda_1 = self.calculate_lambda_1()
        lambda_beta = lambda_1**0.8/np.cos(self.deadrise_angle) * (1 - 0.29*np.sin(self.deadrise_angle)**0.28) * (1 + 1.35*np.sin(self.deadrise_angle)**0.44 * self.ksi/(self.B * Froude_number**0.5))**0.5
        print(f"Froude_number: {Froude_number}, lambda_1: {lambda_1}, lambda_beta: {lambda_beta}")
        return lambda_beta

    def calculate_theta_b(self):
        if self.Ls <= self.Lb:
            return self.theta
        if self.Ls > self.Lb:
            return self.theta + self.kb*(self.Ls - self.Lb)

    

    def calculate_theta_beta(self):
        Froude_number = self.calculate_Froude_number()
        lambda_beta = self.calculate_lambda_beta()
        theta_b = self.calculate_theta_b()
        theta_beta = theta_b + (0.15*np.sin(self.deadrise_angle)**0.8)/(Froude_number**0.3) * (1 - 0.17*np.sqrt(lambda_beta * np.cos(self.deadrise_angle)))/(np.sqrt(lambda_beta * np.cos(self.deadrise_angle)))
        return theta_beta
    
    def calculate_B(self):
        return self.aircraft_data.data['outputs']['general']['d_fuselage']  # Beam of the hull

    
    def get_L_hydro(self):
        lambda_beta = self.calculate_lambda_beta()
        theta_beta = self.calculate_theta_beta()
        B = self.calculate_B()
        wetted_area = lambda_beta * B**2
        print(f"lambda_beta: {lambda_beta}, theta_beta: {theta_beta}, B: {B}")
        L = self.rho_water * self.V_mag**2 * lambda_beta * B**2 * 0.35*np.pi*theta_beta / (1 + 1.4*lambda_beta) if self.pos[1] < 0 else 0
        L_vec = wetted_area*np.array([0, L])
        return L_vec
        

    def get_D_hydro(self):
        pass

    def get_F_buoyancy(self):
        A_disp = self.calculate_submerged_area()
        V_disp = A_disp * self.hull_length
        return np.array([0, self.rho_water * V_disp * self.g])

    def get_F_gravity(self):
        return np.array([0, -self.MTOM * self.g])

    def get_F_thrust(self):
        P = self.aircraft_data.data['inputs']['engine_power'] * self.aircraft_data.data['inputs']['n_engines']  # Total power from all engines
        T = 0.8*P*self.aircraft_data.data['inputs']['prop_efficiency'] / self.V_mag if self.V_mag != 0 else float('inf')  # Thrust based on power and velocity
        T = min(T, self.max_thrust)  # Limit thrust to maximum thrust
        T = float(T)
        T_vec = np.array([T * np.cos(self.gamma), T*np.sin(self.gamma)])
        return T_vec  # Thrust vector in the direction of flight

        
    def run_simulation(self):
        self.t_history = []
        self.Vx_history = []
        self.Vy_history = []
        self.X_history = []
        self.Y_history = []

        self.L_aero_history = []
        self.D_aero_history = []

        self.L_hydro_history = []
        self.D_hydro_history = []
        self.F_buoyancy_history = []
        self.T_history = []


        while self.t <= self.t_end:


            self.L_aero = self.get_L_aero()
            self.D_aero = self.get_D_aero()

            self.L_hydro = self.get_L_hydro()
            # self.D_hydro = self.get_D_hydro()

            self.F_buoyancy = self.get_F_buoyancy()
            self.F_gravity = self.get_F_gravity()

            self.F_thrust = self.get_F_thrust()

            print(f"L_aero: {self.L_aero}, D_aero: {self.D_aero}, L_hydro: {self.L_hydro}, F_buoyancy: {self.F_buoyancy}, F_gravity: {self.F_gravity}, F_thrust: {self.F_thrust}")

            self.F_net = (self.L_aero + 
                          self.D_aero + 
                          self.L_hydro + 
                        #   self.D_hydro + 
                          self.F_buoyancy + 
                          self.F_gravity + 
                          self.F_thrust
                          )
            
            print(f"F_net: {self.F_net}")

            self.a = self.F_net / self.MTOM

            # print(f"Acceleration: {self.a}, V: {self.V}, Position: {self.pos}, Time: {self.t}")

            print(f"A: {self.a}")
            self.V += self.a*self.dt
            self.V_mag = np.linalg.norm(self.V)
            self.gamma = np.arctan2(self.V[1], self.V[0]) if self.V[0] != 0 else 0  # flight path angle
            # TODO: Implement alpha with pitch angle for now assume gamma=alpa
            self.alpha = np.deg2rad(2.5)

            self.pos += self.V*self.dt

            self.t_history.append(self.t)
            self.Vx_history.append(self.V[0])
            self.Vy_history.append(self.V[1])
            self.X_history.append(self.pos[0])
            self.Y_history.append(self.pos[1])
            self.L_aero_history.append(self.L_aero[1])
            self.D_aero_history.append(self.D_aero[0])
            self.L_hydro_history.append(self.L_hydro[1])
            # self.D_hydro_history.append(self.D_hydro[0])  # Assuming D_hydro is not implemented yet
            self.F_buoyancy_history.append(self.F_buoyancy[1])
            self.T_history.append(self.F_thrust[0])


            self.t += self.dt

        return 

    def plot_results(self):
        if not self.t_history:
            print("No simulation data to plot.")
            return
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        axs[0, 0].plot(self.t_history, self.L_hydro_history, color='b')
        axs[0, 0].set_title('L_hydro (m/s)')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Vx (m/s)')
        axs[0, 0].grid()

        axs[0, 1].plot(self.t_history, self.F_buoyancy_history, color='g')
        axs[0, 1].set_title('Bouy (m/s)')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Vy (m/s)')
        axs[0, 1].grid()

        axs[1, 0].plot(self.t_history, self.L_aero_history, color='r')
        axs[1, 0].set_title('L (m)')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('X (m)')
        axs[1, 0].grid()

        axs[1, 1].plot(self.t_history, self.Y_history, color='m')
        axs[1, 1].set_title('Y (m)')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Y (m)')
        axs[1, 1].grid()

        plt.tight_layout()
        plt.show()



if __name__ == "__main__":
    data = Data("design3.json")
    model = Take_off_mechanics(data)
    model.run_simulation()
    model.plot_results()
