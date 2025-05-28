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



class Take_off_mechanics:

    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_number = self.aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.hull_surface = self.aircraft_data.data['outputs']['general']['hull_surface']

        self.MTOM = self.aircraft_data.data['outputs']['max']['MTOM']  # Maximum Take-Off Mass
        self.I_y = 2.7e9 #self.aircraft_data.data['outputs']['general']['I_y']  # Moment of inertia around y-axis

        self.max_thrust = 110000*self.aircraft_data.data['inputs']['n_engines'] * self.aircraft_data.data['inputs']['prop_efficiency']  # Maximum thrust from all engines combined

        self.rho_air = 1.225  # kg/m³ at sea level, 15°C
        self.rho_water = 1025  # kg/m³ at sea level
        self.g = 9.80665
        self.kinmetic_viscosity = self.aircraft_data.data['kinematic_viscosity']  # Kinematic viscosity of water

        # TODO: CHANGE THESE VALUES SO THEY TAKE VALUES FROM JSON
        self.triangle_height = 1.6
        self.width = 7.5
        self.hull_length = 20 #self.aircraft_data.data['outputs']['general']['l_bottom']
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
        self.q = 0
        self.theta = 0


        self.lift_curve = lift_curve()

        # initial conditions
        self.a = np.array([0.0, 0.0])  # [Ax, Ay]
        self.V = np.array([0.0, 0.0])  # [Vx, Vy]
        self.pos = np.array([0.0, self.get_initial_height()])  # [X, Y]
        self.V_mag = np.linalg.norm(self.V)

        self.gamma = np.arctan2(self.V[1], self.V[0]) if self.V[0] != 0 else 0  # flight path angle
        # TODO: Implement alpha with pitch angle for now assume gamma=alpa
        self.alpha = 0

        print(self.pos)

        self.t = 0

        self.dt = 0.1
        self.t_end = 1

        
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
        self.CL = np.clip(self.lift_curve.interpolate_Cl(np.rad2deg(self.alpha)), -0.1, 1.851)


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
        # print(f"Lambda: {lambda_}, h: {self.pos[1]}")
        # print(f"V: {self.V}, V_mag: {self.V_mag}")
        
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
        # print(f"Froude_number: {Froude_number}, lambda_1: {lambda_1}, lambda_beta: {lambda_beta}")
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
        wetted_area = self.calculate_wetted_area()
        # print(f"lambda_beta: {lambda_beta}, theta_beta: {theta_beta}, B: {B}")
        L = self.rho_water * self.V_mag**2 * lambda_beta * B**2 * 0.35*np.pi*theta_beta / (1 + 1.4*lambda_beta) if self.pos[1] < 0 else 0
        L_vec = wetted_area*np.array([0, L])
        return L_vec

    def calculate_wetted_area(self):
        h = self.pos[1]
        if h <= 0:
            d = np.sqrt((self.width/2/self.triangle_height * h)**2+(h)**2)
        else: 
            d = 0
        wetted_area = 2*self.hull_length*d
        return wetted_area
    
    def calculate_Reynolds_number(self):
        V_mag = np.linalg.norm(self.V)
        if V_mag == 0:
            return 0.0001
        return (self.rho_water * V_mag * self.hull_length) / self.kinmetic_viscosity
        

    def cf(self):
        Re = self.calculate_Reynolds_number()
        Cf = 0.075/((np.log10(Re)-2)**2)

        return Cf
    
    def determine_k(self):
        return 1.5
    
    def Cvp(self):
        Cf = self.cf()
        k = self.determine_k()
        return k*Cf

    def Cwa(self):
        Cf = self.cf()
        Cvp = self.Cvp()
        Cwa = 0.5 * (Cf + Cvp)
        return Cwa

    def get_D_hydro(self):
        Cf = self.cf()
        Cvp = self.Cvp()
        Cwa = self.Cwa()

        Cd_water = Cf + Cvp + Cwa
        wetted_area = self.calculate_wetted_area()
        D = 0.5* self.rho_water * self.V_mag**2 * wetted_area * Cd_water
        return np.array([-D, 0]) 
    
    def get_F_buoyancy(self):
        A_disp = self.calculate_submerged_area()
        V_disp = A_disp * self.hull_length
        return np.array([0, self.rho_water * V_disp * self.g])

    def get_F_gravity(self):
        return np.array([0, -self.MTOM * self.g])

    def get_F_thrust(self):
        P = self.aircraft_data.data['inputs']['engine_power'] * self.aircraft_data.data['inputs']['n_engines']  # Total power from all engines
        T = 0.8*P*self.aircraft_data.data['inputs']['prop_efficiency'] / self.V_mag if self.V_mag != 0 else self.max_thrust  # Thrust based on power and velocity
        T = min(T, self.max_thrust)  # Limit thrust to maximum thrust
        T_vec = np.array([T * np.cos(self.gamma), T*np.sin(self.gamma)])
        return T_vec  # Thrust vector in the direction of flight

    def get_Ma(self):
        Cm = self.lift_curve.interpolate_Cm(np.rad2deg(self.alpha))
        Ma = 0.5 * self.rho_air * np.linalg.norm(self.V)**2 * self.S * Cm * self.MAC
        return Ma

    def get_bouyancy_center(self):
        # Assuming the buoyancy center is at the centroid of the submerged area
        h = self.pos[1]
        if h >= 0:
            return 0
        else:
            return self.hull_length / 2

    def get_hydrodynamic_center(self):
        # Assuming the hydrodynamic center is at the centroid of the wetted area
        h = self.pos[1]
        lambda_1 = self.calculate_lambda_1()
        Froude_number = self.calculate_Froude_number()
        if h >= 0:
            return 0, 0
        else:
            x_step = 3/4 + 0.08*lambda_1**0.865/np.sqrt(Froude_number)*self.hull_length
            x_hp = self.hull_length + x_step
            z_hp = h/2
        return x_hp, z_hp  # Return the x and z coordinates of the hydrodynamic center
    
    def get_cg(self):
        x_cg = self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        z_cg = 3 #self.aircraft_data.data['outputs']['cg_range']['most_aft_cg_height']
        return x_cg, z_cg  # Return the x and z coordinates of the center of gravity
    
    def get_Mw(self):
        F_bu = self.get_F_buoyancy()[1]
        L_w = self.get_L_hydro()[1]
        R_w = self.get_D_hydro()[0]
        x_b = self.get_bouyancy_center()
        x_hp, z_hp = self.get_hydrodynamic_center()
        x_cg, z_cg = self.get_cg()
        x_p = x_hp - x_cg  # Distance from the center of gravity to the hydrodynamic center
        z_p = z_hp - z_cg  # Distance from the center of gravity to the hydrodynamic center in the vertical direction
        
        Mw = F_bu * x_b + L_w * x_p + R_w + z_p
        return Mw
    
    def get_MT(self):
        return 0
    
    def get_Mqw(self):
        return 0

        
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

        self.CL_history = []
        self.Cd_history = []
        self.alpha_history = []
        self.gamma_history = []
        self.V_mag_history = []

        self.q_dot_history = []
        self.q_history = []
        self.pitch_angle_history = []


        while self.t <= self.t_end:


            self.L_aero = self.get_L_aero()
            self.D_aero = self.get_D_aero()

            self.L_hydro = self.get_L_hydro()
            self.D_hydro = self.get_D_hydro()

            self.F_buoyancy = self.get_F_buoyancy()
            self.F_gravity = self.get_F_gravity()

            self.F_thrust = self.get_F_thrust()

            self.Moment_aero = self.get_Ma()
            self.Moment_hydro = self.get_Mw()
            self.Moment_T = self.get_MT()
            self.Moment_qw = self.get_Mqw()



            # print(f"L_aero: {self.L_aero}, D_aero: {self.D_aero}, L_hydro: {self.L_hydro}, D_hydro: {self.D_hydro}, F_buoyancy: {self.F_buoyancy}, F_gravity: {self.F_gravity}, F_thrust: {self.F_thrust}")

            self.F_net = (
                self.L_aero + 
                self.D_aero + 
                self.L_hydro + 
                self.D_hydro + 
                self.F_buoyancy + 
                self.F_gravity + 
                self.F_thrust
            )
            
            self.M_net = (
                self.Moment_aero +
                self.Moment_hydro +
                self.Moment_T +
                self.Moment_qw
            )

            

            self.a = self.F_net / self.MTOM
            self.q_dot = self.M_net / self.I_y

            self.q += self.q_dot * self.dt  # Update q using q_dot
            self.theta += self.q * self.dt  # Update theta using q
            print(f"M_net: {self.M_net}, Moment_aero: {self.Moment_aero}, Moment_hydro: {self.Moment_hydro}, Moment_T: {self.Moment_T}, Moment_qw: {self.Moment_qw}")
            print(f"theta: {self.theta}, q: {self.q}, q_dot: {self.q_dot}")


            # print(f"Acceleration: {self.a}, V: {self.V}, Position: {self.pos}, Time: {self.t}")

            # print(f"A: {self.a}")
            self.V += self.a*self.dt
            self.V_mag = np.linalg.norm(self.V)
            self.gamma = np.arctan2(self.V[1], self.V[0]) if self.V[0] != 0 else 0  # flight path angle
            self.alpha = self.theta - self.gamma  # Update alpha based on pitch angle and flight path angle

            print(f"V: {self.V}, V_mag: {self.V_mag}, gamma: {np.rad2deg(self.gamma)}, alpha: {np.rad2deg(self.alpha)}")

            # print(self.alpha)

            self.pos += self.V*self.dt


            self.t_history.append(self.t)
            self.Vx_history.append(self.V[0])
            self.Vy_history.append(self.V[1])
            self.X_history.append(self.pos[0])
            self.Y_history.append(self.pos[1])
            self.L_aero_history.append(self.L_aero[1])
            self.D_aero_history.append(self.D_aero[0])
            self.L_hydro_history.append(self.L_hydro[1])
            self.D_hydro_history.append(self.D_hydro[0])
            self.F_buoyancy_history.append(self.F_buoyancy[1])
            self.T_history.append(self.F_thrust[0])
            self.CL_history.append(self.CL)
            self.Cd_history.append(self.Cd)
            # Store gamma in degrees for plotting
            self.gamma_history.append(np.rad2deg(self.gamma))
            self.alpha_history.append(np.rad2deg(self.alpha))
            self.q_history.append(self.q)
            self.pitch_angle_history.append(np.rad2deg(self.theta))  # Store pitch angle in degrees
            self.V_mag_history.append(self.V_mag)
            self.q_dot_history.append(self.q_dot)
            self.t += self.dt

        return

    def plot_results(self):
        if not self.t_history:
            print("No simulation data to plot.")
            return
        fig, axs = plt.subplots(6, 3, figsize=(18, 24))
        axs = axs.flatten()

        axs[0].plot(self.t_history, self.X_history, color='b')
        axs[0].set_title('X (m)')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('X (m)')
        axs[0].grid()

        axs[1].plot(self.t_history, self.Y_history, color='m')
        axs[1].set_title('Y (m)')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Y (m)')
        axs[1].grid()

        axs[2].plot(self.t_history, self.Vx_history, color='g')
        axs[2].set_title('Vx (m/s)')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Vx (m/s)')
        axs[2].grid()

        axs[3].plot(self.t_history, self.Vy_history, color='c')
        axs[3].set_title('Vy (m/s)')
        axs[3].set_xlabel('Time (s)')
        axs[3].set_ylabel('Vy (m/s)')
        axs[3].grid()

        axs[4].plot(self.t_history, self.L_aero_history, color='r')
        axs[4].set_title('L_aero (N)')
        axs[4].set_xlabel('Time (s)')
        axs[4].set_ylabel('L_aero (N)')
        axs[4].grid()

        axs[5].plot(self.t_history, self.L_hydro_history, color='orange')
        axs[5].set_title('L_hydro (N)')
        axs[5].set_xlabel('Time (s)')
        axs[5].set_ylabel('L_hydro (N)')
        axs[5].grid()

        axs[6].plot(self.t_history, self.D_aero_history, color='k')
        axs[6].set_title('D_aero (N)')
        axs[6].set_xlabel('Time (s)')
        axs[6].set_ylabel('D_aero (N)')
        axs[6].grid()

        axs[7].plot(self.t_history, self.D_hydro_history, color='brown')
        axs[7].set_title('D_hydro (N)')
        axs[7].set_xlabel('Time (s)')
        axs[7].set_ylabel('D_hydro (N)')
        axs[7].grid()

        axs[8].plot(self.t_history, self.F_buoyancy_history, color='purple')
        axs[8].set_title('Buoyancy (N)')
        axs[8].set_xlabel('Time (s)')
        axs[8].set_ylabel('Buoyancy (N)')
        axs[8].grid()

        axs[9].plot(self.t_history, getattr(self, 'CL_history', []), color='navy')
        axs[9].set_title('CL')
        axs[9].set_xlabel('Time (s)')
        axs[9].set_ylabel('CL')
        axs[9].grid()

        axs[10].plot(self.t_history, getattr(self, 'Cd_history', []), color='teal')
        axs[10].set_title('Cd')
        axs[10].set_xlabel('Time (s)')
        axs[10].set_ylabel('Cd')
        axs[10].grid()

        axs[11].plot(self.t_history, getattr(self, 'alpha_history', []), color='darkred')
        axs[11].set_title('Alpha (deg)')
        axs[11].set_xlabel('Time (s)')
        axs[11].set_ylabel('Alpha (deg)')
        axs[11].grid()

        axs[12].plot(self.t_history, getattr(self, 'gamma_history', []), color='darkgreen')
        axs[12].set_title('Gamma (deg)')
        axs[12].set_xlabel('Time (s)')
        axs[12].set_ylabel('Gamma (deg)')
        axs[12].grid()

        axs[13].plot(self.t_history, getattr(self, 'V_mag_history', []), color='darkblue')
        axs[13].set_title('V_mag (m/s)')
        axs[13].set_xlabel('Time (s)')
        axs[13].set_ylabel('V_mag (m/s)')
        axs[13].grid()

        axs[14].plot(self.t_history, getattr(self, 'q_dot_history', []), color='darkviolet')
        axs[14].set_title('q_dot (rad/s^2)')
        axs[14].set_xlabel('Time (s)')
        axs[14].set_ylabel('q_dot (rad/s^2)')
        axs[14].grid()
        axs[15].plot(self.t_history, getattr(self, 'q_history', []), color='slateblue')
        axs[15].set_title('q (rad/s)')
        axs[15].set_xlabel('Time (s)')
        axs[15].set_ylabel('q (rad/s)')
        axs[15].grid()
        axs[16].plot(self.t_history, getattr(self, 'pitch_angle_history', []), color='crimson')
        axs[16].set_title('Pitch Angle (deg)')
        axs[16].set_xlabel('Time (s)')
        axs[16].set_ylabel('Pitch Angle (deg)')
        axs[16].grid()
        # Hide unused subplots
        for i in range(17, 18):
            fig.delaxes(axs[i])
        plt.subplots_adjust(
            top=0.97,
            bottom=0.04,
            left=0.05,
            right=0.97,
            hspace=0.45,
            wspace=0.30
        )

        plt.show()



if __name__ == "__main__":
    data = Data("design3.json")
    model = Take_off_mechanics(data)
    model.run_simulation()
    model.plot_results()
