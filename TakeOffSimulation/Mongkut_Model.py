import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
import numpy as np
import matplotlib.pyplot as plt
from aero.lift_curve import lift_curve


class Simulation:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_file = f"design{self.aircraft_data.data['design_id']}.json"

        self.t_end = 120
        self.dt = 0.1
        self.t = 0.0

        self.lift_curve = lift_curve()

        self.C_delta_0 = self.calculate_C_delta_0()
        self.determine_thrust_function()


        self.x = 0
        self.y = -3.1
        self.v_x = 0
        self.v_y = 0
        self.a_x = 0
        self.a_y = 0

        self.time_list = []

        self.x_list = []
        self.y_list = []
        self.v_y_list = []
        self.v_x_list = []
        self.a_x_list = []
        self.a_y_list = []


        self.velocities = []
        self.R_list = []
        self.D_list = []
        self.Thrust_list = []
        self.R_froude_list = []
        self.buoyance_list = []
        self.total_drag_list = []
        self.total_power_list = []
        self.L_list = []
        self.F_x_list = []
        self.F_y_list = []

    def determine_thrust_function(self):
        vc = (self.aircraft_data.data['requirements']['cruise_speed'])
        vh = (self.aircraft_data.data['outputs']['optimum_speeds']['max'])
        T_static  = (110000)*6
        T_C = (312000)*1.5 #181000
        T_H = (223000)*1.5 #223000
        eta_p = self.aircraft_data.data['inputs']['engine']['engine_efficiency']
        P_BHP = self.aircraft_data.data['inputs']['engine']['engine_power']
        A_matrix = np.array([
            [0, 0, 0, 1],
            [vc**3, vc**2, vc, 1],
            [3*vc**2, 2*vc, 1, 0],
            [vh**3, vh**2, vh, 1]
        ])

        B_matrix = np.array([
            [T_static],
            [T_C],
            [-eta_p*P_BHP/(vc**2)],
            [T_H]
        ])

        x = np.linalg.solve(A_matrix, B_matrix)
        A, B, C, D = x.flatten()
        # print(A, B, C, D)
        self.Thrust_function = lambda x: A * x**3 + B * x**2 + C * x + D


    def calculate_C_delta_0(self):
        MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        rho_water = self.aircraft_data.data['rho_water']
        g = 9.81
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        return (MTOW) / (rho_water * g * B**3)
    
    def calculate_wetted_width(self):
        h = self.y
        if h <= -1.6:
            return self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        elif -1.6 < h <+ 0:
            return -self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'] * h / 1.6
        elif h >= 0:
            return 0
        
    def calculate_wetted_area(self):
        hull_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        wetted_width = self.calculate_wetted_width()
        if wetted_width < 0:
            raise ValueError("Wetted width cannot be negative. Check the height (h) value.")
        if wetted_width == 0:
            return 0
        h = self.y
        wetted_hypotenuse = 2*np.sqrt((wetted_width/2)**2 + h**2)
        wetted_area = hull_length * wetted_hypotenuse
        self.wetted_area = wetted_area
        return wetted_area
    
    def calculate_wetted_volume(self):
        if self.y <= -1.6:
            triangle_area = 0.5*1.6*self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
            rect_area = (-self.y - 1.6) * self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        if -1.6 < self.y <= 0:
            triangle_area = 0.5*-self.y*self.calculate_wetted_width()
            rect_area = 0
        if self.y > 0:
            return 0
        tot_area = triangle_area + rect_area
        hull_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        wetted_volume = hull_length * tot_area
        # print(wetted_volume)
        return wetted_volume



    def update_C_delta(self):
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'] #self.calculate_wetted_width()
        rho_water = self.aircraft_data.data['rho_water']
        g = 9.81
        # print(f"Wetted width (B): {B}")
        if B < 0:
            raise ValueError("Wetted width cannot be negative. Check the height (h) value.")
        if B == 0:
            # print("Returning infinity for C_delta due to zero wetted width.")
            return float('inf')
        self.C_delta = rho_water * g * self.calculate_wetted_volume() / (rho_water * g * B**3)

    def update_buoyancy(self):
        rho_water = self.aircraft_data.data['rho_water']
        g = 9.81
        wetted_volume = self.calculate_wetted_volume()
        self.buoyancy = rho_water * g * wetted_volume

    def update_C_v(self):
        v = self.v_x
        g = 9.81
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        self.C_v = (v) / (np.sqrt(g * B))

    def update_C_R(self):
        C_V = self.C_v
        self.C_R = 0.05789 * np.exp(-((C_V - 1.907) / (0.7561))**2) + 0.0273 * np.exp(-((C_V - 1.347) / (0.2536))**2) - 0.3322 * np.exp(-((C_V - 23.41) / (11.74))**2) + 0.07924 * np.exp(-((C_V - 3.227) / (1.951))**2)

    def update_C_R_delta(self):
        self.C_R_delta = self.C_R * (self.C_delta / self.C_delta_0)

    def update_R(self):
        rho_water = self.aircraft_data.data['rho_water']
        g = 9.81
        self.R = max(0, self.C_R_delta * rho_water * g * self.calculate_wetted_volume())

    def update_Thrust(self):
        v = self.v_x
        if v < 0:
            raise ValueError("Velocity cannot be negative.")
        self.Thrust = self.Thrust_function(v)

    def update_R_froude(self):
        f = 0.01
        Sw = self.calculate_wetted_area()
        n = 1.83
        self.R_froude = f * Sw * self.v_x**n


    def update_D(self):
        h_b = (self.y + self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']) / self.aircraft_data.data['outputs']['wing_design']['b']
        if self.v_x > 70:
            Cl_clean = self.aircraft_data.data['inputs']['CLmax_clean']
        else:
            Cl_clean = self.aircraft_data.data['inputs']['CLmax_takeoff']
        Cd = self.lift_curve.calc_drag_butbetter(h_b=h_b, cl=Cl_clean)
        self.D = 0.5 * 1.225 * (self.v_x**2) * self.aircraft_data.data['outputs']['wing_design']['S'] * Cd

    def update_L(self):
        h_b = (self.y + self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']) / self.aircraft_data.data['outputs']['wing_design']['b']
        if self.v_x > 70:
            Cl = self.aircraft_data.data['inputs']['CLmax_clean']
        else:
            Cl = self.aircraft_data.data['inputs']['CLmax_takeoff']
        CL_WIG = self.lift_curve.Cl_correction_GE(h_b=h_b, cl=Cl)
        self.L = 0.5 * 1.225 * (self.v_x**2) * self.aircraft_data.data['outputs']['wing_design']['S'] * CL_WIG


    def update_all_states(self):
        self.update_C_delta()
        self.update_C_v()
        self.update_C_R()
        self.update_C_R_delta()
        self.update_R()
        self.update_Thrust()
        self.update_R_froude()
        self.update_D()
        self.update_L()
        self.update_buoyancy()
        # print(f"Height: {self.h}, Velocity: {self.v}, C_R_delta: {self.C_R_delta}, R: {self.R}")
        # print(f"C_delta: {self.C_delta}, C_v: {self.C_v}, C_R: {self.C_R}")

    def store_all_states(self):
        self.time_list.append(self.t)
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.v_x_list.append(self.v_x)
        self.v_y_list.append(self.v_y)
        self.a_x_list.append(self.a_x)
        self.a_y_list.append(self.a_y)
        self.R_list.append(self.R)
        self.D_list.append(self.D)
        self.Thrust_list.append(self.Thrust*self.r)
        self.R_froude_list.append(self.R_froude)
        self.L_list.append(self.L)
        self.buoyance_list.append(self.buoyancy)
        self.total_drag = self.R + self.D + self.R_froude
        self.total_drag_list.append(self.total_drag)
        self.total_power = (self.Thrust * self.v_x)
        self.total_power_list.append(self.total_power)

    def plot_results(self):
        plt.plot(self.time_list, self.R_list, 'k-', label='R')
        plt.plot(self.time_list, self.D_list, 'r-', label='D')
        plt.plot(self.time_list, self.Thrust_list, 'g-', label='Thrust')
        plt.plot(self.time_list, self.R_froude_list, 'm-', label='R_froude')
        plt.plot(self.time_list, self.total_drag_list, 'c-', label='Total D')
        # plt.plot(self.time_list, self.y_list, "k-", label='y')
        # plt.plot(self.time_list, self.buoyance_list)
        plt.plot(self.time_list, self.L_list, label='L')
        plt.plot(self.time_list, self.v_x_list, label='V_X')
        # plt.plot(self.time_list, self.v_y_list, label='V_X')
        # plt.plot(self.time_list, self.a_x_list, label='A_X')
        plt.xlabel('Time (S)')
        plt.ylabel('Forces (N)')
        plt.title(f'Simulation at Height: {self.y:.2f} m')
        handles, labels = plt.gca().get_legend_handles_labels()
        unique = dict()
        for h, l in zip(handles, labels):
            if l not in unique:
                unique[l] = h
        plt.legend(unique.values(), unique.keys())
        plt.show()

        plt.plot(self.time_list, self.total_power_list)
        plt.show()

    def save_takeoff_power(self):
        to_speed = self.aircraft_data.data['requirements']['stall_speed_takeoff'] * 1.05
        diff_list = abs(np.array(self.v_x_list) - to_speed)
        takeoff_power = self.total_power_list[np.argmin(diff_list)]
        self.aircraft_data.data['outputs']['takeoff_power'] = takeoff_power
        self.aircraft_data.save_design(self.design_file)

        
    def run_simulation(self):
        self.r = 0
        self.update_all_states()
        self.store_all_states()

        # for v in range(0, 70):
        #     self.v_x = v
        #     self.update_Thrust()
        #     plt.plot(self.v_x, self.Thrust, 'ro-')
        # plt.show()
            
        while self.t < self.t_end:

            self.r = 0.1 + 0.9*self.t/10 if self.t < 10 else 1

            self.F_x = self.Thrust*self.r - self.R_froude - self.D - self.R
            self.a_x = self.F_x / self.aircraft_data.data['outputs']['max']['MTOM']
            self.F_y = self.buoyancy + self.L - self.aircraft_data.data['outputs']['max']['MTOW']
            self.a_y = self.F_y / self.aircraft_data.data['outputs']['max']['MTOM']

            self.v_x += self.a_x * self.dt
            print(self.v_x)
            self.v_y += self.a_y * self.dt

            self.x += self.v_x * self.dt
            self.y += self.v_y * self.dt

            self.t += self.dt

            self.update_all_states()
            self.store_all_states()

        self.plot_results()
        self.save_takeoff_power()

if __name__ == "__main__":
    aircraft_data = Data('design3.json')
    simulation = Simulation(aircraft_data)
    simulation.run_simulation()

    # print(simulation.Thrust_list[0])