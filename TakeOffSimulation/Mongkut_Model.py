import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, plt
import numpy as np
# import matplotlib.pyplot as plt
from aero.lift_curve import lift_curve


class Simulation:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_file = f"design{self.aircraft_data.data['design_id']}.json"

        self.t_end = 150
        self.dt = 0.1
        self.t = 0.0
        self.sea_state_factor = 1.75

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
        self.total_power_req_list = []
        self.total_power_ava_list = []
        self.L_list = []
        self.F_x_list = []
        self.F_y_list = []

    def determine_thrust_function(self):
        vc = (self.aircraft_data.data['requirements']['cruise_speed'])
        vh = (self.aircraft_data.data['outputs']['optimum_speeds']['max'])
        T_static  = (80000)*6
        T_C = (181000)*1.5 #181000
        T_H = (223000)*1.5 #223000
        eta_p = self.aircraft_data.data['inputs']['engine']['prop_efficiency']
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
        return 0.93*wetted_volume



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
        B = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        self.R = self.sea_state_factor*max(0, self.C_R_delta * rho_water * g * self.calculate_wetted_volume())

    def update_Thrust(self):
        v = np.sqrt(self.v_x**2)
        if v < 0:
            raise ValueError("Velocity cannot be negative.")
        elif self.v_x > self.aircraft_data.data['requirements']['cruise_speed']:
            self.Thrust = self.D
        else:
            self.Thrust = self.Thrust_function(v)

    def update_R_froude(self):
        f = 0.01
        Sw = self.calculate_wetted_area()
        n = 1.83
        self.R_froude = self.sea_state_factor*f * Sw * self.v_x**n


    def update_D(self):
        h_b = (self.y + self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']) / self.aircraft_data.data['outputs']['wing_design']['b']
        Cd = self.lift_curve.calc_drag_butbetter(h_b=h_b, cl=self.Cl)
        self.D = 0.5 * 1.225 * (self.v_x**2) * self.aircraft_data.data['outputs']['wing_design']['S'] * Cd

    def update_L(self):
        h_b = (self.y + self.aircraft_data.data['outputs']['fuselage_dimensions']['wing_height']) / self.aircraft_data.data['outputs']['wing_design']['b']
        CL_WIG = self.lift_curve.Cl_correction_GE(h_b=h_b, cl=self.Cl)
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
        self.total_power_req = (self.total_drag * self.v_x)
        self.total_power_ava = (self.Thrust * self.v_x)
        self.total_power_req_list.append(self.total_power_req)
        self.total_power_ava_list.append(self.total_power_ava)

    def plot_results(self):
        import matplotlib.pyplot as plt
        # 1. Plot D, total_drag, R_froude, R, Thrust vs time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_list, self.D_list, label='Aerodynamic Drag D', color='tab:red')
        plt.plot(self.time_list, self.total_drag_list, label='Total Drag', color='tab:orange')
        plt.plot(self.time_list, self.R_froude_list, label='Fiction Resistance (Froude)', color='tab:purple')
        plt.plot(self.time_list, self.R_list, label='Hydrodynamic Resistance', color='tab:blue')
        plt.plot(self.time_list, self.Thrust_list, label='Thrust', color='tab:green')
        plt.xlabel('Time [s]', fontsize=18)
        plt.ylabel('Force [N]', fontsize=18)
        # plt.title('Forces vs Time')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=18)
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # 2. Plot MTOW, Buoyancy, Lift, and Total Lift vs time
        MTOW = self.aircraft_data.data['outputs']['max'].get('MTOW', 0)
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_list, [MTOW]*len(self.time_list), label='Weight', color='black', linestyle='--')
        if hasattr(self, 'buoyance_list') and self.buoyance_list:
            plt.plot(self.time_list, self.buoyance_list, label='Buoyancy', color='tab:blue')
        else:
            plt.plot(self.time_list, [getattr(self, 'buoyancy', 0)]*len(self.time_list), label='Buoyancy', color='tab:orange')
        if hasattr(self, 'L_list') and self.L_list:
            plt.plot(self.time_list, self.L_list, label='Lift', color='tab:green')
        else:
            plt.plot(self.time_list, [getattr(self, 'L', 0)]*len(self.time_list), label='Lift', color='tab:purple')
        # Add total_lift = buoyancy + lift
        if hasattr(self, 'buoyance_list') and hasattr(self, 'L_list') and self.buoyance_list and self.L_list:
            total_lift = [b + l for b, l in zip(self.buoyance_list, self.L_list)]
            plt.plot(self.time_list, total_lift, label='Total Lift', color='tab:red', linestyle='-')
        else:
            total_lift = [getattr(self, 'buoyancy', 0) + getattr(self, 'L', 0)]*len(self.time_list)
            plt.plot(self.time_list, total_lift, label='Total Lift', color='tab:cyan', linestyle='-')
        plt.xlabel('Time [s]', fontsize=18)
        plt.ylabel('Force [N]', fontsize=18)
        # plt.title('MTOW, Buoyancy, Lift, and Total Lift vs Time')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=18)
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # 3. Plot y vs time with axhline at y=0
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_list, self.y_list, label='Vertical Position (y)', color='tab:blue')
        plt.axhline(0, color='k', linestyle='--', label='Waterline')
        plt.xlabel('Time [s]', fontsize=18)
        plt.ylabel('Vertical Position y [m]', fontsize=18)
        # plt.title('Vertical Position vs Time')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=18)
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        plt.plot(self.x_list, self.y_list, label='Flight Path', color='tab:blue')
        plt.xlabel('Horizontal Position x [m]', fontsize=18)
        plt.ylabel('Vertical Position y [m]', fontsize=18)
        # plt.title('Flight Path')
        plt.axhline(0, color='k', linestyle='--', label='Waterline')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=18)
        plt.grid(True)
        plt.tight_layout()

        plt.show()

    def save_takeoff_power(self):
        takeoff_power = max(self.total_power_ava_list)
        self.aircraft_data.data['outputs']['takeoff_power'] = takeoff_power
        self.aircraft_data.save_design(self.design_file)
        print(f"Takeoff power: {takeoff_power:,.0f} W")
        
    def run_simulation(self):
        self.r = 0
        self.Cl = self.aircraft_data.data['inputs']['CLmax_takeoff']
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
            # print(self.v_x)
            self.v_y += self.a_y * self.dt

            self.x += self.v_x * self.dt
            self.y += self.v_y * self.dt

            self.t += self.dt

            if self.y >= 2.4:
                if self.v_x > 58:
                    self.Cl = 1.9
                # if self.v_x > 60:
                #     self.Cl = 1.8
                # if self.v_x > 62:
                #     self.Cl = 1.7
                # if self.v_x > 67:
                #     self.Cl = 3
                # if self.v_x > 68:
                #     self.Cl = 1.4
                # if self.v_x > 75:
                #     self.Cl = 1.2
                # if self.v_x > 77:
                #     self.Cl = 1.0
                # if self.v_x > 85:
                #     self.Cl = 0.9
                # if self.v_x > 90:
                #     self.Cl = 0.78
                # if self.v_x > 95:
                #     self.Cl = 0.67
                # if self.v_x > 105:
                #     self.Cl = 0.61
                # if self.v_x > 110:
                #     self.Cl = 0.52
                # if self.v_x > 117:
                #     self.Cl = 0.46
                # if self.v_x > 120:
                #     self.Thrust = 0


            self.update_all_states()
            self.store_all_states()

        self.plot_results()
        self.save_takeoff_power()

if __name__ == "__main__":
    aircraft_data = Data('final_design.json')
    simulation = Simulation(aircraft_data)
    simulation.run_simulation()

    # print(simulation.Thrust_list[0])