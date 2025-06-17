import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, plt, ft2m, W2hp, ftsq2msq, lbf2N, kts2ms, lbsperft32kgperm3, N2lbf, ms2kts
import numpy as np
# import matplotlib.pyplot as plt
from aero.lift_curve import lift_curve
from TakeOffSimulation.Mongkut_Model import Simulation



class Verification(Simulation):
    def __init__(self, aircraft_data: Data):
        super().__init__(aircraft_data)
        self.verify = True

        self.t_end = 100
        self.dt = 0.01

        self.B = ft2m(4.27)
        self.Length = ft2m(13.55)
        self.b = ft2m(32.94)
        self.S = ftsq2msq(152.82)
        self.MTOW = lbf2N(1430)
        self.MTOM = self.MTOW / 9.81
        self.P_bhp = 115
        self.V_h = kts2ms(151)
        self.eta_p = 0.82
        self.T_h = lbf2N(1972.32)
        self.V_c = kts2ms(141.47)
        self.T_c = lbf2N(1277.82)
        self.T_static = lbf2N(2300)
        self.Cl = 0.2

        self.determine_thrust_function()

        self.A = 7.1
        self.e = 0.6

        self.f = 0.005
        self.n = 1.83

        print(f"MTOW: {self.MTOW:.2f} N")
        print(f"V_h: {self.V_h:.2f} m/s")
        print(f"V_c: {self.V_c:.2f} m/s")
        print(f"B: {self.B:.2f} m")
        print(f"b: {self.b:.2f} m")
        print(f"S: {self.S:.2f} m²")
        print(f"P_bhp: {self.P_bhp:.2f} hp")
        print(f"T_h: {self.T_h:.2f} N")

        self.y = -self.MTOW / (self.g*self.B*self.Length*self.rho_water)
        print(f"y: {self.y:.2f} N/m³")


    def verify_CR_Cv(self):
        Cvs = []
        CRs = []
        for  v in np.arange(0, 70, 0.5):
            self.v_x = v
            self.update_C_v()
            self.update_C_R()
            Cvs.append(self.C_v)
            CRs.append(self.C_R)

        plt.xlim(0, 5.0)
        plt.ylim(-0.05, 0.2)
        plt.plot(Cvs, CRs, color='tab:blue', marker='o', markersize=5, label='C_R vs C_v')
        plt.xlabel('Cv', fontsize=18)
        plt.ylabel('CR', fontsize=18)
        # plt.title('C_R vs C_v', fontsize=20)
        plt.show()

    def verify_thrust(self):
        self.determine_thrust_function()
        Ts = []
        for v in np.arange(0, 40, 0.5):
            self.v_x = v
            self.update_Thrust()
            Ts.append(N2lbf(self.Thrust))
        # plt.xlim(0, 80)
        # plt.ylim(0, 2500)
        plt.plot(ms2kts(np.arange(0, 40, 0.5)), Ts, color='tab:orange', label='Thrust vs Velocity')
        plt.show()


    def verify_drags(self):
        self.run_simulation()
        fig, ax1 = plt.subplots(figsize=(12, 8))
        ax1.set_xlabel('Velocity (KCAS)', fontsize=18)
        ax1.set_ylabel('Buoyancy (lbf)', fontsize=18, color='tab:purple')
        ax1.tick_params(axis='y', labelcolor='tab:purple')
        ax1.set_ylim(0, 3500)
        plt.grid(True)
        plt.plot((np.array(self.v_x_list)), N2lbf(np.array(self.buoyance_list)), color='tab:purple', label='Buoyancy vs Velocity')
        ax2 = ax1.twinx()
        ax2.set_ylabel('Force (lbf)', fontsize=18)
        ax2.tick_params(axis='y', labelcolor='tab:blue')
        ax2.set_xlim(0, 70)
        ax2.set_ylim(0, 500)
        self.total_drag_list = N2lbf(N2lbf(np.array(self.D_list))) + N2lbf(np.array(self.R_froude_list)) + N2lbf(np.array(self.R_list))
        print(self.total_drag_list)
        plt.plot((np.array(self.v_x_list)), N2lbf(N2lbf(np.array(self.D_list))), color='tab:green', label='Drag vs Velocity')
        plt.plot((np.array(self.v_x_list)), N2lbf(N2lbf(np.array(self.L_list))), color='tab:blue', label='Lift vs Velocity')
        plt.plot((np.array(self.v_x_list)), N2lbf(np.array(self.Thrust_list)), color='tab:orange', label='Thrust vs Velocity')
        plt.plot((np.array(self.v_x_list)), N2lbf(np.array(self.R_froude_list)), color='tab:red', label='Froude Number vs Velocity')
        plt.plot((np.array(self.v_x_list)), N2lbf(np.array(self.R_list)), color='tab:purple', label='R vs Velocity')
        plt.plot((np.array(self.v_x_list)), ((np.array(self.total_drag_list))), color='tab:orange', label='Total Drag vs Velocity')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.xlabel('Velocity (KCAS)', fontsize=18)
        plt.ylabel('Force (lbf)', fontsize=18)
        plt.title('Drag, Lift, Thrust, Froude Number, and R vs Velocity', fontsize=20)
        plt.xlim(0, 75)
        # plt.ylim(0, 2500)
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    aircraft_data = Data('final_design.json')
    simulation = Verification(aircraft_data)


    # simulation.verify_CR_Cv()
    # simulation.verify_thrust()
    simulation.verify_drags()

    

