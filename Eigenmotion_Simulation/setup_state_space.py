from control.matlab import *
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from termcolor import colored

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, EigenMotion, EigenMotionType


class StateSpaceModel:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.stability_coefficients = aircraft_data.data['outputs']['stability_coefficients']

        self.time_step = 0.1
        self.time_vector = np.arange(0, 100, self.time_step)

        self.elevator_deflection = np.zeros_like(self.time_vector)
        self.aileron_deflection = np.zeros_like(self.time_vector)
        self.rudder_deflection = np.zeros_like(self.time_vector)

        c = self.aircraft_data.data['outputs']['wing_design']['MAC']
        v = self.aircraft_data.data['requirements']['cruise_speed']

        mu_c = self.stability_coefficients['mu_c']
        C_z_alpha_dot = self.stability_coefficients['C_z_alpha_dot']
        K_y = self.stability_coefficients['K_y']
        C_m_alpha_dot = self.stability_coefficients['C_m_alpha_dot']
        C_Xu = self.stability_coefficients['C_Xu']
        C_Xalpha = self.stability_coefficients['C_Xalpha']
        C_Z0 = self.stability_coefficients['C_Z0']
        C_Zu = self.stability_coefficients['C_Zu']
        C_Zalpha = self.stability_coefficients['C_Zalpha']
        C_X0 = self.stability_coefficients['C_X0']
        C_zq = self.stability_coefficients['C_zq']
        C_mu = self.stability_coefficients['C_mu']
        C_malpha = self.stability_coefficients['C_malpha']
        C_mq = self.stability_coefficients['C_mq']


        C_Xdelta_e = self.stability_coefficients['C_Xdelta_e']
        C_zdelta_e = self.stability_coefficients['C_zdelta_e']
        C_mdelta_e = self.stability_coefficients['C_mdelta_e']

        C_Ybeta_dot = self.stability_coefficients['C_Ybeta_dot']
        C_nbeta_dot = self.stability_coefficients['C_nbeta_dot']
        K_X = self.stability_coefficients['K_X']
        K_XZ = self.stability_coefficients['K_XZ']
        K_Z = self.stability_coefficients['K_Z']
        b = self.aircraft_data.data['outputs']['wing_design']['b']
        mu_b = self.stability_coefficients['mu_b']

        C_Ybeta = self.stability_coefficients['C_Ybeta']
        CL = self.stability_coefficients['CL']
        C_Yp = self.stability_coefficients['C_Yp']
        C_Yr = self.stability_coefficients['C_Yr']
        C_lbeta = self.stability_coefficients['C_lbeta']
        C_lp = self.stability_coefficients['C_lp']
        C_lr = self.stability_coefficients['C_lr']
        C_nbeta = self.stability_coefficients['C_nbeta']
        C_np = self.stability_coefficients['C_np']
        C_nbeta = self.stability_coefficients['C_nbeta']
        C_nbeta_dot = self.stability_coefficients['C_nbeta_dot']
        C_Yr = self.stability_coefficients['C_Yr']
        C_Yp = self.stability_coefficients['C_Yp']

        C_Ydelta_a = self.stability_coefficients['C_Ydelta_a']
        C_Ydelta_r = self.stability_coefficients['C_Ydelta_r']
        C_ldelta_a = self.stability_coefficients['C_ldelta_a']
        C_ldelta_r = self.stability_coefficients['C_ldelta_r']
        C_ndelta_a = self.stability_coefficients['C_ndelta_a']
        C_ndelta_r = self.stability_coefficients['C_ndelta_r']
        

        P_sym = np.array([
            [-2*mu_c*c/v, 0, 0, 0],
            [0, (C_z_alpha_dot-2*mu_c)*c/v, 0, 0],
            [0, 0, -c/v, 0],
            [0, C_m_alpha_dot*c/v, 0, -2*mu_c*K_y**2*c/v]
        ])

        Q_sym = np.array([
            [-C_Xu, -C_Xalpha, -C_Z0, 0],
            [-C_Zu, -C_Zalpha, C_X0, -(C_zq+2*mu_c)],
            [0, 0, 0, -1],
            [-C_mu, -C_malpha, 0, -C_mq]
        ])

        R_sym = np.array([
            [-C_Xdelta_e],
            [-C_zdelta_e],
            [0.0],
            [-C_mdelta_e]
        ])

        A_sym = P_sym.inverse() @ Q_sym
        B_sym = P_sym.inverse() @ R_sym

        print("Symmetric Model A Matrix:")
        print(A_sym)
        print("Symmetric Model B Matrix:")
        print(B_sym)
        

        C_sym = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        D_sym = np.array([
            [0],
            [0],
            [0],
            [0]
        ])
        
        P_asym = np.array([
            [(C_Ybeta_dot-2*mu_b)*b/v, 0, 0, 0],
            [0, -b/(2*v), 0, 0],
            [0, 0, -4*mu_b*K_X**2*b/v, 4*mu_b*K_XZ*b/v],
            [C_nbeta_dot*b/v, 0, 4*mu_b*K_XZ*b/v, -4*mu_b*K_Z**2*b/v]
        ])

        Q_asym = np.array([
            [-C_Ybeta, -CL, -C_Yp, -(C_Yr-4*mu_b)],
            [0, 0, -1, 0],
            [-C_lbeta, 0, -C_lp, -C_lr],
            [-C_nbeta, 0, -C_np, -C_lp]
        ])

        R_asym = np.array([
            [-C_Ydelta_a, -C_Ydelta_r],
            [0.0, 0.0],
            [-C_ldelta_a, -C_ldelta_r],
            [-C_ndelta_a, -C_ndelta_r]
        ])

        A_asym = P_asym.inverse() @ Q_asym
        B_asym = P_asym.inverse() @ R_asym

        print("Asymmetric Model A Matrix:")
        print(A_asym)
        print("Asymmetric Model B Matrix:")
        print(B_asym)


        C_asym = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        D_asym = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0]
        ])
        
        self.sym_model = ss(A_sym, B_sym, C_sym, D_sym)
        self.asym_model = ss(A_asym, B_asym, C_asym, D_asym)

        self.eigenvalues_sym = np.linalg.eigvals(A_sym)
        self.eigenvalues_asym = np.linalg.eigvals(A_asym)

    def reset(self):
        self.elevator_deflection = np.zeros_like(self.time_vector)
        self.aileron_deflection = np.zeros_like(self.time_vector)
        self.rudder_deflection = np.zeros_like(self.time_vector)
        self.y = None
        self.x_out = None
        self.model = None


    def get_phugoid_input(self):
        self.reset()

        self.elevator_deflection[:50] = np.sin(self.time_vector[:50])
        return np.array(self.elevator_deflection)

    def get_short_period_input(self):
        self.reset()

        self.elevator_deflection[:1] = 0.1
        return np.array(self.elevator_deflection)

    def get_dutch_roll_input(self):
        self.reset()

        self.aileron_deflection[:1] = 0.1
        self.rudder_deflection[:1] = 0.1
        return np.array([self.aileron_deflection, self.rudder_deflection])

    def get_dutch_roll_damped_input(self):
        self.reset()

        self.aileron_deflection[:1] = 0.1
        self.rudder_deflection[:1] = 0.1
        return np.array([self.aileron_deflection, self.rudder_deflection])

    def get_roll_input(self):
        self.reset()

        self.aileron_deflection[:1] = 0.1
        return np.array([self.aileron_deflection, self.rudder_deflection])

    def get_spiral_input(self):
        self.reset()
        
        self.aileron_deflection[:1] = 0.1
        self.rudder_deflection[:1] = 0.1
        return np.array([self.aileron_deflection, self.rudder_deflection])

    def get_response(self, eigemotion_type: EigenMotion):
        if eigemotion_type == EigenMotion.PHUGOID:
            input_signal = self.get_phugoid_input()
            self.model = self.sym_model
            self.eigenmotion_type = EigenMotionType.SYMMETRIC
        elif eigemotion_type == EigenMotion.SHORT_PERIOD:
            input_signal = self.get_short_period_input()
            self.model = self.sym_model
            self.eigenmotion_type = EigenMotionType.SYMMETRIC
        elif eigemotion_type == EigenMotion.DUTCH_ROLL:
            input_signal = self.get_dutch_roll_input().T
            self.model = self.asym_model
            self.eigenmotion_type = EigenMotionType.ASYMMETRIC
        elif eigemotion_type == EigenMotion.DUTCH_ROLL_DAMPED:
            input_signal = self.get_dutch_roll_damped_input().T
            self.model = self.asym_model
            self.eigenmotion_type = EigenMotionType.ASYMMETRIC
        elif eigemotion_type == EigenMotion.ROLL:
            input_signal = self.get_roll_input().T
            self.model = self.asym_model
            self.eigenmotion_type = EigenMotionType.ASYMMETRIC
        elif eigemotion_type == EigenMotion.SPIRAL:
            input_signal = self.get_spiral_input().T
            self.model = self.asym_model
            self.eigenmotion_type = EigenMotionType.ASYMMETRIC
        else:
            raise ValueError("Invalid eigenmotion type")
        
        self.y, self.time_vector, self.x_out = lsim(sys=self.model, U=input_signal, T=self.time_vector)
        return self.y
    
    def plot_input(self, eigemotion_type: EigenMotion):
        """Plot the input(s) used for the given eigenmotion type."""
        # Ensure the input signals are set up
        self.get_response(eigemotion_type)
        plt.figure(figsize=(8, 4))
        if self.eigenmotion_type == EigenMotionType.SYMMETRIC:
            plt.plot(self.time_vector, self.elevator_deflection, label='Elevator')
            plt.title(f'Elevator Input for {eigemotion_type.name}')
            plt.xlabel('Time (s)')
            plt.ylabel('Deflection')
            plt.grid()
            plt.legend()
        elif self.eigenmotion_type == EigenMotionType.ASYMMETRIC:
            plt.plot(self.time_vector, self.aileron_deflection, label='Aileron')
            plt.plot(self.time_vector, self.rudder_deflection, label='Rudder')
            plt.title(f'Aileron and Rudder Inputs for {eigemotion_type.name}')
            plt.xlabel('Time (s)')
            plt.ylabel('Deflection')
            plt.grid()
            plt.legend()
        else:
            raise ValueError("Invalid eigenmotion type for input plotting")
        plt.tight_layout()
        plt.show()

    def plot_response(self, eigemotion_type: EigenMotion):
        self.get_response(eigemotion_type)


        fig, axs = plt.subplots(2, 2, figsize=(10, 8))
        if self.eigenmotion_type == EigenMotionType.SYMMETRIC:
            state_labels = ['û [m/s]', 'α [deg]', 'θ [deg]', 'q·c̄/V [idk]']
        elif self.eigenmotion_type == EigenMotionType.ASYMMETRIC:
            state_labels = ['β [deg]', 'φ [deg]', 'p·b/2V [idk]', 'r·b/2V [idk]']
        else:
            raise ValueError("Invalid eigenmotion type for plotting")
        axs = axs.flatten()

        for i in range(4):
            axs[i].plot(self.time_vector, self.y[:, i] if self.y.ndim > 1 else self.y)
            axs[i].set_title(state_labels[i])
            axs[i].set_xlabel('Time (s)')
            axs[i].set_ylabel('Response')
            axs[i].grid()

        plt.suptitle(f'Response for {eigemotion_type.name}', y=1.0, fontsize=16)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    # Example usage
    aircraft_data = Data('design3.json')
    state_space_model = StateSpaceModel(aircraft_data)
    eigenmotion_type = EigenMotion.PHUGOID


    state_space_model.plot_input(eigenmotion_type)
    state_space_model.plot_response(eigenmotion_type)

    print("Eigenvalues for symmetric model:")
    for idx, eig in enumerate(state_space_model.eigenvalues_sym, 1):
        stability = colored("STABLE", "green") if eig.real < 0 else colored("UNSTABLE", "red")
        print(f"  λ{idx}: {eig.real:.4f} {'+' if eig.imag >= 0 else '-'} {abs(eig.imag):.4f}j  [{stability}]")
    print("\nEigenvalues for asymmetric model:")
    for idx, eig in enumerate(state_space_model.eigenvalues_asym, 1):
        stability = colored("STABLE", "green") if eig.real < 0 else colored("UNSTABLE", "red")
        print(f"  λ{idx}: {eig.real:.4f} {'+' if eig.imag >= 0 else '-'} {abs(eig.imag):.4f}j  [{stability}]")