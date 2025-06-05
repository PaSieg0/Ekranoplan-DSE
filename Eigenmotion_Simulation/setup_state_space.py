from control.matlab import *
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from termcolor import colored

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, EigenMotion, EigenMotionType, ISA


class StateSpaceModel:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.stability_coefficients_sym = aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']
        self.stability_coefficients_asym = aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']

        self.time_step = 0.1
        self.time_vector = np.arange(0, 10, self.time_step)

        self.elevator_deflection = np.zeros_like(self.time_vector)
        self.aileron_deflection = np.zeros_like(self.time_vector)
        self.rudder_deflection = np.zeros_like(self.time_vector)

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.c = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.v = self.aircraft_data.data['requirements']['cruise_speed']
        self.MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        self.MTOM = self.aircraft_data.data['outputs']['max']['MTOM']
        self.rho = ISA(self.aircraft_data.data['inputs']['cruise_altitude']).rho
        self.S = self.aircraft_data.data['outputs']['wing_design']['S']

        CL = self.stability_coefficients_sym['CL']

        self.gamma_0 = np.deg2rad(0)  # Initial flight path angle (assumed level flight)

        mu_c = self.MTOM/(self.rho*self.S*self.c)
        C_z_alpha_dot = self.stability_coefficients_sym['C_Z_alphadot']
        self.I_yy = self.aircraft_data.data['outputs']['inertia']['I_yy']
        K_yy = self.I_yy/(self.MTOM*self.c**2)
        C_m_alpha_dot = self.stability_coefficients_sym['C_m_alphadot']
        C_Xu = self.stability_coefficients_sym['C_X_u']
        C_Xalpha = self.stability_coefficients_sym['C_x_alpha']
        C_Z0 = -self.MTOW*np.cos(self.gamma_0)/(0.5*self.rho*self.v**2*self.S)
        C_Zu = self.stability_coefficients_sym['C_Z_u']
        C_Zalpha = self.stability_coefficients_sym['C_z_alpha']
        C_X0 = self.MTOW*np.sin(self.gamma_0)/(0.5*self.rho*self.v**2*self.S)
        C_zq = self.stability_coefficients_sym['C_z_q']
        C_mu = self.stability_coefficients_sym['C_m_u']
        C_malpha = self.stability_coefficients_sym['C_m_alpha']
        C_mq = self.stability_coefficients_sym['C_m_q']


        C_Xdelta_e = self.stability_coefficients_sym['C_x_delta_e']
        C_zdelta_e = self.stability_coefficients_sym['C_z_delta_e']
        C_mdelta_e = self.stability_coefficients_sym['C_m_delta_e']

        C_Ybeta_dot = self.stability_coefficients_asym['CyBdot']
        C_nbeta_dot = self.stability_coefficients_asym['C_nBdot']
        self.I_xx = self.aircraft_data.data['outputs']['inertia']['I_xx']
        self.I_xz = self.aircraft_data.data['outputs']['inertia']['I_xz']
        self.I_zz = self.aircraft_data.data['outputs']['inertia']['I_zz']

        K_xx = self.I_xx/(self.MTOM*self.b**2)
        K_xz = self.I_xz/(self.MTOM*self.b*self.c)
        K_zz = self.I_zz/(self.MTOM*self.b**2)

        mu_b = self.MTOM/(self.rho*self.S*self.b)

        C_Ybeta = self.stability_coefficients_asym['CyB']
        C_Yp = self.stability_coefficients_asym['Cyp']
        C_Yr = self.stability_coefficients_asym['Cyr']
        C_lbeta = self.stability_coefficients_asym['ClB']
        C_lp = self.stability_coefficients_asym['Clp']
        C_lr = self.stability_coefficients_asym['Clr']
        C_nbeta = self.stability_coefficients_asym['CnB']
        C_np = self.stability_coefficients_asym['Cnp']

        C_Ydelta_a = self.stability_coefficients_asym['C_y_delta_a']
        C_Ydelta_r = self.stability_coefficients_asym['C_y_delta_r']
        C_ldelta_a = self.stability_coefficients_asym['C_l_delta_a']
        C_ldelta_r = self.stability_coefficients_asym['C_l_delta_r']
        C_ndelta_a = self.stability_coefficients_asym['C_n_delta_a']
        C_ndelta_r = self.stability_coefficients_asym['C_n_delta_r']
        

        P_sym = np.array([
            [-2*mu_c*self.c/self.v, 0                                   , 0             , 0                            ],
            [0                    , (C_z_alpha_dot-2*mu_c)*self.c/self.v, 0             , 0                            ],
            [0                    , 0                                   , -self.c/self.v, 0                            ],
            [0                    , C_m_alpha_dot*self.c/self.v         , 0             , -2*mu_c*K_yy**2*self.c/self.v]
        ])

        Q_sym = np.array([
            [-C_Xu, -C_Xalpha, -C_Z0, 0             ],
            [-C_Zu, -C_Zalpha, C_X0 , -(C_zq+2*mu_c)],
            [0    , 0        , 0    , -1            ],
            [-C_mu, -C_malpha, 0    , -C_mq         ]
        ])

        R_sym = np.array([
            [-C_Xdelta_e],
            [-C_zdelta_e],
            [0.0        ],
            [-C_mdelta_e]
        ])

        A_sym = np.linalg.solve(P_sym, -Q_sym)
        B_sym = np.linalg.solve(P_sym, -R_sym)


        C_sym = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, self.v/self.c]
        ])

        D_sym = np.array([
            [0],
            [0],
            [0],
            [0]
        ])
        
        P_asym = np.array([
            [(C_Ybeta_dot-2*mu_b)*self.b/self.v, 0                 , 0                            , 0                            ],
            [0                                 , -self.b/(2*self.v), 0                            , 0                            ],
            [0                                 , 0                 , -4*mu_b*K_xx**2*self.b/self.v, 4*mu_b*K_xz*self.b/self.v    ],
            [C_nbeta_dot*self.b/self.v         , 0                 , 4*mu_b*K_xz*self.b/self.v    , -4*mu_b*K_zz**2*self.b/self.v]
        ])

        Q_asym = np.array([
            [-C_Ybeta, -CL, -C_Yp, -(C_Yr-4*mu_b)],
            [0       , 0  , -1   , 0             ],
            [-C_lbeta, 0  , -C_lp, -C_lr         ],
            [-C_nbeta, 0  , -C_np, -C_lp         ]
        ])

        R_asym = np.array([
            [-C_Ydelta_a, -C_Ydelta_r],
            [0.0        , 0.0        ],
            [-C_ldelta_a, -C_ldelta_r],
            [-C_ndelta_a, -C_ndelta_r]
        ])
        A_asym = np.linalg.solve(P_asym, -Q_asym)
        B_asym = np.linalg.solve(P_asym, -R_asym)


        C_asym = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, self.v/self.b, 0.0],
            [0.0, 0.0, 0.0, self.v/self.b]
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

        self.elevator_deflection[:50] = np.deg2rad(1)
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
        stability_str = "STABLE" if eig.real < 0 else "UNSTABLE"
        stability_col = colored(f"{stability_str}", "green" if eig.real < 0 else "red")
        print(f"  λ{idx}: {eig.real:.4f} {'+' if eig.imag >= 0 else '-'} {abs(eig.imag):.4f}j  [{stability_col}]")
    print("\nEigenvalues for asymmetric model:")
    for idx, eig in enumerate(state_space_model.eigenvalues_asym, 1):
        stability_str = "STABLE" if eig.real < 0 else "UNSTABLE"
        stability_col = colored(f"{stability_str}", "green" if eig.real < 0 else "red")
        print(f"  λ{idx}: {eig.real:.4f} {'+' if eig.imag >= 0 else '-'} {abs(eig.imag):.4f}j  [{stability_col}]")