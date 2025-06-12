import numpy as np
import scipy.linalg as ln
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials, EvaluateType
import Flutter_functions.typical_section_3DOF as ts3
import Flutter_functions.theodorsen_aerodynamics as ta
import Flutter_functions.wagner_aerodynamics as wa

np.set_printoptions(precision=4, linewidth=400)

# region Parameters:

# nn =1
# U = np.array([10.0])

# nn = 1
# U = np.array([40.9499])

# UNDERLYING LINEAR SYSTEM SETTINGS:
# ----------------------------------
class FlutterAnalysis:

    def __init__(self, aircraft_data: Data, wing_mat: Materials, evaluate: EvaluateType = EvaluateType.WING):
        self.eps = 1e-13
        self.nn = 100
        self.aircraft_data = aircraft_data
        self.evaluate = evaluate
        # Change this to maximum speed of our aircraft in m/s
        self.u_min = self.aircraft_data.data['outputs']['optimum_speeds']['max_aoc']
        self.u_max = self.aircraft_data.data['outputs']['optimum_speeds']['max']
        self.U = np.linspace(self.u_min, self.u_max, self.nn)
        self.legend_entries = [r'$h$', r'$\alpha0$', r'$\beta$']
        self.shape_coeff = np.array([1.875, 4.694, 7.855, 10.996])

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.n = np.array([1, 2, 3, 4])
        key_string_map = {
            EvaluateType.WING: 'wing_box',
            EvaluateType.HORIZONTAL: 'horizontal_wing_box',
            EvaluateType.VERTICAL: 'vertical_wing_box'
        }

        self.wing_box_string = key_string_map[self.evaluate]
        self.plot_colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 
                    'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray']

        self.Ip = self.aircraft_data.data['inputs']['structures'][self.wing_box_string]['Ip_MAC']
        self.I_xx = self.aircraft_data.data['inputs']['structures'][self.wing_box_string]['I_xx_MAC']
        self.J = self.aircraft_data.data['inputs']['structures'][self.wing_box_string]['J_MAC']
        self.section_mass = self.aircraft_data.data['inputs']['structures'][self.wing_box_string]['section_mass_MAC']
        self.wing_material = self.aircraft_data.data['inputs']['structures']['materials'][wing_mat.name.lower()]
        self.E = self.wing_material['E']
        self.G = self.wing_material['G']

        self.damping_coeff = 0.02

        # ----------------------------------


        # OVERLYING LINEAR SYSTEM SETTINGS:
        # ---------------------------------
        # eps = 1e-13
        # nn = 200
        # u_max = 80
        # U = np.linspace(0, u_max, nn)
        # legend_entries = [r'$h$', r'$\beta$', r'$\alpha$']
        #
        # omega_h = 2*pi*2.
        # omega_th = 2*pi*80.
        # omega_bt = 2*pi*10.
        # # ---------------------------------


    def get_bending_fn(self,beta):
        self.bending_fn = beta**2/2/np.pi/(self.b/2)**2 * np.sqrt(self.E*self.I_xx/(self.section_mass/0.01))
        return self.bending_fn
    
    def get_torsion_fn(self,n):
        self.torsion_fn = n/2/(self.b/2)*np.sqrt(self.G*self.J/(self.Ip))
        return self.torsion_fn

    def initialize_analysis(self,beta,n):
        self.omega_h = 2*np.pi*self.get_bending_fn(beta)
        self.omega_th = 2*np.pi*self.get_torsion_fn(n)
        self.omega_bt = 2*np.pi*10
        self.constant = ts3.get_constant(self.aircraft_data, self.wing_box_string)

        self.Kh = self.omega_h**2*self.constant['m']
        self.Kth = self.omega_th**2*self.constant['Ith']
        self.Kbt = self.omega_bt**2*self.constant['Ibt']

        # Change these damping values to the critical damping coef * percentage (0.02-0.05)
        self.Ch  = 2*np.sqrt(self.Kh*self.constant['m'])* self.damping_coeff
        self.Cth = 2*np.sqrt(self.Kth*self.constant['Ith'])* self.damping_coeff
        self.Cbt = 2*np.sqrt(self.Kbt*self.constant['Ibt'])* self.damping_coeff

        self.constant['Kh'] = self.Kh
        self.constant['Kth'] = self.Kth
        self.constant['Kbt'] = self.Kbt

        self.constant['Ch'] = self.Ch
        self.constant['Cth'] = self.Cth
        self.constant['Cbt'] = self.Cbt

        # Aerodynamic parameters and constants:
        self.T = ta.theodorsen_coefficients(self.constant)
        self.coefW = wa.get_approximation_coefficients()

        # endregion

        # region Eigenvalue analysis - velocity sweep:

    def eigenvalue_analysis(self):
        self.w_n = np.zeros((self.nn,3))
        self.zeta = np.zeros((self.nn,3))

        for i, U_i in enumerate(self.U):

            # Assemble 3DOF system matrix
            x0 = np.identity(12) # dummy input to retrieve the 3DOF system matrix
            Q = ts3.typical_section_3DOF_lin(x0, U_i, self.constant,self.T, self.coefW)

            # Calculate eigen values and eigen vectors
            eval,evec = ln.eig(Q)

            # sort out the complex-conjugate eigenvalues and their pertinent eigen vectors:
            eval_cc = eval[eval.imag > self.eps]
            evec_cc = evec[:, eval.imag > self.eps]

            idx = np.argsort(eval_cc.imag)

            eval_cc = eval_cc[idx]
            evec_cc = evec_cc[:, idx]

            # uncomment this part if the you are interrested in 1 paritcular velocity only:
            # -----------------------------------------------------------------------------
            # modes = evec_cc.real[:3] #/ln.norm(evec_cc.real[3:6], axis=0)
            #
            # print('eig. values: {}\n'.format(eval_cc))
            # print('eig. vectors:\n {}\n'.format(evec_cc))
            # print('modes:\n {}\n'.format(modes))
            # ------------------------------------------------------------------------------

            if len(eval_cc)>3:
                print(eval_cc)
            self.w_n[i,:] = np.absolute(eval_cc)
            self.zeta[i,:] = -eval_cc.real/np.absolute(eval_cc)

            if np.any(self.zeta[i, :] < 0):
                raise ValueError(f'Warning, flutter detected at U = {self.U[i]:.2f} m/s for mode {self.idx+1}')

        # endregion

        # region plot:

    def plot_results(self, ax): 
        # Use a color per mode index
        color = self.plot_colors[self.idx % len(self.plot_colors)]

        linestyles = ['-', '--']  # Solid for h, dashed for alpha0

        for i in range(2):  # h and alpha0
            ax[0].plot(self.U, self.w_n[:, i],
                    label=f'{self.legend_entries[i]} mode {self.idx+1}',
                    color=color, linestyle=linestyles[i])
            ax[1].plot(self.U, self.zeta[:, i],
                    color=color, linestyle=linestyles[i])  # No legend for ax[1]

        ax[0].set_ylabel('omega_n [rad/s]')
        ax[1].set_xlabel('U [m/s]')
        ax[1].set_ylabel('zeta [-]')
    
    def main(self, plot=True):
        self.idx = 0
        if plot:
            fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

        self.global_min_zeta = np.inf
        self.global_min_zeta_velocity = None
        self.global_min_zeta_mode = None
        
        for i in range(len(self.shape_coeff[:2])):
            beta = self.shape_coeff[i]
            n = self.n[i]
            self.initialize_analysis(beta, n)
            self.eigenvalue_analysis()
            if plot:
                self.plot_results(ax)

            min_zeta = np.min(self.zeta[:,:2])
            min_idx = np.unravel_index(np.argmin(self.zeta[:,:2]), self.zeta[:,:2].shape)
            min_velocity = self.U[min_idx[0]]
            if min_zeta < self.global_min_zeta:
                self.global_min_zeta = min_zeta
                self.global_min_zeta_velocity = min_velocity
                self.global_min_zeta_mode = f"Mode {self.idx+1}, DOF {self.legend_entries[min_idx[1]]}"
            self.idx += 1
        print(f"Global minimum zeta: {self.global_min_zeta:.4f} at U = {self.global_min_zeta_velocity:.2f} m/s ({self.global_min_zeta_mode})")

        if plot:
            ax[0].set_xlim([0, self.u_max])
            ax[0].axis('tight') 
            ax[1].axis('tight')

            # Legend on top subplot, positioned to the right
            ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.5), title='Modes')
            
            ax[0].grid(True)
            ax[1].grid(True)

            plt.tight_layout()
            plt.show()
        
        self.update_attributes()

    def update_attributes(self):
        key_map = {
            EvaluateType.WING: 'wing_stress',
            EvaluateType.HORIZONTAL: 'horizontal_tail_stress',
            EvaluateType.VERTICAL: 'vertical_tail_stress'
        }
        self.aircraft_data.data['outputs'][key_map[self.evaluate]]['V_min_damp'] = self.global_min_zeta_velocity
        self.aircraft_data.data['outputs'][key_map[self.evaluate]]['min_damp'] = self.global_min_zeta
        self.aircraft_data.data['outputs'][key_map[self.evaluate]]['min_damp_mode'] = self.global_min_zeta_mode

        self.aircraft_data.save_design(design_file=f"design{self.aircraft_data.data['design_id']}.json")

if __name__ == '__main__':
    aircraft_data = Data('design3.json')
    wing_material = Materials.Al7075
    evaluate_type = EvaluateType.VERTICAL
    flutter_analysis = FlutterAnalysis(aircraft_data, wing_material, evaluate= evaluate_type)
    flutter_analysis.main(plot=True)
