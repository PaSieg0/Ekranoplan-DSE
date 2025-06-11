import numpy as np
import scipy.linalg as ln
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials
import common.typical_section_3DOF as ts3
import common.theodorsen_aerodynamics as ta
import common.wagner_aerodynamics as wa

np.set_printoptions(precision=4, linewidth=400)

# region Parameters:

# nn =1
# U = np.array([10.0])

# nn = 1
# U = np.array([40.9499])

# UNDERLYING LINEAR SYSTEM SETTINGS:
# ----------------------------------
class FlutterAnalysis:

    def __init__(self, aircraft_data: Data, wing_mat: Materials):
        self.eps = 1e-13
        self.nn = 100
        # Change this to maximum speed of our aircraft in m/s
        self.u_max = 45
        self.aircraft_data = aircraft_data
        self.U = np.linspace(0, self.u_max, self.nn)
        self.legend_entries = [r'$h$', r'$\alpha0$', r'$\beta$']
        self.shape_coeff = np.array([1.875, 4.694, 7.855, 10.996])

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.n = np.array([1, 2, 3, 4])

        self.Ip = self.aircraft_data.data['outputs']['wing_stress']['Ip_MAC']
        self.I_xx = self.aircraft_data.data['outputs']['wing_stress']['Ixx_MAC']
        self.J = self.aircraft_data.data['outputs']['wing_stress']['J_MAC']
        self.section_mass = self.aircraft_data.data['outputs']['wing_stress']['section_mass_MAC']
        self.wing_material = self.aircraft_data.data['inputs']['structures'][wing_mat.name.lower()]
        self.E = self.wing_material['E']
        self.G = self.wing_material['G']

        # Change these initial values to the ones outputted by nat freq formulas
        self.omega_h = 2*np.pi*self.get_bending_fn()
        self.omega_th = 2*np.pi*self.get_torsion_fn()
        self.omega_bt = 2*np.pi*10

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


    def get_bending_fn(self):
        self.bending_fn = self.shape_coeff**2/2/np.pi/(self.b/2)**2 * np.sqrt(self.E*self.I_xx/self.section_mass)
        return self.bending_fn
    
    def get_torsion_fn(self):
        self.torsion_fn = self.n/2/(self.b/2)*np.sqrt(self.G*self.J/self.Ip)
        return self.torsion_fn

    def initialize_analysis(self):
        self.constant = ts3.get_constant(self.aircraft_data)

        self.Kh = self.omega_h**2*self.constant['m']
        self.Kth = self.omega_th**2*self.constant['Ith']
        self.Kbt = self.omega_bt**2*self.constant['Ibt']

        # Change these damping values to the critical damping coef * percentage (0.02-0.05)
        self.Ch  = self.Kh/1000.
        self.Cth = self.Kth/1000.
        self.Cbt = self.Kbt/1000.

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

            print(i)

        # endregion

        # region plot:

    def plot_results(self):
        fig, ax = plt.subplots(2,1, sharex=True)

        for i in range(3):
            ax[0].plot(self.U, self.w_n[:,i]/(2*np.pi))
            ax[1].plot(self.U, self.zeta[:, i])

        ax[0].set_ylabel('omega_n [Hz]')
        ax[1].set_xlabel('U [m/s]')
        ax[1].set_ylabel('zeta [-]')

        ax[0].axis('tight')
        ax[1].axis('tight')

        ax[0].set_ylim([0,20])
        ax[1].set_ylim([-0.2,0.5])
        ax[0].set_xlim([0,self.u_max])

        ax[0].legend(self.legend_entries)
        ax[0].grid(True)
        ax[1].grid(True)


        plt.show()
        # endregion
