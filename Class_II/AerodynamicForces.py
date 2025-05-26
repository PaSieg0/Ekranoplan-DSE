import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data
from numpy.polynomial import Polynomial
import numpy as np

class AerodynamicForces:

    def __init__(self, aircraft_data: Data, airfoil_aerodynamics: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data

        self.aerodynamics = airfoil_aerodynamics

        self.yspan_values = self.aerodynamics.data['yspan']
        self.cl_values = self.aerodynamics.data['cl']
        self.induced_cd_values = self.aerodynamics.data['induced_cd']
        self.chord_values = self.aerodynamics.data['chord']
        self.cm_values = self.aerodynamics.data['cm']

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.b_array = np.arange(0, self.b/2, 0.01)

        self.rho = self.aircraft_data.data['rho_air']
        self.V = self.aircraft_data.data['requirements']['cruise_speed']

        self.airfoil_Cd0 = 0.00734 #TODO link to json data
        self.airfoil_Cd0_GE = 0.006 #TODO link to json data

    def plot_Lift_distribution(self):
        plt.figure()
        plt.plot(self.yspan_values, self.lift_values)
        plt.plot(self.b_array, self.fitted_l, label="Fitted L")
        plt.xlabel("y-span")
        plt.ylabel("L [N]")
        plt.title("L Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_Drag_distribution(self):
        plt.figure()
        plt.plot(self.yspan_values, self.drag_values)
        plt.plot(self.b_array, self.fitted_d, label="Fitted Cd")
        plt.xlabel("y-span")
        plt.ylabel("D [N]")
        plt.title("D Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_Moment_distribution(self):
        plt.figure()
        plt.plot(self.yspan_values, self.moment_values)
        plt.plot(self.b_array, self.fitted_M)
        plt.xlabel("y-span")
        plt.ylabel("M [Nm]")
        plt.title("Moment Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.show()

    def lift_distribution(self):

        self.lift_values = []
        for cl, c in zip(self.cl_values, self.chord_values):
            if cl < 0 or c < 0:
                raise ValueError("Cl values must be non-negative.")
            else:
                l = cl * (0.5*self.rho*self.V**2*c)
                self.lift_values.append(l)

    def drag_distribution(self):
        self.drag_values = []
        for cdi, c in zip(self.induced_cd_values, self.chord_values):
            if cdi < 0 or c < 0:
                raise ValueError("Cl values must be non-negative.")
            else:
                cd = self.airfoil_Cd0 + cdi
                d = cd * (0.5*self.rho*self.V**2*c)
                self.drag_values.append(d)

    def moment_distribution(self):
        self.moment_values = []
        for cm, c in zip(self.cm_values, self.chord_values):
            m = cm * (0.5*self.rho*self.V**2*c**2)
            self.moment_values.append(m)
    
    def get_lift_function(self):
        self.lift_distribution()
        self.L_y = Polynomial.fit(self.yspan_values, self.lift_values, 9)
        self.fitted_l = self.L_y(self.b_array)
        return self.L_y
    
    def get_drag_function(self):
        self.drag_distribution()
        self.D_y = Polynomial.fit(self.yspan_values, self.drag_values, 10)
        self.fitted_d = self.D_y(self.b_array)
        return self.D_y
    
    def get_moment_function(self):
        self.moment_distribution()
        self.M_y = Polynomial.fit(self.yspan_values, self.moment_values, 10)
        self.fitted_M = self.M_y(self.b_array)
        return self.M_y
        
if __name__ == "__main__":
    aerodynamics_data = Data("AeroForces.txt", "aerodynamics")
    aircraft_data = Data("design3.json", "design")

    aeroforces = AerodynamicForces(aircraft_data, aerodynamics_data)

    L_y = aeroforces.get_lift_function()
    D_y = aeroforces.get_drag_function()
    M_y = aeroforces.get_moment_function()

    aeroforces.plot_Lift_distribution()
    aeroforces.plot_Drag_distribution()
    aeroforces.plot_Moment_distribution()
