import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data
from AerodynamicForces import AerodynamicForces
from WingStructure import WingStructure
from scipy.integrate import quad
import numpy as np


class StressAnalysisWing(AerodynamicForces, WingStructure):

    def __init__(self, aircraft_data: Data, airfoil_aerodynamics: Data, airfoil_data: Data):
        AerodynamicForces.__init__(self, aircraft_data, airfoil_aerodynamics)
        WingStructure.__init__(self, aircraft_data, airfoil_data)
        self.get_wing_structure()
        self.safety_factor = 1.5
        self.lift_function = self.get_lift_function()
        self.drag_function = self.get_drag_function()
        self.moment_function = self.get_moment_function()

        self.engine_power = self.aircraft_data.data['inputs']['engine']['engine_power']
        self.engine_thrust = self.engine_power / self.V
        self.dy = np.gradient(self.b_array)
        self.E = 68.9e9
        self.max_load_factor = self.aircraft_data.data['outputs']['general']['nmax']
        self.min_load_factor = self.aircraft_data.data['outputs']['general']['nmin']
        self.evaluate_case = 'max'
        self.drag_array = -self.drag_function(self.b_array)
        self.I_xx_array = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
        self.I_yy_array = np.array([self.wing_structure[i]['I_yy'] for i in range(len(self.wing_structure))])
        self.I_xy_array = np.array([self.wing_structure[i]['I_xy'] for i in range(len(self.wing_structure))])
        self.sigma_y = 276e6  # Yield strength in Pa of AL6061-T6
        self.k_v = 1.5 
        self.poisson_ratio = 0.3

    def resultant_vertical_distribution(self):
        if self.evaluate_case == 'max':
            return self.max_load_factor*self.lift_function(self.b_array) - self.wing_weight_dist()
        elif self.evaluate_case == 'min':
            return self.min_load_factor*self.lift_function(self.b_array) + self.wing_weight_dist()
    
    def resultant_torque_distribution(self):
        quarter_chord_dist = [self.wing_structure[i]['quarter_chord_dist'] for i in range(len(self.wing_structure))]
        if self.evaluate_case == 'max':
            return self.max_load_factor*self.lift_function(self.b_array) * quarter_chord_dist + self.moment_function(self.b_array)
        elif self.evaluate_case == 'min':
            return self.min_load_factor*self.lift_function(self.b_array) * quarter_chord_dist + self.moment_function(self.b_array)
    
    def resultant_horizontal_distribution(self):
        self.horizontal_distribution = self.drag_array.copy()
        for engine_y in self.engine_positions:
            self.horizontal_distribution[int(round(engine_y,2)*100)] += self.engine_thrust

        
        return self.horizontal_distribution

    def internal_vertical_shear_force(self):

        load = self.resultant_vertical_distribution()

        Vy_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.Vy_internal = -Vy_flipped[::-1]

        return self.Vy_internal
    
    def internal_horizontal_shear_force(self):
        engine_load = np.zeros_like(self.b_array)
        for engine_y in self.engine_positions:
            engine_load[int(round(engine_y,2)*100)] = self.engine_thrust
        Vx_flipped_engines = np.cumsum(engine_load[::-1] * self.dy[::-1])

        load = self.drag_array
        Vx_flipped = np.cumsum(load[::-1] * self.dy[::-1]) 
        self.Vx_internal = -Vx_flipped[::-1] + Vx_flipped_engines[::-1] 
    
        return self.Vx_internal
    
    def internal_bending_moment_x(self):
        load = self.internal_vertical_shear_force()
        M_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.M_internal = M_flipped[::-1]
        return self.M_internal
    
    def internal_bending_moment_y(self):
        load = self.internal_horizontal_shear_force()
        M_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.M_internal_y = M_flipped[::-1]
        return self.M_internal_y
    
    def internal_torque(self,y):
        load = self.resultant_torque_distribution()
        T_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.T_internal = T_flipped[::-1]
        return self.T_internal
    
    def calculate_top_bending_stress(self): 
        y = [self.wing_structure[i]['spar_info']['max_y'] - self.wing_structure[i]['centroid'][1] for i in range(len(self.wing_structure))]
        x = [self.wing_structure[i]['centroid'][0] -self.wing_structure[i]['spar_info']['max_x'] for i in range(len(self.wing_structure))]
        self.top_bending_stress = self.safety_factor*((self.internal_bending_moment_x()*self.I_yy_array -(self.internal_bending_moment_y()*self.I_xy_array))*y + (self.internal_bending_moment_y()*self.I_xx_array - (self.internal_bending_moment_x()*self.I_xy_array))*x) / (self.I_xx_array*self.I_yy_array - self.I_xy_array**2)/1000000
        #print(y[0], self.internal_bending_moment_x()[0], self.I_xx_array[0])
        #self.top_bending_stress = self.internal_bending_moment_x()*y/(self.I_xx_array)/1000000
        return self.top_bending_stress

    def calculate_shear_stress(self):
        Vy_internal = self.internal_vertical_shear_force()
        avg_shear_stress = Vy_internal / [sum(self.wing_structure[i]['spar_info']['spar_heights_t']) for i in range(len(self.wing_structure))]
        self.max_shear_stress = avg_shear_stress * self.k_v
        return self.max_shear_stress
    
    def calculate_wing_deflection(self):
        moment = self.internal_bending_moment_x()
        moment_flipped = np.cumsum(moment[::-1]*self.dy[::-1])
        self.wing_deflection = np.cumsum(moment_flipped*self.dy[::-1]) / (-self.E * self.I_xx_array[::-1])

        return self.wing_deflection
    
    def calculate_wingtip_twist(self, dtheta_dz):
        sectional_twist_flipped = np.cumsum(dtheta_dz[::-1]*self.dy[::-1])
        self.wingtip_twist = np.trapz(sectional_twist_flipped[::-1])
        return self.wingtip_twist
    
    # def calculate_critical_buckling_load(self):
    #     """Calculate the critical buckling load using Euler's formula
    #     P_cr = (n^2 * pi^2 * E * I) / (L_e^2)
    #     L_e is effective length, L is the length of the wing
    #     One fixed, one free --> L_e / L = 2.0
    #     B.C. Fixed-Free --> v(0) = 0, v'(0) = 0"""
    #     n = 1 # Chosen to be conservative, as will result in lowest critical load
    #     K = 2.0  # Effective length factor for fixed-free boundary condition
    #     self.P_cr = (n**2 * np.pi**2 * self.E * self.I_xx) / (self.b/2*K)**2
    #     return self.P_cr

    def calculate_critical_buckling_stress(self):
        """
        Loop over each wing-box cell (using widths_top and widths_bottom from WingStructure),
        compute the critical buckling stress for the top panels and bottom panels separately.
        Returns:
            buckling_span_points (np.ndarray): span coordinate.
            buckling_stress_top (np.ndarray): computed buckling stress (in Pa) for the top panel.
            buckling_stress_bottom (np.ndarray): computed buckling stress (in Pa) for the bottom panel.
        """
        C = 4.0
        buckling_stress_top_list = []
        buckling_stress_bottom_list = []
        span_points = []
        
        for y in self.b_array:
            # Loop over each wing-box cell (number of cells = self.n_cells)
            for i in range(self.n_cells):
                sigma_cr_top = C * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.t_skin / self.widths_top[i])
                sigma_cr_bottom = C * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.t_skin / self.widths_bottom[i])
                buckling_stress_top_list.append(sigma_cr_top)
                buckling_stress_bottom_list.append(sigma_cr_bottom)
                span_points.append(y)
            
            self.buckling_stress_top = np.array(buckling_stress_top_list)
            self.buckling_stress_bottom = np.array(buckling_stress_bottom_list)
            self.buckling_span_points = np.array(span_points)

        return self.buckling_span_points, self.buckling_stress_top, self.buckling_stress_bottom
    
    def calculate_critical_buckling_shear_stress(self):
       
        k_s = 10
        buckling_stress_top_list = []
        buckling_stress_bottom_list = []
        span_points = []
        
        for y in self.b_array:
            # Loop over each wing-box cell (number of cells = self.n_cells)
            for i in range(self.n_cells):
                sigma_cr_top = k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.t_spar / self.widths_top[i])
                sigma_cr_bottom = k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.t_spar / self.widths_bottom[i])
                buckling_stress_top_list.append(sigma_cr_top)
                buckling_stress_bottom_list.append(sigma_cr_bottom)
                span_points.append(y)
            
            self.buckling_stress_top = np.array(buckling_stress_top_list)
            self.buckling_stress_bottom = np.array(buckling_stress_bottom_list)
            self.buckling_span_points = np.array(span_points)

        return self.buckling_span_points, self.buckling_stress_top, self.buckling_stress_bottom
    

    
    def plot_internal_torque(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.internal_torque(self.b_array), label='Torque', color='purple')

        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Torque [Nm]')
        plt.title('Torque Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_deflection(self):

        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.calculate_wing_deflection(), label='Deflection', color='orange')
        plt.axhline(0.15*self.b/2, color='black', linestyle='--', linewidth=0.5)  # Horizontal line at y=0
        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Deflection [m]')
        plt.title('Wing Deflection Distribution over Span')
        plt.grid(True)
        plt.legend()
        plt.show()
    
    def plot_internal_vertical_shear_force(self):
        
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.internal_vertical_shear_force(), label='Vertical Shear Force', color='blue')
        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Shear Force [N]')
        plt.title('Vertical Shear Force Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_internal_horizontal_shear_force(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.internal_horizontal_shear_force(), label='Horizontal Shear Force', color='red')
        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Shear Force [N]')
        plt.title('Horizontal Shear Force Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()
    
    def plot_internal_moment_x(self):
        
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.internal_bending_moment_x(), label='Bending Moment', color='green')

        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Moment [Nm]')
        plt.title('Bending Moment Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_internal_moment_y(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.internal_bending_moment_y(), label='Bending Moment Y', color='orange')

        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Moment [Nm]')
        plt.title('Bending Moment Y Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_horizontal_distribution(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.resultant_horizontal_distribution(), label='Drag Distribution', color='red')
        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Force [N]')
        plt.title('Horizontal Force Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()
    
    def plot_vertical_distribution(self):
        
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.resultant_vertical_distribution(), label='Vertical Distribution', color='blue')

        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Force [N]')
        plt.title('Vertical Force Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_bending_stress(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.b_array, self.calculate_top_bending_stress(), label='Top Bending Stress', color='purple')

        plt.xlabel('Spanwise Position (y) [m]')
        plt.ylabel('Stress [MPa]')
        plt.title('Top Bending Stress Distribution over Wing Span')
        plt.grid(True)
        plt.legend()
        plt.show()

if __name__ == "__main__":

    stress_analysis = StressAnalysisWing(
        aircraft_data=Data("design3.json"),
        airfoil_aerodynamics=Data("AeroForces.txt", 'aerodynamics'),
        airfoil_data=Data("Airfoil_data.dat", 'airfoil_geometry')

    )
    print(stress_analysis.calculate_shear_stress())


    


        