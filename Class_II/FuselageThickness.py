import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from scipy.integrate import quad

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials, EvaluateType

class FuselageThickness:
    def __init__(self, aircraft_data: Data, airfoil_data: Data, fuselage_mat: Materials):
          
        self.data = airfoil_data.data
        self.aircraft_data = aircraft_data

        self.front_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['rear_spar']

        self.t_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['t_spar']/1000
        self.t_skin = self.aircraft_data.data['inputs']['structures']['wing_box']['t_skin']/1000
        self.t_wing = self.aircraft_data.data['inputs']['structures']['wing_box']['t_wing']/1000

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.b_array = np.arange(0, self.b/2, 0.01)
        # self.chord_array = self.chord_span_function(self.b_array)

        self.chord_length = self.chord_root

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']

        self.n_stringers = self.aircraft_data.data['inputs']['structures']['wing_box']['n_stringers']

        self.fuel_tank = self.aircraft_data.data['inputs']['fuel_tank']

        self.material = self.aircraft_data.data['inputs']['structures']['materials'][fuselage_mat.name.lower()]
        self.G = self.material['G']
        self.E = self.material['E']
        self.sigma_y = self.material['sigma_y']
        self.poisson_ratio = self.material['poisson_ratio']

        self.rho_fuselage = self.material['rho']

        self.fuselage_width = np.array([5.7, 5.7, 5.7])
        self.fuselage_height = np.array([4.5, 6, 4.5])
        self.fuselage_ratio =  self.fuselage_height / self.fuselage_width

        self.t_fuselage = self.aircraft_data.data['inputs']['t_fuselage']/1000


    def calculate_fuselage_centroid(self):

        self.a_dim = 0.5 / np.tan(np.radians(65))
        self.s_dim = 0.5 / np.sin(np.radians(65))
        self.o_dim = self.s_dim/2 * np.sin(np.radians(25))

        #Unit is meters
        self.y_bar_fuselage = (2*(self.fuselage_ratio*((self.a_dim + 0.4)*self.fuselage_width)) + 2*(self.s_dim * (self.o_dim * self.fuselage_width)) + ((self.a_dim + self.fuselage_ratio) * self.fuselage_width)) / (2*self.s_dim + 2*self.fuselage_ratio + 1)
        
        self.x_bar_fuselage = (self.s_dim * 0.25 * self.fuselage_width + (self.s_dim * 0.75 * self.fuselage_width) + (0.5*self.fuselage_width) + (self.fuselage_ratio*self.fuselage_width)) / (2*self.s_dim + 2*self.fuselage_ratio + 1)

        return self.x_bar_fuselage, self.y_bar_fuselage
        
    
    def calculate_fuselage_moi(self):
        # Thin walled assumption used for fuselage
        # All moments of inertia are calculated about the centroid of the fuselage
        # All moments of inertia still have to be multiplied by the thickness of the skin
        # Unit is m^4 --> means that t_skin must also be in METERS!

        self.I_xy_fuselage = 0 # * t_fuselage

        self.I_xx_fuselage = self.fuselage_width*(((self.fuselage_ratio + self.a_dim)*self.fuselage_width - self.y_bar_fuselage)**2) + 2*(((self.fuselage_ratio*self.fuselage_width)**3)/12 + self.fuselage_ratio*self.fuselage_width*((self.a_dim + self.fuselage_ratio/2)*self.fuselage_width - self.y_bar_fuselage)**2) + 2*(((self.s_dim**3 * (np.sin(np.radians(25)))**2)/12) + self.s_dim*(self.o_dim -self.y_bar_fuselage)**2) # * t_fuselage

        self.I_yy_fuselage = (self.fuselage_width**3) / 12 + 2*(self.fuselage_ratio*self.fuselage_width*(self.fuselage_width/2)**2) + 2*(self.s_dim*(0.25*self.fuselage_width)**2) # * self.t_fuselage

        return self.I_xx_fuselage, self.I_yy_fuselage, self.I_xy_fuselage
        
    
    def calculate_stresses(self):

        self.sigma_1 = (((M_x * self.I_yy_fuselage)*((self.a_dim+(self.fuselage_ratio*self.fuselage_width)-self.y_bar_fuselage))) + (M_y*self.I_xx_fuselage)*(-0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)
        self.sigma_2 = (((M_x * self.I_yy_fuselage)*((self.a_dim+(self.fuselage_ratio*self.fuselage_width)-self.y_bar_fuselage))) + (M_y*self.I_xx_fuselage)*(0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)

        self.sigma_3 = ((M_x * self.I_yy_fuselage) * -(self.y_bar_fuselage-self.a_dim) + (M_y *self.I_xx_fuselage)*(-0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)
        self.sigma_4 = ((M_x * self.I_yy_fuselage) * -(self.y_bar_fuselage-self.a_dim) + (M_y *self.I_xx_fuselage)*(0.5*self.fuselage_width)) / (self.I_xx_fuselage*self.I_yy_fuselage)

        self.sigma_5 = ((M_x * self.I_yy_fuselage)*(-self.y_bar_fuselage)) / (self.I_xx_fuselage*self.I_yy_fuselage)
        return self.sigma_1, self.sigma_2, self.sigma_3, self.sigma_4, self.sigma_5

    def calculate_boom_areas(self):
       
        # All boom areas are a function of the skin thickness
        w = self.fuselage_width
        r = self.fuselage_ratio
        s = self.s_dim

        B1 = (w / 6) * (2 + self.sigma_2 / self.sigma_1) + (r * w / 6) * (2 + self.sigma_3 / self.sigma_1)
        B2 = (w / 6) * (2 + self.sigma_1 / self.sigma_2) + (r * w / 6) * (2 + self.sigma_4 / self.sigma_2)
        B3 = (r * w / 6) * (2 + self.sigma_1 / self.sigma_3) + (s * w / 6) * (2 + self.sigma_5 / self.sigma_3)
        B4 = (r * w / 6) * (2 + self.sigma_2 / self.sigma_4) + (s * w / 6) * (2 + self.sigma_5 / self.sigma_4)
        B5 = (s * w / 6) * (2 + self.sigma_3 / self.sigma_5) + (s * w / 6) * (2 + self.sigma_4 / self.sigma_5)

        return B1, B2, B3, B4, B5


    def iterate_booms_per_station(
        self, 
        M_x, 
        M_y, 
        t_fuselage_init=0.005, 
        I_xx_init=10.0, 
        I_yy_init=3.0, 
        tol=1e-4, 
        max_iter=100, 
        alpha=0.5
    ):
        n_stations = len(self.fuselage_width)

        boom_areas_all = np.zeros((n_stations, 5))
        I_xx_all = np.zeros(n_stations)
        I_yy_all = np.zeros(n_stations)
        t_fuselage_final = np.zeros(n_stations)
        iterations_to_converge = np.zeros(n_stations, dtype=int)

        for i in range(n_stations):
            w = self.fuselage_width[i]
            h = self.fuselage_height[i]
            fr = h / w

            # Initial skin thickness (scalar or array)
            t = t_fuselage_init if np.isscalar(t_fuselage_init) else t_fuselage_init[i]

            # Initial MOI (scalar or array)
            I_xx_old = I_xx_init if np.isscalar(I_xx_init) else I_xx_init[i]
            I_yy_old = I_yy_init if np.isscalar(I_yy_init) else I_yy_init[i]

            # Geometry setup
            a_dim = 0.5 / np.tan(np.radians(65))
            s_dim = 0.5 / np.sin(np.radians(65))
            o_dim = s_dim / 2 * np.sin(np.radians(25))

            y_bar = (2 * (fr * ((a_dim + 0.4) * w)) + 2 * (s_dim * (o_dim * w)) + ((a_dim + fr) * w)) / (2 * s_dim + 2 * fr + 1)

            y_coords = np.array([
                (a_dim + fr * w - y_bar),
                (a_dim + fr * w - y_bar),
                (y_bar - a_dim),
                (y_bar - a_dim),
                -y_bar
            ])
            x_coords = np.array([
                -0.5 * w,
                0.5 * w,
                -0.5 * w,
                0.5 * w,
                0.0
            ])

            # Initial boom areas
            B_old = np.ones(5) * 0.01

            for iteration in range(max_iter):
                # --- Stress calculation using current MOIs ---
                sigma = np.zeros(5)
                for j in range(5):
                    sigma[j] = (M_x * I_yy_old * y_coords[j] + M_y * I_xx_old * x_coords[j]) / (I_xx_old * I_yy_old)

                # --- Boom area update ---
                B1 = (w / 6) * (2 + sigma[1] / sigma[0]) + (fr * w / 6) * (2 + sigma[2] / sigma[0])
                B2 = (w / 6) * (2 + sigma[0] / sigma[1]) + (fr * w / 6) * (2 + sigma[3] / sigma[1])
                B3 = (fr * w / 6) * (2 + sigma[0] / sigma[2]) + (s_dim * w / 6) * (2 + sigma[4] / sigma[2])
                B4 = (fr * w / 6) * (2 + sigma[1] / sigma[3]) + (s_dim * w / 6) * (2 + sigma[4] / sigma[3])
                B5 = (s_dim * w / 6) * (2 + sigma[2] / sigma[4]) + (s_dim * w / 6) * (2 + sigma[3] / sigma[4])
                B_new = np.array([B1, B2, B3, B4, B5]) * t

                # Under-relaxation: boom areas
                B_i = alpha * B_new + (1 - alpha) * B_old

                # --- Update MOIs from updated boom areas ---
                I_xx_new = np.sum((y_coords ** 2) * B_i)
                I_yy_new = np.sum((x_coords ** 2) * B_i)

                # Under-relaxation: moments of inertia
                I_xx_relaxed = alpha * I_xx_new + (1 - alpha) * I_xx_old
                I_yy_relaxed = alpha * I_yy_new + (1 - alpha) * I_yy_old

                # --- Optional thickness update based on stress ratio ---
                sigma_max = np.max(np.abs(sigma))
                t_new = t * (sigma_max / self.sigma_y)
                t = alpha * t_new + (1 - alpha) * t

                # --- Convergence check ---
                if (
                    np.all(np.abs(B_i - B_old) < tol) and
                    abs(I_xx_relaxed - I_xx_old) < tol and
                    abs(I_yy_relaxed - I_yy_old) < tol and
                    abs(t_new - t) < tol
                ):
                    iterations_to_converge[i] = iteration + 1
                    print(f"Station {i}: Converged in {iteration + 1} iteration(s)")
                    break

                # Update for next iteration
                B_old = B_i.copy()
                I_xx_old = I_xx_relaxed
                I_yy_old = I_yy_relaxed

            else:
                iterations_to_converge[i] = max_iter
                print(f"Warning: Station {i} did not converge after {max_iter} iterations.")

            # Save final results
            boom_areas_all[i, :] = B_i
            I_xx_all[i] = I_xx_relaxed
            I_yy_all[i] = I_yy_relaxed
            t_fuselage_final[i] = t

        return boom_areas_all, I_xx_all, I_yy_all, t_fuselage_final




if __name__ == '__main__':
    aircraft_data = Data("design3.json")
    airfoil_data = Data("AirfoilData/example_airfoil.json")
    fuselage_material = Materials.Al5052  
    
    fuselage = FuselageThickness(aircraft_data, airfoil_data, fuselage_material)
    
    M_x = 200e6  # Nm # TODO: Replace with actual bending moment
    M_y = 100e6  # Nm # TODO: Replace with actual moment

    boom_areas, I_xx_array, I_yy_array, t_fuselage = fuselage.iterate_booms_per_station(M_x, M_y)

    print("Boom Areas per Station(must still be multiplied by t_fuselage):\n", boom_areas)
    print("I_xx per Station(must still be multiplied by t_fuselage):\n", I_xx_array)
    print("I_yy per Station(must still be multiplied by t_fuselage):\n", I_yy_array)
    print("Fuselage Thickness per Station:\n", t_fuselage)