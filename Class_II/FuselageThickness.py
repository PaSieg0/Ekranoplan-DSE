import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from scipy.integrate import quad
from weight_distributions import CGCalculation
from AerodynamicForces import AerodynamicForces

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials

class FuselageThickness:
    def __init__(self, aircraft_data: Data, fuselage_mat: Materials):
          
        self.aircraft_data = aircraft_data

        self.front_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['rear_spar']

        self.t_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['t_spar']/1000
        self.t_skin = self.aircraft_data.data['inputs']['structures']['wing_box']['t_skin']/1000
        self.t_wing = self.aircraft_data.data['inputs']['structures']['wing_box']['t_wing']/1000

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.b_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.b_array = np.arange(0, self.b/2, 0.01)
        self.b_h_array = np.arange(0, self.b_h/2 + 0.01, 0.01)
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

        self.fuselage_width = np.array([self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'], self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'], self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']])
        self.fuselage_height = np.array([self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station1'], self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station2'], self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station3']])
        self.fuselage_ratio =  self.fuselage_height / self.fuselage_width
        self.l_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        self.wing_LE_pos = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.lift_acting_point = self.wing_LE_pos + 1/4* self.chord_root

        self.t_fuselage = self.aircraft_data.data['inputs']['t_fuselage']/1000

        self.b_array = np.arange(0, self.b/2 + 0.01, 0.01)

        aerodynamics = Data("AeroForces.txt", "aerodynamics")
        self.aeroforces = AerodynamicForces(self.aircraft_data, aerodynamics)
        fuselage_mass = CGCalculation(self.aircraft_data)
        fuselage_mass.load_diagram()
        self.distributed_weight = fuselage_mass.loads
        self.x_points = fuselage_mass.x_points
        self.dx = np.gradient(self.x_points)
        self.h_tail_pos = self.x_points[-1]
        self.a_dim = 0.5 / np.tan(np.radians(65))
        self.s_dim = 0.5 / np.sin(np.radians(65))
        self.o_dim = self.s_dim/2 * np.sin(np.radians(25))



    def calculate_fuselage_centroid(self):


        #Unit is meters
        self.z_bar_fuselage = (2*(self.fuselage_ratio*((self.a_dim + 0.4)*self.fuselage_width)) + 2*(self.s_dim * (self.o_dim * self.fuselage_width)) + ((self.a_dim + self.fuselage_ratio) * self.fuselage_width)) / (2*self.s_dim + 2*self.fuselage_ratio + 1)
        
        self.x_bar_fuselage = (self.s_dim * 0.25 * self.fuselage_width + (self.s_dim * 0.75 * self.fuselage_width) + (0.5*self.fuselage_width) + (self.fuselage_ratio*self.fuselage_width)) / (2*self.s_dim + 2*self.fuselage_ratio + 1)

        return self.x_bar_fuselage, self.z_bar_fuselage
        
    
    def calculate_fuselage_moi(self):
        # Thin walled assumption used for fuselage
        # All moments of inertia are calculated about the centroid of the fuselage
        # All moments of inertia still have to be multiplied by the thickness of the skin
        # Unit is m^4 --> means that t_skin must also be in METERS!

        self.I_xy_fuselage = 0 # * t_fuselage

        self.I_yy_fuselage = self.fuselage_width*(((self.fuselage_ratio + self.a_dim)*self.fuselage_width - self.z_bar_fuselage)**2) + 2*(((self.fuselage_ratio*self.fuselage_width)**3)/12 + self.fuselage_ratio*self.fuselage_width*((self.a_dim + self.fuselage_ratio/2)*self.fuselage_width - self.z_bar_fuselage)**2) + 2*(((self.s_dim**3 * (np.sin(np.radians(25)))**2)/12) + self.s_dim*(self.o_dim -self.z_bar_fuselage)**2) # * t_fuselage

        self.I_zz_fuselage = (self.fuselage_width**3) / 12 + 2*(self.fuselage_ratio*self.fuselage_width*(self.fuselage_width/2)**2) + 2*(self.s_dim*(0.25*self.fuselage_width)**2) # * self.t_fuselage

        return self.I_yy_fuselage, self.I_zz_fuselage, self.I_xy_fuselage
        
    
    # def calculate_stresses(self):

    #     self.sigma_1 = (((M_x * self.I_zz_fuselage)*((self.a_dim+(self.fuselage_ratio*self.fuselage_width)-self.z_bar_fuselage))) + (M_y*self.I_yy_fuselage)*(-0.5*self.fuselage_width)) / (self.I_yy_fuselage*self.I_zz_fuselage)
    #     self.sigma_2 = (((M_x * self.I_zz_fuselage)*((self.a_dim+(self.fuselage_ratio*self.fuselage_width)-self.z_bar_fuselage))) + (M_y*self.I_yy_fuselage)*(0.5*self.fuselage_width)) / (self.I_yy_fuselage*self.I_zz_fuselage)

    #     self.sigma_3 = ((M_x * self.I_zz_fuselage) * -(self.z_bar_fuselage-self.a_dim) + (M_y *self.I_yy_fuselage)*(-0.5*self.fuselage_width)) / (self.I_yy_fuselage*self.I_zz_fuselage)
    #     self.sigma_4 = ((M_x * self.I_zz_fuselage) * -(self.z_bar_fuselage-self.a_dim) + (M_y *self.I_yy_fuselage)*(0.5*self.fuselage_width)) / (self.I_yy_fuselage*self.I_zz_fuselage)

    #     self.sigma_5 = ((M_x * self.I_zz_fuselage)*(-self.z_bar_fuselage)) / (self.I_yy_fuselage*self.I_zz_fuselage)
    #     return self.sigma_1, self.sigma_2, self.sigma_3, self.sigma_4, self.sigma_5

    def calculate_boom_areas(self):
       
        # All boom areas are a function of the skin thickness
        w = self.fuselage_width
        r = self.fuselage_ratio
        s = self.s_dim

        B1 = (w / 6) * (2 + self.sigma_2 / self.sigma_1) + (r * w / 6) * (2 + self.sigma_3 / self.sigma_1) # * t_fuselage
        B2 = (w / 6) * (2 + self.sigma_1 / self.sigma_2) + (r * w / 6) * (2 + self.sigma_4 / self.sigma_2) # * t_fuselage
        B3 = (r * w / 6) * (2 + self.sigma_1 / self.sigma_3) + (s * w / 6) * (2 + self.sigma_5 / self.sigma_3) # * t_fuselage
        B4 = (r * w / 6) * (2 + self.sigma_2 / self.sigma_4) + (s * w / 6) * (2 + self.sigma_5 / self.sigma_4) # * t_fuselage
        B5 = (s * w / 6) * (2 + self.sigma_3 / self.sigma_5) + (s * w / 6) * (2 + self.sigma_4 / self.sigma_5) # * t_fuselage

        return B1, B2, B3, B4, B5


    def iterate_booms_per_station(
        self, 
        M_x, 
        M_y, 
        t_fuselage_init=0.001, 
        I_yy_init=0.01, 
        I_zz_init=0.01, 
        tol=1e-5, 
        max_iter=1000, 
        alpha=0.5
    ):
        n_stations = len(self.fuselage_width)

        self.boom_areas_all = np.zeros((n_stations, 5))
        self.I_yy_all = np.zeros(n_stations)
        self.I_zz_all = np.zeros(n_stations)
        self.t_fuselage_final = np.zeros(n_stations)
        iterations_to_converge = np.zeros(n_stations, dtype=int)

        for i in range(n_stations):
            w = self.fuselage_width[i]
            h = self.fuselage_height[i]
            fr = h / w

            # Initial fuselage thickness 
            t = t_fuselage_init if np.isscalar(t_fuselage_init) else t_fuselage_init[i]

            # Initial moments of inertia
            I_yy_old = I_yy_init if np.isscalar(I_yy_init) else I_yy_init[i]
            I_zz_old = I_zz_init if np.isscalar(I_zz_init) else I_zz_init[i]

            
            a_dim = 0.5 / np.tan(np.radians(65))
            s_dim = 0.5 / np.sin(np.radians(65))
            o_dim = s_dim / 2 * np.sin(np.radians(25))

            z_bar = (2 * (fr * ((a_dim + 0.4) * w)) + 2 * (s_dim * (o_dim * w)) + ((a_dim + fr) * w)) / (2 * s_dim + 2 * fr + 1)

            self.z_coords = np.array([
                (z_bar - a_dim),
                (z_bar - a_dim),
                (a_dim + fr * w - z_bar),
                (a_dim + fr * w - z_bar),
                -z_bar
            ])
            self.y_coords = np.array([
                0.5 * w,
                -0.5 * w,
                0.5 * w,
                -0.5 * w,
                0.0
            ])

            # Initial boom areas
            B_old = np.ones(5) * 0.01

            for iteration in range(max_iter):
                # Stress calculation
                sigma = np.zeros(5)
                for j in range(5):
                    sigma[j] = (M_x * I_zz_old * self.z_coords[j] + M_y * I_yy_old * self.y_coords[j]) / (I_yy_old * I_zz_old)

                # New boom areas
                B1 = (w / 6) * (2 + sigma[1] / sigma[0]) + (fr * w / 6) * (2 + sigma[2] / sigma[0])
                B2 = (w / 6) * (2 + sigma[0] / sigma[1]) + (fr * w / 6) * (2 + sigma[3] / sigma[1])
                B3 = (fr * w / 6) * (2 + sigma[0] / sigma[2]) + (s_dim * w / 6) * (2 + sigma[4] / sigma[2])
                B4 = (fr * w / 6) * (2 + sigma[1] / sigma[3]) + (s_dim * w / 6) * (2 + sigma[4] / sigma[3])
                B5 = (s_dim * w / 6) * (2 + sigma[2] / sigma[4]) + (s_dim * w / 6) * (2 + sigma[3] / sigma[4])
                B_new = np.array([B1, B2, B3, B4, B5]) * t

                # Under-relaxation: boom areas
                B_i = alpha * B_new + (1 - alpha) * B_old

                # New moments of inertia based on updated boom areas
                I_yy_new = np.sum((self.z_coords ** 2) * B_i)
                I_zz_new = np.sum((self.y_coords ** 2) * B_i)

                # Under-relaxation: moments of inertia
                I_yy_relaxed = alpha * I_yy_new + (1 - alpha) * I_yy_old
                I_zz_relaxed = alpha * I_zz_new + (1 - alpha) * I_zz_old

                # Thickness update
                sigma_max = np.max(np.abs(sigma))
                t_new = t * (sigma_max / self.sigma_y)
                t = alpha * t_new + (1 - alpha) * t

                # Check for convergence
                if (
                    np.all(np.abs(B_i - B_old) < tol) and
                    abs(I_yy_relaxed - I_yy_old) < tol and
                    abs(I_zz_relaxed - I_zz_old) < tol and
                    abs(t_new - t) < tol
                ):
                    iterations_to_converge[i] = iteration + 1
                    print(f"Station {i}: Converged in {iteration + 1} iteration(s)")
                    break

                # Update for next iteration
                B_old = B_i.copy()
                I_yy_old = I_yy_relaxed
                I_zz_old = I_zz_relaxed

            else:
                iterations_to_converge[i] = max_iter
                print(f"Warning: Station {i} did not converge after {max_iter} iterations.")

            # Save final results
            self.boom_areas_all[i, :] = B_i
            self.I_yy_all[i] = I_yy_relaxed
            self.I_zz_all[i] = I_zz_relaxed
            self.t_fuselage_final[i] = t

            self.boom_areas_all[i, :] *= t 
            self.I_yy_all[i] *= t 
            self.I_zz_all[i] *= t
        return self.boom_areas_all
    
    def calculate_base_shearflows(self, B, z, y, idx):
        qb = -self.V_z/self.I_yy_all[idx]*B*y - self.V_y/self.I_zz_all[idx]*B*z
        return qb

    def calculate_shear_flow_distribution(self):
        
        # Both shear forces are assumed to act at the centroid of 
        self.V_y = -5e6
        self.V_z = 10e6

        self.A_m = (self.fuselage_width**2 * self.fuselage_ratio) + 2*(0.25*self.fuselage_width*self.a_dim)
        distances_array = np.array([(0.5*self.fuselage_ratio*(self.fuselage_width**2)), (0.5*self.fuselage_ratio*(self.fuselage_width**2)), self.z_bar_fuselage]) / (2*self.A_m)
        self.base_shear_flows = []
        self.tot_shear_flow = []
        self.shear_flow_dicts = []
        for i, station in enumerate(self.boom_areas_all):
            B1, B2, B3, B4, B5 = station
            delta_B1 = self.calculate_base_shearflows(B1, self.z_coords[0], self.y_coords[0], i)
            delta_B3 = self.calculate_base_shearflows(B3, self.z_coords[2], self.y_coords[2], i)
            delta_B5 = self.calculate_base_shearflows(B5, self.z_coords[4], self.y_coords[4], i)
            delta_B4 = self.calculate_base_shearflows(B4, self.z_coords[3], self.y_coords[3], i)

            q21 = 0
            q13 = delta_B1
            q35 = q13 + delta_B3
            q54 = q35 + delta_B5
            q42 = q54 + delta_B4

            base_shear_flows = [q21, q13, q35, q54, q42]

            print(q42, q13)
            red_base_q_array = [q42, q13, self.V_y]
            q_s0 = np.dot(red_base_q_array, distances_array[:, i])
            print(q_s0)
            total_shear_flows = [q + q_s0 for q in base_shear_flows]
            self.tot_shear_flow.append(total_shear_flows)

            # Store as dictionary
            shear_flow_dict = {
            "q21": total_shear_flows[0],
            "q13": total_shear_flows[1],
            "q35": total_shear_flows[2],
            "q54": total_shear_flows[3],
            "q42": total_shear_flows[4]
            }
            self.shear_flow_dicts.append(shear_flow_dict)

        print(self.shear_flow_dicts)



    def main(self):
        M_x = 20e6  # Nm # TODO: Replace with actual bending moment
        M_y = 0  # Nm # TODO: Replace with actual moment
        self.calculate_fuselage_centroid()
        self.iterate_booms_per_station(M_x, M_y)

        self.calculate_shear_flow_distribution()


if __name__ == '__main__':
    aircraft_data = Data("design3.json")
    airfoil_data = Data("AirfoilData/example_airfoil.json")
    fuselage_material = Materials.Al7075  
    
    fuselage = FuselageThickness(aircraft_data, fuselage_material)

    fuselage.main()

    # print("Boom Areas per Station(must still be multiplied by t_fuselage in m):\n", boom_areas)
    # print("I_yy per Station(must still be multiplied by t_fuselage in m):\n", I_yy_array)
    # print("I_zz per Station(must still be multiplied by t_fuselagein m):\n", I_zz_array)
    # print("Fuselage Thickness per Station:\n", t_fuselage*(1000))  #mm
    # print(f"Kt for cargo door: {fuselage.calculate_stress_concentration_factor()}")




    # def get_lift_on_fuselage(self):
        
    #     self.lift_function = self.aeroforces.get_lift_function()
    #     self.horizontal_tail_lift = self.aeroforces.get_horizontal_tail_lift_distribution()
    #     self.lift_function = self.lift_function(self.b_array)
    #     self.lift_on_fuselage = 2*np.trapz(self.lift_function, self.b_array)

    #     self.h_lift_on_fuselage = 2*np.trapz(self.horizontal_tail_lift, self.b_h_array)
    #     print(self.h_lift_on_fuselage, self.lift_on_fuselage)

    # def internal_vertical_shear_fuselage(self):
        
    #     load = self.distributed_weight
    #     Vy_fus = np.cumsum(load * self.dx)
    #     self.Vy_fus_internal = -Vy_fus 

    #     main_idx = np.where(self.x_points >= self.lift_acting_point)[0][0]
    #     self.Vy_fus_internal[main_idx:] += self.lift_on_fuselage

    #     tail_idx = np.where(self.x_points >= self.h_tail_pos)[0][0]
    #     self.Vy_fus_internal[tail_idx:] += self.h_lift_on_fuselage

    #     plt.plot(self.x_points, self.Vy_fus_internal, label='Internal Vertical Shear Force on Fuselage')
    #     plt.xlabel('Position along Fuselage (m)')
    #     plt.ylabel('Internal Vertical Shear Force (N)')
    #     plt.title('Internal Vertical Shear Force Distribution on Fuselage')
    #     plt.legend()
    #     plt.grid()
    #     plt.show()
    #     return self.Vy_fus_internal
        
    # def internal_bending_moment_fuselage(self):
    #     load = self.Vy_fus_internal
    #     M_fus = np.cumsum(load * self.dx)
    #     self.M_fus_internal = M_fus

    #     plt.plot(self.x_points, self.M_fus_internal, label='Internal Bending Moment on Fuselage')
    #     plt.xlabel('Position along Fuselage (m)')
    #     plt.ylabel('Internal Bending Moment (Nm)')
    #     plt.title('Internal Bending Moment Distribution on Fuselage')
    #     plt.legend()
    #     plt.grid()
    #     plt.show()

    #     return self.M_fus_internal