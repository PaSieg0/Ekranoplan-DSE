import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from scipy.integrate import quad
from Class_II.weight_distributions import load_diagram
from Class_II.AerodynamicForces import AerodynamicForces

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials

class FuselageThickness:
    def __init__(self, aircraft_data: Data, fuselage_mat: Materials = Materials.Al7075, frame_mat: Materials = Materials.Al7075, plot=False):
          
        self.aircraft_data = aircraft_data

        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.front_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['rear_spar']

        self.t_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['t_spar']/1000
        self.t_skin = self.aircraft_data.data['inputs']['structures']['wing_box']['t_skin']/1000
        self.t_wing = self.aircraft_data.data['inputs']['structures']['wing_box']['t_wing']/1000

        self.frame_material = self.aircraft_data.data['inputs']['structures']['materials'][frame_mat.name.lower()]
        self.epoxy_in = self.aircraft_data.data['inputs']['structures']['fuselage']['epoxy_in']
        self.epoxy_out = self.aircraft_data.data['inputs']['structures']['fuselage']['epoxy_out']
        self.rho_epoxy = self.aircraft_data.data['inputs']['structures']['materials']['Epoxy']['rho']

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.b_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.b_array = np.arange(0, self.b/2, 0.01)
        self.b_h_array = np.arange(0, self.b_h/2 + 0.01, 0.01)
        # self.chord_array = self.chord_span_function(self.b_array)

        self.chord_length = self.chord_root
        self.plot = plot

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']
        self.fuel_tank = self.aircraft_data.data['inputs']['fuel_tank']

        self.material = self.aircraft_data.data['inputs']['structures']['materials'][fuselage_mat.name.lower()]
        self.G = self.material['G']
        self.E = self.material['E']
        self.sigma_y = self.material['sigma_y']
        self.poisson_ratio = self.material['poisson_ratio']
        self.density = self.material['rho']

        self.C = self.aircraft_data.data['inputs']['structures']['fuselage']['C_fus']
        self.k_s = self.aircraft_data.data['inputs']['structures']['fuselage']['k_s']

        self.shear_yield = self.sigma_y * 0.577

        self.rho_fuselage = self.material['rho']

        self.rib_density = self.frame_material['rho']

        self.fuselage_width = np.array([self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'], self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'], self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']])
        self.fuselage_height = np.array([self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station1'], self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station2'], self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station3']])
        self.fuselage_ratio =  self.fuselage_height / self.fuselage_width
        self.l_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        self.wing_LE_pos = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.lift_acting_point = self.wing_LE_pos + 1/4* self.chord_root

        self.t_fuselage = self.aircraft_data.data['inputs']['structures']['fuselage']['t_fuselage']/1000

        self.b_array = np.arange(0, self.b/2 + 0.01, 0.01)

        self.fuselage_mass = load_diagram(self.aircraft_data, plot=False)
        self.fuselage_mass.get_weights()
        self.fuselage_mass.get_load_distribution()
        self.fuselage_mass.get_internal_loads()
        self.x_points = self.fuselage_mass.x_points

        self.dx = np.gradient(self.x_points)
        self.h_tail_pos = self.x_points[-1]
        self.hull_angle = self.aircraft_data.data['outputs']['fuselage_dimensions']['hull_angle']
        self.a_dim = 0.5 / np.tan(np.radians(90-self.hull_angle))*self.fuselage_width
        self.s_dim = 0.5 / np.sin(np.radians(90-self.hull_angle))*self.fuselage_width
        self.o_dim = self.s_dim/2 * np.sin(np.radians(self.hull_angle))

        self.station1_threshold = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose']
        self.station2_threshold = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        self.station3_threshold = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']

        self.safety_factor = 1.5
        self.tolerance = 1e-6
        self.max_iterations = 100
        self.iteration = 0
        self.thresholds = [
            self.station1_threshold,
            self.station2_threshold,
            self.station3_threshold]  
        
        self.get_boom_areas()

        self.section_order = ['12', '13', '34', '42']
        self.section_map = {'12': ('B1', 'B2'), '13': ('B1', 'B3'), '34': ('B3', 'B4'),
                    '42': ('B4', 'B2')}

        self.section_lengths = [self.fuselage_width, self.fuselage_ratio * self.fuselage_width, self.s_dim, self.s_dim, self.fuselage_ratio * self.fuselage_width]
        
    def get_boom_areas(self):    
        self.B1 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B1']/1e6 for i in range(1,4)])
        self.B2 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B2']/1e6 for i in range(1,4)])
        self.B3 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B3']/1e6 for i in range(1,4)])
        self.B4 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B4']/1e6 for i in range(1,4)])

        self.stringer_areas = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['stringer_area']/1e6 for i in range(1,4)])

        self.stringer_radius = np.sqrt(self.stringer_areas * 4 / np.pi)/2

        self.n_stringers = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['n_stringers'] for i in range(1,4)])

        self.boom_map = {
            'B1': self.B1,
            'B2': self.B2,
            'B3': self.B3,
            'B4': self.B4,
        }

    def calculate_fuselage_centroid(self):
        self.boom_coords = {
            "B1": {'coords': (0.5*self.fuselage_width, 0.5* self.fuselage_width), 'area': self.B1},
            "B2": {'coords': (-0.5*self.fuselage_width, 0.5*self.fuselage_width), 'area': self.B2},
            "B3": {'coords': (0.5*self.fuselage_width, np.array([0,0,0])), 'area': self.B3},
            "B4": {'coords': (-0.5*self.fuselage_width,  np.array([0,0,0])), 'area': self.B4},     }
        
        print(self.boom_coords)
        
        area_distance = np.sum([i['coords'][1] * i['area'] for i in self.boom_coords.values()], axis=0)
        area = np.sum([i['area'] for i in self.boom_coords.values()], axis=0)
        self.z_bar = area_distance / area
        self.y_bar = 0.0 

        self.z_coords = [
            self.boom_coords['B1']['coords'][1] - self.z_bar,
            self.boom_coords['B2']['coords'][1] - self.z_bar,
            self.boom_coords['B3']['coords'][1] - self.z_bar,
            self.boom_coords['B4']['coords'][1] - self.z_bar,
        ]

        self.y_coords = [
            self.boom_coords['B1']['coords'][0] - self.y_bar,
            self.boom_coords['B2']['coords'][0] - self.y_bar,
            self.boom_coords['B3']['coords'][0] - self.y_bar,
            self.boom_coords['B4']['coords'][0] - self.y_bar,
        ]


        self.calculate_MOI()

    def calculate_MOI(self):
        
        self.I_yy_all = []
        self.I_zz_all = []
        count = 0
        for i in self.boom_coords:
            area = self.boom_coords[i]['area']
            z_coord = self.z_coords[count]
            y_coord = self.y_coords[count]
            I_yy = area * z_coord**2
            I_zz = area * y_coord**2
            self.I_yy_all.append(I_yy)
            if isinstance(I_zz, float):
                I_zz = np.zeros_like(self.z_coords[count])
            self.I_zz_all.append(I_zz)
            count += 1

        self.I_yy_all = np.array(self.I_yy_all)
        self.I_zz_all = np.array(self.I_zz_all)
        self.I_yy_all = np.array([sum(self.I_yy_all[:,i]) for i in range(len(self.thresholds))])
        self.I_zz_all = np.array([sum(self.I_zz_all[:,i]) for i in range(len(self.thresholds))])

    def calculate_base_shearflows(self, V_z=10e6, V_y=0):
        count = 0
        self.delta_qs = []
        for i in self.boom_coords:
            B = self.boom_coords[i]['area']

            qb = -V_z/self.I_yy_all*B*self.z_coords[count] - V_y/self.I_zz_all*B*self.y_coords[count]
            self.delta_qs.append(qb)
            count += 1

        self.delta_qs = np.array(self.delta_qs)
        self.base_shear_flows = {}
        for i in range(self.delta_qs.shape[1]):
            column = self.delta_qs[:, i]

            dq1, dq2, dq3, dq4 = column

            q21 = 0
            q13 = round(dq1,4)
            q34 = round(q13 + dq3,4)
            q42 = round(q34 + dq4,4)

            self.base_shear_flows[f'Station_{i}'] = {
                "q21": q21,
                "q13": q13,
                "q34": q34,
                "q42": q42
            }

        return self.base_shear_flows
    

    def calculate_shear_flow_distribution(self, V_z=10e6, V_y=0):

        self.A_m = (self.fuselage_width**2 * self.fuselage_ratio) + 2*(0.25*self.fuselage_width*self.a_dim)
        distances_array = np.array([(0.5*self.fuselage_ratio*(self.fuselage_width**2)), (0.5*self.fuselage_ratio*(self.fuselage_width**2)), self.z_bar]) / (2*self.A_m)
        self.base_shear_flows = []
        self.tot_shear_flow = []
        self.shear_flow_dicts = []

        self.calculate_base_shearflows(V_y=V_y, V_z=V_z)

        for i in range(2,len(self.thresholds),1):
            q21 = self.base_shear_flows[f'Station_{i}']['q21']
            q13 = self.base_shear_flows[f'Station_{i}']['q13']
            q34 = self.base_shear_flows[f'Station_{i}']['q34']
            q42 = self.base_shear_flows[f'Station_{i}']['q42']

            print(f'station_{i}')
            base_shear_flows = [q21, q13, q34, q42]
            red_base_q_array = [q42, q13, V_y]

            q_s0 = np.dot(red_base_q_array, distances_array[:, i])
            if 0 < abs(q_s0) < 1e-8:
                q_s0 = 0.0


            total_shear_flows = [q + q_s0 for q in base_shear_flows]
            self.tot_shear_flow.append(total_shear_flows)

        # Store as dictionary
        shear_flow_dict = {
        "q21": total_shear_flows[0],
        "q13": total_shear_flows[1],
        "q34": total_shear_flows[2],
        "q42": total_shear_flows[3]
        }
        self.shear_flow_dicts.append(shear_flow_dict)

    def calculate_bending_stress(self):
        self.bending_stresses = {}
        loop_flag = 0
        M_y = 5e6
        M_y = M_y * np.ones_like(self.x_points)
        for boom in self.boom_coords:
            self.bending_stresses[boom] = []
            count = 0
            prev_idx = 0

            for idx, iyy in enumerate(self.I_yy_all):
                count += 1

                curr_idx = np.argmin(np.abs(self.x_points - self.thresholds[idx]))
                stress = M_y * self.z_coords[loop_flag][idx] / iyy
                print(stress)
                self.bending_stresses[boom].append(stress)
                prev_idx = curr_idx

            loop_flag += 1
        self.bending_stresses = {k: np.concatenate(v, axis=0) for k, v in self.bending_stresses.items()}

        return self.bending_stresses
    

    def calculate_boom_thicknesses(self):

        boom_connections = [('B1', 'B2', 'B3'), ('B2', 'B1', 'B4'), ('B3', 'B1', 'B4'), ('B4', 'B2', 'B3')]
        boom_sections = {
            'B1': [self.fuselage_width, 0.5 * self.fuselage_width],
            'B2': [self.fuselage_width, 0.5 * self.fuselage_width],
            'B3': [0.5*self.fuselage_width, self.fuselage_width],
            'B4': [0.5*self.fuselage_width, self.fuselage_width],
        }

        self.final_thicknesses = {}
        for connection in boom_connections:
            main_boom = connection[0]
            boom_area = self.boom_coords[main_boom]['area']
            ratio_sum = 0
            for i, boom_name in enumerate(connection[1:]):               
                section = boom_sections[main_boom][i]
                
                stress_ratio = min(
                     self.bending_stresses[main_boom][np.flatnonzero(self.bending_stresses[main_boom])]/
                    self.bending_stresses[boom_name][np.flatnonzero(self.bending_stresses[boom_name])]
                )


                contribution = section / 6 * (2 + stress_ratio)
                ratio_sum += contribution
            
            thickness = boom_area / ratio_sum

            self.final_thicknesses[main_boom] = {connection[1]: thickness, connection[2]: thickness}

        # Output list
        final_thickness_list = []

        final_thickness_list = []

        for sec in self.section_order:
            boom1, boom2 = self.section_map[sec]
            
            # Get thickness from both directions if available
            t1 = self.final_thicknesses.get(boom1, {}).get(boom2, None)
            t2 = self.final_thicknesses.get(boom2, {}).get(boom1, None)
            
            if t1 is not None and t2 is not None:
                thickness = t1 + t2  
            elif t1 is not None:
                thickness = t1
            elif t2 is not None:
                thickness = t2
            else:
                raise ValueError(f"Section {sec} ({boom1}-{boom2}) not found in either direction.")
            
            final_thickness_list.append(thickness)

        self.final_thicknesses = final_thickness_list

    def main(self):

        self.calculate_fuselage_centroid()

        self.calculate_shear_flow_distribution()
        print("Base Shear Flows Station 2 \n", self.shear_flow_dicts)

        self.calculate_bending_stress()
        self.calculate_boom_thicknesses()
        print(self.final_thicknesses)

        self.plot_station_cross_section(station_idx=1)

    def plot_station_cross_section(self, station_idx=0):

        station_name = f"Station_{station_idx + 1}"
        fig, ax = plt.subplots()

        boom_positions = {}
        boom_areas = {}

        # --- Process boom data ---
        for boom_name, boom_data in self.boom_coords.items():
            area = boom_data['area'][station_idx]
            if boom_name == 'B5':
                pos = (0, 0)
            else:
                y = boom_data['coords'][0][station_idx]  
                z = boom_data['coords'][1][station_idx]
                pos = (y, z)
            boom_positions[boom_name] = pos
            boom_areas[boom_name] = area

        centroid_y = self.y_bar  
        centroid_z = self.z_bar[station_idx]
        ax.scatter(centroid_y, centroid_z, s=120, c='green', edgecolor='black', zorder=15, label='Centroid')

        # --- Plot reference frame indicators ---
        # Origin at centroid
        arrow_length = 0.2 * max(self.fuselage_width)  

        ax.arrow(centroid_y, centroid_z, 0, arrow_length, head_width=0.03*arrow_length, head_length=0.05*arrow_length, fc='black', ec='black', linewidth=2, zorder=20)
        ax.text(centroid_y, centroid_z + arrow_length + 0.02*arrow_length, 'z', fontsize=10, ha='center', va='bottom')
        ax.arrow(centroid_y, centroid_z, arrow_length, 0, head_width=0.03*arrow_length, head_length=0.05*arrow_length, fc='black', ec='black', linewidth=2, zorder=20)
        ax.text(centroid_y + arrow_length + 0.05*arrow_length, centroid_z, 'y', fontsize=10, ha='right', va='center')

        # --- Plot the 4 main booms (B1-B4) ---
        boom_label_added = False
        # Define the order for plotting and legend consistency
        boom_keys_to_plot = ["B1", "B2", "B3", "B4"] 
        for boom_name_key in boom_keys_to_plot:
            if boom_name_key in boom_positions: # Booms B1-B4 should be in boom_positions
                pos = boom_positions[boom_name_key]
                label_val = None
                if not boom_label_added:
                    label_val = "Booms"
                    boom_label_added = True
                ax.scatter(pos[0], pos[1], s=80, c='blue', edgecolor='black', zorder=12, label=label_val)

        # --- Draw boom connections to form the fuselage section outline ---
        # This replaces the previous connection logic based on self.section_order and self.section_map
        square_connections = [('B1', 'B2'), ('B2', 'B4'), ('B4', 'B3'), ('B3', 'B1')]
        outline_label_added = False
        for (b1_name, b2_name) in square_connections:
            if b1_name in boom_positions and b2_name in boom_positions:
                pos1 = boom_positions[b1_name]
                pos2 = boom_positions[b2_name]
                label_val = None
                if not outline_label_added:
                    label_val = 'Fuselage Outline'
                    outline_label_added = True
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 'k-', linewidth=1.5, label=label_val)
            else:
                # This warning should ideally not appear if B1-B4 are correctly processed
                print(f"Warning: Plotting: Skipping connection {b1_name}-{b2_name} due to missing boom position for station {station_idx + 1}.")
        
        # Add labels and title
        ax.set_xlabel('y (m)')
        ax.set_ylabel('z (m)')
        # ax.set_title(f'Fuselage Cross-Section at {station_name} (x = {self.thresholds[station_idx]:.2f} m)')
        ax.set_aspect('equal')
        ax.invert_xaxis()  # Flip x-axis so left is positive
        # --- Legend and layout ---
        plt.tight_layout()
        plt.show()

        return fig, ax

        
if __name__ == '__main__':
    aircraft_data = Data("verification.json")
    fuselage_material = Materials.Al7075  
    rib_material = Materials.Al7075
    
    fuselage = FuselageThickness(aircraft_data, fuselage_material, frame_mat=rib_material, plot=True)

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