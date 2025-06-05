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

        self.C = self.aircraft_data.data['inputs']['structures']['fuselage']['C_fus']

        self.shear_yield = self.sigma_y * 0.577

        self.rho_fuselage = self.material['rho']

        self.fuselage_width = np.array([self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'], self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage'], self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']])
        self.fuselage_height = np.array([self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station1'], self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station2'], self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station3']])
        self.fuselage_ratio =  self.fuselage_height / self.fuselage_width
        self.l_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        self.wing_LE_pos = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.lift_acting_point = self.wing_LE_pos + 1/4* self.chord_root

        self.t_fuselage = self.aircraft_data.data['inputs']['structures']['fuselage']['t_fuselage']/1000

        self.b_array = np.arange(0, self.b/2 + 0.01, 0.01)

        aerodynamics = Data("AeroForces.txt", "aerodynamics")
        self.aeroforces = AerodynamicForces(self.aircraft_data, aerodynamics)
        self.fuselage_mass = CGCalculation(self.aircraft_data)
        self.fuselage_mass.load_diagram()
        self.distributed_weight = self.fuselage_mass.loads
        self.x_points = self.fuselage_mass.x_points

        self.V_internal = self.fuselage_mass.shear

        self.M_internal = self.fuselage_mass.moment

        self.dx = np.gradient(self.x_points)
        self.h_tail_pos = self.x_points[-1]
        self.a_dim = 0.5 / np.tan(np.radians(65))*self.fuselage_width
        self.s_dim = 0.5 / np.sin(np.radians(65))*self.fuselage_width
        self.o_dim = self.s_dim/2 * np.sin(np.radians(25))

        self.station1_threshold = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose']
        self.station2_threshold = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        self.station3_threshold = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody'] + self.aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']

        self.thresholds = [
            self.station1_threshold,
            self.station2_threshold,
            self.station3_threshold]  
        
    def get_boom_areas(self):    
        self.B1 = self.aircraft_data.data['inputs']['structures']['fuselage']['B1']/1e6
        self.B2 = self.aircraft_data.data['inputs']['structures']['fuselage']['B2']/1e6
        self.B3 = self.aircraft_data.data['inputs']['structures']['fuselage']['B3']/1e6
        self.B4 = self.aircraft_data.data['inputs']['structures']['fuselage']['B4']/1e6
        self.B5 = self.aircraft_data.data['inputs']['structures']['fuselage']['B5']/1e6


    def calculate_fuselage_centroid(self):
        self.boom_coords = {
            "B1": {'coords': (0.5*self.fuselage_width, self.a_dim + self.fuselage_ratio * self.fuselage_width), 'area': self.B1},
            "B2": {'coords': (-0.5*self.fuselage_width, self.a_dim + self.fuselage_ratio * self.fuselage_width), 'area': self.B2},
            "B3": {'coords': (0.5*self.fuselage_width, self.a_dim), 'area': self.B3},
            "B4": {'coords': (-0.5*self.fuselage_width, self.a_dim), 'area': self.B4},
            "B5": {'coords': (0.0, 0.0), 'area': self.B5}
        }

        area_distance = sum([i['coords'][1]* i['area'] for i in self.boom_coords.values()])
        area = sum([i['area'] for i in self.boom_coords.values()])
        self.z_bar = area_distance / area
        self.y_bar = 0.0 

        self.z_coords = [
            self.boom_coords['B1']['coords'][1] - self.z_bar,
            self.boom_coords['B2']['coords'][1] - self.z_bar,
            self.boom_coords['B3']['coords'][1] - self.z_bar,
            self.boom_coords['B4']['coords'][1] - self.z_bar,
            self.boom_coords['B5']['coords'][1] - self.z_bar
        ]

        self.y_coords = [
            self.boom_coords['B1']['coords'][0] - self.y_bar,
            self.boom_coords['B2']['coords'][0] - self.y_bar,
            self.boom_coords['B3']['coords'][0] - self.y_bar,
            self.boom_coords['B4']['coords'][0] - self.y_bar,
            self.boom_coords['B5']['coords'][0] - self.y_bar
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

            dq1, dq2, dq3, dq4, dq5 = column

            q21 = 0
            q13 = dq1
            q35 = q13 + dq3
            q54 = q35 + dq5
            q42 = q54 + dq4

            self.base_shear_flows[f'Station_{i}'] = {
                "q21": q21,
                "q13": q13,
                "q35": q35,
                "q54": q54,
                "q42": q42
            }

        return self.base_shear_flows
    

    def calculate_shear_flow_distribution(self, V_z=10e6, V_y=0):

        self.A_m = (self.fuselage_width**2 * self.fuselage_ratio) + 2*(0.25*self.fuselage_width*self.a_dim)
        distances_array = np.array([(0.5*self.fuselage_ratio*(self.fuselage_width**2)), (0.5*self.fuselage_ratio*(self.fuselage_width**2)), self.z_bar]) / (2*self.A_m)
        self.base_shear_flows = []
        self.tot_shear_flow = []
        self.shear_flow_dicts = []

        self.calculate_base_shearflows()

        for i in range(len(self.thresholds)):
            q21 = self.base_shear_flows[f'Station_{i}']['q21']
            q13 = self.base_shear_flows[f'Station_{i}']['q13']
            q35 = self.base_shear_flows[f'Station_{i}']['q35']
            q54 = self.base_shear_flows[f'Station_{i}']['q54']
            q42 = self.base_shear_flows[f'Station_{i}']['q42']


            base_shear_flows = [q21, q13, q35, q54, q42]
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
        "q35": total_shear_flows[2],
        "q54": total_shear_flows[3],
        "q42": total_shear_flows[4]
        }
        self.shear_flow_dicts.append(shear_flow_dict)

    def calculate_shear_stress(self):
        self.shear_stresses = {
            'Section_12': [],
            'Section_13': [],
            'Section_35': [],
            'Section_45': [],
            'Section_24': []
        }

        for i in range(len(self.x_points)):
            V_z = self.V_internal[i]
            V_y = 0
            self.calculate_shear_flow_distribution(V_z, V_y)
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - self.x_points[i]))

            for idx,q_dist in enumerate(self.shear_flow_dicts):
                for i,q in enumerate(list(q_dist.values())):
                    section_key = f'Section_{self.section_order[i]}'
                    if q != 0:
                        shear_stress = abs(q / (self.final_thicknesses[idx][station_idx]))
                    else:
                        shear_stress = 0.0
                    self.shear_stresses[section_key].append(shear_stress)
                    
        self.shear_stresses = {k: np.array(v) for k, v in self.shear_stresses.items()}
        evaluate_stress = 'Section_13'
        plt.plot(self.x_points, self.shear_stresses[evaluate_stress]/1e6, label='Bending Stress Distribution')
        plt.plot(self.x_points, np.full_like(self.x_points, np.sign(self.shear_stresses[evaluate_stress])*self.shear_yield/1e6), 'k--', label='Zero Stress Line')
        plt.xlabel('Position along Fuselage (m)')
        plt.ylabel('Bending Stress (MPa)')
        plt.title('Bending Stress Distribution on Fuselage')
        plt.legend()
        plt.grid()
        plt.show()
        

    def calculate_top_bending_stress(self):
        self.bending_stresses = {}
        loop_flag = 0
        for boom in self.boom_coords:
            self.bending_stresses[boom] = []
            count = 0
            prev_idx = 0

            for idx, iyy in enumerate(self.I_yy_all):
                count += 1

                curr_idx = np.argmin(np.abs(self.x_points - self.thresholds[idx]))
                self.bending_stresses[boom].append(self.M_internal[prev_idx:curr_idx]*self.z_coords[loop_flag][idx]/iyy)
                if count == 3:
                    self.bending_stresses[boom].append(self.M_internal[curr_idx:]*self.z_coords[loop_flag][idx]/iyy)
                prev_idx = curr_idx

            loop_flag += 1
        self.bending_stresses = {k: np.concatenate(v, axis=0) for k, v in self.bending_stresses.items()}

        evaluate_stress = 'B5'
        plt.plot(self.x_points, self.bending_stresses[evaluate_stress]/1e6, label='Bending Stress Distribution')
        plt.plot(self.x_points, np.full_like(self.x_points, np.sign(self.bending_stresses[evaluate_stress])*self.sigma_y/1e6), 'k--', label='Zero Stress Line')
        plt.xlabel('Position along Fuselage (m)')
        plt.ylabel('Bending Stress (MPa)')
        plt.title('Bending Stress Distribution on Fuselage')
        plt.legend()
        plt.grid()
        plt.show()
        return self.bending_stresses
    
    def calculate_boom_thicknesses(self):

        boom_connections = [('B1', 'B2', 'B3'), ('B2', 'B1', 'B3'), ('B3', 'B1', 'B5'), ('B4', 'B2', 'B5'), ('B5', 'B3', 'B4')]

        boom_sections = {
            'B1': [self.fuselage_width, self.fuselage_ratio * self.fuselage_width],
            'B2': [self.fuselage_width, self.fuselage_ratio * self.fuselage_width],
            'B3': [self.fuselage_ratio*self.fuselage_width, self.s_dim],
            'B4': [self.fuselage_ratio*self.fuselage_width, self.s_dim],
            'B5': [self.s_dim, self.s_dim]
        }

        self.final_thicknesses = {}
        for connection in boom_connections:
            main_boom = connection[0]
            boom_area = self.boom_coords[main_boom]['area']
            ratio_sum = 0
            for boom in range(len(connection[1:])):
                boom_name = connection[boom]
                section = boom_sections[main_boom][boom]
                
                stress_ratio = min(self.bending_stresses[boom_name][np.flatnonzero(self.bending_stresses[boom_name])]/self.bending_stresses[main_boom][np.flatnonzero(self.bending_stresses[main_boom])])

                contribution = section/6*(2+stress_ratio)
                ratio_sum += contribution
            
            thickness = boom_area / ratio_sum
            self.final_thicknesses[main_boom] = {connection[1]: thickness, connection[2]: thickness}

        self.section_order = ['12', '13', '35', '45', '24']
        self.section_map = {'12': ('B1', 'B2'), '13': ('B1', 'B3'), '35': ('B3', 'B5'),
                    '45': ('B4', 'B5'), '24': ('B2', 'B4')}

        # Output list
        final_thickness_list = []

        for sec in self.section_order:
            boom1, boom2 = self.section_map[sec]
            
            # Get thickness from both directions if available
            t1 = self.final_thicknesses.get(boom1, {}).get(boom2, None)
            t2 = self.final_thicknesses.get(boom2, {}).get(boom1, None)
            
            if t1 is not None and t2 is not None:
                thickness = np.maximum(t1, t2)
            elif t1 is not None:
                thickness = t1
            elif t2 is not None:
                thickness = t2
            else:
                raise ValueError(f"Section {sec} ({boom1}-{boom2}) not found in either direction.")
            
            final_thickness_list.append(thickness)

        self.final_thicknesses = final_thickness_list
        print("Final Thicknesses per Boom Connection:")
        print(self.final_thicknesses)

    def calculate_fuselage_weight(self):

        for x in self.x_points:
            for section in self.final_thicknesses:
                thickness = self.final_thicknesses[section]

                


    def calculate_rib_spacing_skin(self):
        factor = (np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))
        
        self.rib_spacings = []
        self.rib_positions = []
        self.rib_span_indices = []

        self.calculate_top_bending_stress()
        x = 0.0  
        self.rib_positions.append(x)
        
        for idx, x in enumerate(self.x_points):
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - x))
            
            stress = self.bending_stresses[idx]
            if abs(stress) < 1e6:
                b = 2.0
            else:
                b = self.final_thicknesses[station_idx]['54'] * np.sqrt(self.C * factor / stress)
            self.rib_spacings.append(b)
            x += b
            self.rib_positions.append(x)

        
        self.rib_amount = len(self.rib_positions)


    def get_shear_buckling_thickness(self, tau_cr):
        b = self.cutout_spacing 
        factor = (12 * (1 - self.poisson_ratio**2)) / (self.k_s * np.pi**2 * self.E)

        t_required = b * np.sqrt(tau_cr * factor)
        return t_required


    def main(self):
        M_x = 20e6  # Nm # TODO: Replace with actual bending moment
        M_y = 0  # Nm # TODO: Replace with actual moment
        self.calculate_fuselage_centroid()

        self.calculate_shear_flow_distribution()
        self.calculate_top_bending_stress()
        self.calculate_boom_thicknesses()
        self.calculate_shear_stress()
        # self.calculate_rib_spacing_skin()


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