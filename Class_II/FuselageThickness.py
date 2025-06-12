import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from scipy.integrate import quad
from weight_distributions import load_diagram
from AerodynamicForces import AerodynamicForces

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials

class FuselageThickness:
    def __init__(self, aircraft_data: Data, fuselage_mat: Materials, frame_mat: Materials, plot=False):
          
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

        self.V_internal = self.fuselage_mass.shear

        self.M_internal = self.fuselage_mass.moment

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

        self.section_order = ['12', '13', '35', '45', '24']
        self.section_map = {'12': ('B1', 'B2'), '13': ('B1', 'B3'), '35': ('B3', 'B5'),
                    '45': ('B4', 'B5'), '24': ('B2', 'B4')}

        self.section_lengths = [self.fuselage_width, self.fuselage_ratio * self.fuselage_width, self.s_dim, self.s_dim, self.fuselage_ratio * self.fuselage_width]
        
    def get_boom_areas(self):    
        self.B1 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B1']/1e6 for i in range(1,4)])
        self.B2 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B2']/1e6 for i in range(1,4)])
        self.B3 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B3']/1e6 for i in range(1,4)])
        self.B4 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B4']/1e6 for i in range(1,4)])
        self.B5 = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['B5']/1e6 for i in range(1,4)])

        self.stringer_areas = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['stringer_area']/1e6 for i in range(1,4)])

        self.stringer_radius = np.sqrt(self.stringer_areas * 4 / np.pi)/2

        self.n_stringers = np.array([self.aircraft_data.data['inputs']['structures']['fuselage'][f'station_{i}']['n_stringers'] for i in range(1,4)])

        self.boom_map = {
            'B1': self.B1,
            'B2': self.B2,
            'B3': self.B3,
            'B4': self.B4,
            'B5': self.B5
        }

    def calculate_fuselage_centroid(self):
        self.boom_coords = {
            "B1": {'coords': (0.5*self.fuselage_width, self.a_dim + self.fuselage_ratio * self.fuselage_width), 'area': self.B1},
            "B2": {'coords': (-0.5*self.fuselage_width, self.a_dim + self.fuselage_ratio * self.fuselage_width), 'area': self.B2},
            "B3": {'coords': (0.5*self.fuselage_width, self.a_dim), 'area': self.B3},
            "B4": {'coords': (-0.5*self.fuselage_width, self.a_dim), 'area': self.B4},
            "B5": {'coords': (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])), 'area': self.B5}        }
        
        area_distance = np.sum([i['coords'][1] * i['area'] for i in self.boom_coords.values()], axis=0)
        area = np.sum([i['area'] for i in self.boom_coords.values()], axis=0)
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

        self.get_stringer_placement()

        self.calculate_MOI()

    def get_stringer_placement(self):

        n_sections = len(self.section_lengths)
        section_weights = [0.3, 0.2, 0.15,0.15, 0.2]
        total = sum(section_weights)

        self.stringer_dict = {}
        station_I_yy = []
        station_I_zz = []
        self.top_spacings = []
        self.side_spacings = []
        self.slope_spacings = []
        for station in range(len(self.thresholds)):
            station_idx = f'Station_{station+1}'
            self.stringer_dict[station_idx] = {'y': [], 'z': []}
            I_yy = 0
            I_zz = 0
            extra_stringers = [int(self.n_stringers[station] * w / total) for w in section_weights]

            for idx,section in enumerate(self.section_lengths):
                
                available_length = section[station] - 2 * self.stringer_radius[station]
                stringer_spacing = available_length / (extra_stringers[idx] + 1)
                b1 = self.section_map[self.section_order[idx]][0]
                b2 = self.section_map[self.section_order[idx]][1]

                dy = (self.boom_coords[b1]['coords'][0][station] - self.boom_coords[b2]['coords'][0][station]) 
                dz = (self.boom_coords[b1]['coords'][1][station] - self.boom_coords[b2]['coords'][1][station])

                for i in range(extra_stringers[idx]):

                    y_coord = self.boom_coords[b1]['coords'][0][station] - (i + 1) * stringer_spacing * dy / available_length 
                    z_coord = self.boom_coords[b1]['coords'][1][station] - (i + 1) * stringer_spacing * dz / available_length 

                    y_centroid = y_coord
                    z_centroid = z_coord - self.z_bar[station]

                    I_yy += self.stringer_areas[station] * z_centroid**2
                    I_zz += self.stringer_areas[station] * y_centroid**2

                    self.stringer_dict[station_idx]['y'].append(y_coord)
                    self.stringer_dict[station_idx]['z'].append(z_coord)
                
                if idx == 0:
                    self.top_spacings.append(stringer_spacing)
                elif idx == 1:
                    self.side_spacings.append(stringer_spacing)
                elif idx == 2:
                    self.slope_spacings.append(stringer_spacing)

            station_I_yy.append(I_yy)
            station_I_zz.append(I_zz)
        self.stringer_I_yy = np.array(station_I_yy)
        self.stringer_I_zz = np.array(station_I_zz)

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

        self.I_yy_all += self.stringer_I_yy
        self.I_zz_all += self.stringer_I_zz

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

        self.calculate_base_shearflows(V_y=V_y, V_z=V_z)

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
            '12': [],
            '13': [],
            '35': [],
            '45': [],
            '24': []
        }

        for i in range(len(self.x_points)):
            V_z = self.V_internal[i]
            V_y = 0
            self.calculate_shear_flow_distribution(V_z, V_y)
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - self.x_points[i]))

            for idx,q_dist in enumerate(self.shear_flow_dicts):
                for i,q in enumerate(list(q_dist.values())):
                    section_key = self.section_order[i]
                    if q != 0:
                        shear_stress = abs(q / (self.final_thicknesses[idx][station_idx]))
                    else:
                        shear_stress = 0.0
                    self.shear_stresses[section_key].append(shear_stress)
                    
        self.shear_stresses = {k: np.array(v) for k, v in self.shear_stresses.items()}

    def calculate_bending_stress(self):
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

        return self.bending_stresses
    
    def calculate_boom_thicknesses(self):

        boom_connections = [('B1', 'B2', 'B3'), ('B2', 'B1', 'B4'), ('B3', 'B1', 'B5'), ('B4', 'B2', 'B5'), ('B5', 'B3', 'B4')]
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

    def calculate_critical_buckling_stress(self):
        factor = self.C * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2)))

        spacing_factor = self.top_spacings - self.n_stringers/2 * self.stringer_radius
        if any(i for i in spacing_factor < 0.08):
            spacing_factor = np.where(spacing_factor < 0.08, 0.08, spacing_factor)
        sigma_cr_skin = factor * (self.final_thicknesses[0] / (spacing_factor))**2 
        sigma_cr_combined = ((self.stringer_areas + self.n_stringers*self.stringer_radius*self.final_thicknesses[0]) * self.sigma_y + (self.boom_coords['B1']['area']*2 * sigma_cr_skin)) / (self.stringer_areas + self.boom_coords['B1']['area']*2)

        self.sigma_buckle_cr = np.zeros(len(self.x_points))

        for i in range(len(self.x_points)):
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - self.x_points[i]))

            self.sigma_buckle_cr[i] = sigma_cr_combined[station_idx]/self.safety_factor

        return self.sigma_buckle_cr
    
    def calculate_critical_web_stress(self):
        
        self.tau_cr_side = self.k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.final_thicknesses[1] / self.side_spacings-2*self.stringer_radius)**2
        self.tau_cr_slope = self.k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.final_thicknesses[2] / self.slope_spacings-2*self.stringer_radius)**2
        self.tau_cr_top = self.k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.final_thicknesses[0] / self.top_spacings-2*self.stringer_radius)**2

        self.side_web_stress = np.zeros(len(self.x_points))
        self.slope_web_stress = np.zeros(len(self.x_points))
        self.top_web_stress = np.zeros(len(self.x_points))
        for i in range(len(self.x_points)):
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - self.x_points[i]))
            self.side_web_stress[i] = self.tau_cr_side[station_idx] / self.safety_factor
            self.slope_web_stress[i] = self.tau_cr_slope[station_idx] / self.safety_factor
            self.top_web_stress[i] = self.tau_cr_top[station_idx] / self.safety_factor
        
        self.tau_map = {
            '12': self.top_web_stress,
            '13': self.side_web_stress,
            '35': self.slope_web_stress,
            '45': self.slope_web_stress,
            '24': self.side_web_stress
        }
        
    def get_margins(self):

        self.section_margins_bending = {}
        self.critical_margins = {'bending': {}, 'shear': {}} 

        for stress in self.bending_stresses:
            prev_idx = 0
            bending_stress = self.bending_stresses[stress]
            self.section_margins_bending[stress] = {'yield': [], 'buckling': []}
            self.margin_yield = abs(self.sigma_y / self.safety_factor / bending_stress)
            self.margin_buckle = abs(self.sigma_buckle_cr / bending_stress)

            for i in range(len(self.thresholds)):
                curr_idx = np.argmin(np.abs(self.x_points - self.thresholds[i]))

                station_yield = self.margin_yield[prev_idx:curr_idx]
                station_buckle = self.margin_buckle[prev_idx:curr_idx]

                if i == 2:
                    station_yield = self.margin_yield[curr_idx:]
                    station_buckle = self.margin_buckle[curr_idx:]

                min_yield = min(station_yield)
                min_buckle = min(station_buckle)

                self.section_margins_bending[stress]['yield'].append(min_yield)
                self.section_margins_bending[stress]['buckling'].append(min_buckle)

                if 1 < min_yield < 2:
                    self.critical_margins['bending'][(stress, 'yield', i)] = min_yield
                if 1 < min_buckle < 2:
                    self.critical_margins['bending'][(stress, 'buckling', i)] = min_buckle

                prev_idx = curr_idx

        self.section_margins_shear = {}
        for section in self.shear_stresses:
            prev_idx = 0
            shear_stress = self.shear_stresses[section]
            self.section_margins_shear[section] = {'yield': [], 'buckling': []}
            margin_yield = abs(self.shear_yield/self.safety_factor / shear_stress)
            margin_buckle = abs(self.tau_map[section] / shear_stress)

            for i in range(len(self.thresholds)):
                curr_idx = np.argmin(np.abs(self.x_points - self.thresholds[i]))

                station_yield = margin_yield[prev_idx:curr_idx]
                station_buckle = margin_buckle[prev_idx:curr_idx]

                if i == 2:
                    station_yield = margin_yield[curr_idx:]
                    station_buckle = margin_buckle[curr_idx:]

                min_yield = min(station_yield)
                min_buckle = min(station_buckle)

                self.section_margins_shear[section]['yield'].append(min_yield)
                self.section_margins_shear[section]['buckling'].append(min_buckle)

                if 1 < min_yield < 2:
                    self.critical_margins['shear'][(section, 'yield', i)] = min_yield
                if 1 < min_buckle < 2:
                    self.critical_margins['shear'][(section, 'buckling', i)] = min_buckle

                prev_idx = curr_idx

        self.optimize_parameters()


    def optimize_parameters(self, k=1.0):

        self.prev_B1 = self.B1.copy()
        self.prev_stringer_area = self.stringer_areas.copy()
        self.prev_n_stringers = self.n_stringers.copy()

        boom_scale = 1.15
        stringer_scale = 1.01

        for boom in self.boom_map:
            yield_margin = np.array(self.section_margins_bending[boom]['yield'])
            buckle_margin = np.array(self.section_margins_bending[boom]['buckling'])

            if np.any(yield_margin < 1):
                indices = np.where(yield_margin < 1)[0]

                for idx in indices:
                    self.boom_map[boom][idx] *= boom_scale
            
            if np.any(buckle_margin < 1):
                indices = np.where(buckle_margin < 1)[0]
                for idx in indices:
                    self.boom_map[boom][idx] *= boom_scale
                    self.n_stringers[idx] = np.ceil(self.n_stringers[idx] * stringer_scale)

        for section in self.section_margins_shear:
            yield_margin = np.array(self.section_margins_shear[section]['yield'])
            buckle_margin = np.array(self.section_margins_shear[section]['buckling'])

            if np.any(yield_margin < 1):
                indices = np.where(yield_margin < 1)[0]
                b1, b2 = self.section_map[section]
                for idx in indices:
                    self.boom_map[b1][idx] *= boom_scale
                    self.boom_map[b2][idx] *= boom_scale
            
            if np.any(buckle_margin < 1):
                indices = np.where(buckle_margin < 1)[0]
                for idx in indices:
                    self.n_stringers[idx] = np.ceil(self.n_stringers[idx] * stringer_scale)
                    self.boom_map[boom][idx] *= boom_scale
                
        
        bending_safe = all(
            min(y, b) >= 1.0
            for boom in self.section_margins_bending
            for y, b in zip(self.section_margins_bending[boom]['yield'], self.section_margins_bending[boom]['buckling'])
            if np.isfinite(y) and np.isfinite(b)
        )

        shear_safe = all(
            min(y, b) >= 1.0
            for sec in self.section_margins_shear
            for y, b in zip(self.section_margins_shear[sec]['yield'], self.section_margins_shear[sec]['buckling'])
            if np.isfinite(y) and np.isfinite(b)
        )

        converged = (
            np.allclose(self.B1, self.prev_B1, atol=self.tolerance) and
            np.allclose(self.stringer_areas, self.prev_stringer_area, atol=self.tolerance) and
            bending_safe and shear_safe
        )

        if not converged and self.iteration < self.max_iterations:
            self.iteration += 1
            if self.iteration % 10 == 0:
                print(f"Iteration {self.iteration}: Parameters not converged, optimizing...")
            self.main()
        else:
            print(f"Converged after {self.iteration} iterations.")
            self.plot_station_cross_section(1)
            self.calculate_rib_spacing_skin()
            self.calculate_mass()
            if self.plot:
                self.plot_shear_stress()
                self.plot_bending_stress()
            print(f"Frame Amount: {self.rib_amount}")
            print("Final Thicknesses:", self.final_thicknesses)
            print("Final Boom Areas:", self.boom_map)
            print("Final Stringer Area:", self.stringer_areas)
            print("Final Number of Stringers:", self.n_stringers)
            self.update_attributes()

    def calculate_rib_spacing_skin(self):
        factor = (np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))

        self.rib_spacings = []
        self.rib_positions = []
        self.rib_span_indices = []

        x = 0.0
        self.rib_positions.append(x)

        while x < self.l_fuselage:
            print(x)
            idx = np.argmin(np.abs(self.x_points - x))
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - x))

            stress = abs(self.bending_stresses['B1'][idx])
            if round(stress, 2) == 0.00:
                b = 40 * 0.0254  
            else:
                thickness = self.final_thicknesses[1][station_idx]
                b = min(40 * 0.0254, thickness * np.sqrt(self.C * factor / stress))
                print(b,thickness)
            self.rib_spacings.append(b)
            x += b

            if x < self.l_fuselage:
                self.rib_positions.append(x)
            else:
                break

        self.rib_amount = len(self.rib_positions)
        print("Rib Count:", self.rib_amount)
        self.calculate_rib_thickness()

    def calculate_frame_width(self, thickness, tau_cr):
        factor = (12 * (1 - self.poisson_ratio**2)) / (self.k_s * np.pi**2 * self.E)

        b_required = min(thickness/np.sqrt(tau_cr * factor),0.15)

        return b_required

    def calculate_rib_thickness(self):
        self.rib_thicknesses = []
        self.rib_widths = []
        for i in range(len(self.rib_positions)):
            x = self.rib_positions[i]
            idx = np.argmin(np.abs(self.x_points - x))
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - x))
            thickness = self.final_thicknesses[1][station_idx]
            self.rib_thicknesses.append(thickness)

            tau_cr = self.tau_map['13'][idx]*self.safety_factor

            frame_width = self.calculate_frame_width(thickness, tau_cr)
            self.rib_widths.append(frame_width)
        
    def calculate_mass(self):

        section_names = self.section_order 

        skin_area_along_fuselage = np.zeros(len(self.x_points))
        stringer_area_along_fuselage = np.zeros(len(self.x_points))
        stringer_area_epoxy = np.zeros(len(self.x_points))
        skin_area_epoxy = np.zeros(len(self.x_points))
        max_thickness = max([max(i) for i in self.final_thicknesses])
        epoxy_thickness_in = max_thickness*self.epoxy_in
        epoxy_thickness_out = max_thickness*self.epoxy_out
        for i, x in enumerate(self.x_points):
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - x))

            skin_area = 0.0
            epoxy_skin_area = 0.0
            for sec_idx, sec in enumerate(section_names):
                thickness = self.final_thicknesses[sec_idx][station_idx] if isinstance(self.final_thicknesses[sec_idx], (list, np.ndarray)) else self.final_thicknesses[sec_idx]
                length = self.section_lengths[sec_idx][station_idx] if isinstance(self.section_lengths[sec_idx], (list, np.ndarray, np.ndarray)) else self.section_lengths[sec_idx]
                skin_area += thickness * length
                epoxy_skin_area += length*(epoxy_thickness_in + epoxy_thickness_out)

            stringer_area = self.stringer_areas[station_idx] * self.n_stringers[station_idx]
            stringer_epoxy = stringer_area * (self.epoxy_in + self.epoxy_out)

            skin_area_along_fuselage[i] = skin_area
            stringer_area_along_fuselage[i] = stringer_area
            stringer_area_epoxy[i] = stringer_epoxy
            skin_area_epoxy[i] = epoxy_skin_area

        rib_volume = 0.0
        for rib in range(len(self.rib_positions)):
            station_idx = np.argmin(np.abs(np.array(self.thresholds) - self.rib_positions[rib]))

            for section in self.section_lengths:
                thickness = self.rib_thicknesses[rib]
                length = section[station_idx]
                width = self.rib_widths[rib]
                rib_volume += thickness * length * width
        
        self.rib_mass = rib_volume * self.rib_density
        print(len(skin_area_along_fuselage))
        skin_mass = np.trapz(skin_area_along_fuselage, self.x_points) * self.density
        stringer_mass = np.trapz(stringer_area_along_fuselage, self.x_points) * self.density
        self.epoxy_mass = np.trapz(skin_area_epoxy, self.x_points) * self.rho_epoxy + np.trapz(stringer_area_epoxy, self.x_points) * self.rho_epoxy
        total_mass = skin_mass + stringer_mass + self.rib_mass + self.epoxy_mass
        self.fuselage_mass = total_mass 

        print(f"Skin Mass: {skin_mass:.2f} kg")
        print(f"Frame Mass: {self.rib_mass:.2f} kg")
        print(f"Stringer Mass: {stringer_mass:.2f} kg")
        print(f"Epoxy Mass: {self.epoxy_mass:.2f} kg")
        print(f"Fuselage Mass (total): {total_mass:.2f} kg")

        # Optional: plot area distributions
        if self.plot:
            plt.plot(self.x_points, skin_area_along_fuselage, label='Skin Area Distribution')
            plt.plot(self.x_points, stringer_area_along_fuselage, label='Stringer Area Distribution')
            plt.xlabel('Position along Fuselage (m)')
            plt.ylabel('Area (m²)')
            plt.title('Fuselage Area Distribution')
            plt.legend()
            plt.grid()
            plt.show()

    def update_attributes(self):
        json_friendly_margins = {
            category: {
                f"{key[0]}|{key[1]}|{key[2]}": value
                for key, value in items.items()
            }
            for category, items in self.critical_margins.items()
        }
        self.aircraft_data.data['outputs']['fuselage_stress']['frame_amount'] = self.rib_amount
        self.aircraft_data.data['outputs']['fuselage_stress']['frame_thickness_1'] = self.final_thicknesses[1][0]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['frame_thickness_2'] = self.final_thicknesses[1][1]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['frame_thickness_3'] = self.final_thicknesses[1][2]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['frame_mass'] = self.rib_mass
        self.aircraft_data.data['outputs']['fuselage_stress']['side_thickness_1'] = self.final_thicknesses[1][0]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['side_thickness_2'] = self.final_thicknesses[1][1]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['side_thickness_3'] = self.final_thicknesses[1][2]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['hull_thickness_1'] = self.final_thicknesses[2][0]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['hull_thickness_2'] = self.final_thicknesses[2][1]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['hull_thickness_3'] = self.final_thicknesses[2][2]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['top_thickness_1'] = self.final_thicknesses[0][0]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['top_thickness_2'] = self.final_thicknesses[0][1]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['top_thickness_3'] = self.final_thicknesses[0][2]*1000
        self.aircraft_data.data['outputs']['fuselage_stress']['stringer_amount_1'] = float(self.n_stringers[0])
        self.aircraft_data.data['outputs']['fuselage_stress']['stringer_amount_2'] = float(self.n_stringers[1])
        self.aircraft_data.data['outputs']['fuselage_stress']['stringer_amount_3'] = float(self.n_stringers[2])
        self.aircraft_data.data['outputs']['fuselage_stress']['I_yy'] = float(sum(self.I_yy_all))
        self.aircraft_data.data['outputs']['fuselage_stress']['I_zz'] = float(sum(self.I_zz_all))
        self.aircraft_data.data['outputs']['fuselage_stress']['critical_margins'] = json_friendly_margins
        self.aircraft_data.data['outputs']['component_weights']['fuselage'] = self.fuselage_mass*9.81

        self.aircraft_data.save_design(self.design_file)


    def plot_shear_stress(self, evaluate_stress="13"):

        plt.plot(self.x_points, self.shear_stresses[evaluate_stress]/1e6, label='Shear Stress Distribution')
        plt.plot(self.x_points, np.full_like(self.x_points, np.sign(self.shear_stresses[evaluate_stress])*self.shear_yield/1e6), 'k--', label='Shear Yield Stress')
        plt.plot(self.x_points, np.sign(self.shear_stresses[evaluate_stress])*self.tau_map[evaluate_stress]/1e6, 'r--', label='Critical Shear Stress Side')
        plt.xlabel('Position along Fuselage (m)')
        plt.ylabel('Shear Stress (MPa)')
        plt.title('Shear Stress Distribution on Fuselage')
        plt.legend()
        plt.grid()
        plt.show()
    
    def plot_bending_stress(self, evaluate_stress="B1"):
        plt.plot(self.x_points, self.bending_stresses[evaluate_stress]/1e6, label='Bending Stress Distribution')
        plt.plot(self.x_points, np.full_like(self.x_points, np.sign(self.bending_stresses[evaluate_stress])*self.sigma_y/1e6), 'k--', label='Yield Stress')
        plt.plot(self.x_points, -self.sigma_buckle_cr/1e6, 'r--', label='Critical Buckling Stress')
        plt.xlabel('Position along Fuselage (m)')
        plt.ylabel('Bending Stress (MPa)')
        plt.title('Bending Stress Distribution on Fuselage')
        plt.legend()
        plt.grid()
        plt.show()
        
    def main(self):

        self.calculate_fuselage_centroid()

        self.calculate_shear_flow_distribution()
        self.calculate_bending_stress()
        self.calculate_boom_thicknesses()
        self.calculate_shear_stress()
        self.calculate_critical_buckling_stress()
        self.calculate_critical_web_stress()
        self.get_margins()

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

        # --- Process stringer data ---
        stringer_y = []
        stringer_z = []
        if station_name in self.stringer_dict:
            ys = self.stringer_dict[station_name]['y']
            zs = self.stringer_dict[station_name]['z']
            for y, z in zip(ys, zs):
                stringer_y.append(y)  
                stringer_z.append(z)

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

        # Plot stringers, but only add label to the first one for the legend
        for i, (y, z) in enumerate(zip(stringer_y, stringer_z)):
            label = "Stringer" if i == 0 else None
            ax.scatter(y, z, s=100*self.stringer_radius[station_idx]+10, c='red', edgecolor='black', zorder=10, label=label)

        # --- Draw boom connections ---
        for section_id in self.section_order:
            b1, b2 = self.section_map[section_id]
            if b1 in boom_positions and b2 in boom_positions:
                y1, z1 = boom_positions[b1]
                y2, z2 = boom_positions[b2]
                ax.plot([y1, y2], [z1, z2], 'k-', linewidth=2)

        # --- Additional connections ---
        for b1, b2 in [('B3', 'B5'), ('B4', 'B5')]:
            if b1 in boom_positions and b2 in boom_positions:
                y1, z1 = boom_positions[b1]
                y2, z2 = boom_positions[b2]
                ax.plot([y1, y2], [z1, z2], 'k-', linewidth=2)

        # --- Plot booms ---
        for boom_name, (y, z) in boom_positions.items():
            area = boom_areas.get(boom_name, 0)
            size = 3000 * area + 50
            boom_radius = np.sqrt(area * 4 / np.pi) / 2
            ax.scatter(y, z, s=size, c='blue', edgecolor='black', zorder=10,
                    label=f"{boom_name} ({area:.2e} m²)")

        ax.grid(True)
        ax.set_xlabel('y (m)')
        ax.set_ylabel('z (m)')
        ax.set_title(f'Fuselage Cross-Section at {station_name} (x = {self.thresholds[station_idx]:.2f} m)')
        ax.set_aspect('equal')
        ax.invert_xaxis()  # Flip x-axis so left is positive
        # --- Legend and layout ---
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.show()

        return fig, ax

        
if __name__ == '__main__':
    aircraft_data = Data("design3.json")
    airfoil_data = Data("AirfoilData/example_airfoil.json")
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