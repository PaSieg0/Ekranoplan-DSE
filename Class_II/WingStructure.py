import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data


class WingStructure:
    def __init__(self, aircraft_data: Data, airfoil_data: Data):
        self.data = airfoil_data.data
        self.aircraft_data = aircraft_data.data

        self.front_spar = self.aircraft_data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data['inputs']['structures']['wing_box']['rear_spar']

        self.t_front_spar = self.aircraft_data['inputs']['structures']['wing_box']['t_front_spar']
        self.t_rear_spar = self.aircraft_data['inputs']['structures']['wing_box']['t_rear_spar']
        self.t_skin = self.aircraft_data['inputs']['structures']['wing_box']['t_skin']

        self.b = self.aircraft_data['outputs']['wing_design']['b']
        self.chord_root = self.aircraft_data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data['outputs']['wing_design']['chord_tip']

        self.b_array = np.arange(0, self.b/2, 0.01)
        self.chord_array = self.chord_span_function(self.b_array)

        self.chord_length = self.chord_root

        self.n_stringers = self.aircraft_data['inputs']['structures']['wing_box']['n_stringers']
        self.stringer_area = self.aircraft_data['inputs']['structures']['wing_box']['stringer_area']
        self.stringer_radius = np.sqrt(self.stringer_area / np.pi)

        self.n_cells = self.aircraft_data['inputs']['structures']['wing_box']['n_cells']

        if self.n_cells == 2:
            self.mid_spar_1 = self.aircraft_data['inputs']['structures']['wing_box']['mid_spar_1']
            self.t_mid_spar_1 = self.aircraft_data['inputs']['structures']['wing_box']['t_mid_spar_1']

        elif self.n_cells == 3:
            self.mid_spar_1 = self.aircraft_data['inputs']['structures']['wing_box']['mid_spar_1']
            self.mid_spar_2 = self.aircraft_data['inputs']['structures']['wing_box']['mid_spar_2']
            self.t_mid_spar_1 = self.aircraft_data['inputs']['structures']['wing_box']['t_mid_spar_1']
            self.t_mid_spar_2 = self.aircraft_data['inputs']['structures']['wing_box']['t_mid_spar_2']
        
        self.bottom_spar_margin = 1.3
        self.top_spar_margin = 0.98


    def chord_span_function(self,y):
        return self.chord_root + (self.chord_tip - self.chord_root) / (self.b/2) * y

    def split_airfoil_surfaces(self):
        x = np.array(self.data['x']) * self.chord_length
        y = np.array(self.data['y']) * self.chord_length

        le_index = np.argmin(x)

        upper_x = x[:le_index + 1]
        upper_y = y[:le_index + 1]

        lower_x = x[le_index:]
        lower_y = y[le_index:]

        self.x_array = lower_x  # Denormalized
        self.y_upper = np.interp(self.x_array, upper_x[::-1], upper_y[::-1])
        self.y_lower = np.interp(self.x_array, lower_x, lower_y)

    def get_element_functions(self):
        self.element_functions = {'upper': [], 'lower': []}

        for idx in range(len(self.x_array) - 1):
            x0 = self.x_array[idx] 
            x1 = self.x_array[idx + 1] 

            yu0 = self.y_upper[idx]
            yu1 = self.y_upper[idx + 1]

            yl0 = self.y_lower[idx]
            yl1 = self.y_lower[idx + 1]

            length_u = np.sqrt((x1 - x0) ** 2 + (yu1 - yu0) ** 2)
            slope_u = (yu1 - yu0) / (x1 - x0)
            intercept_u = yu0 - slope_u * x0
            self.element_functions['upper'].append({
                'slope': slope_u, 'intercept': intercept_u, 'x_start': x0, 'x_end': x1, 'length': length_u
            })

            length_l = np.sqrt((x1 - x0) ** 2 + (yl1 - yl0) ** 2)
            slope_l = (yl1 - yl0) / (x1 - x0)
            intercept_l = yl0 - slope_l * x0
            self.element_functions['lower'].append({
                'slope': slope_l, 'intercept': intercept_l, 'x_start': x0, 'x_end': x1, 'length': length_l
            })


    def get_spar_heights(self):

        front_x = self.front_spar * self.chord_length
        rear_x = self.rear_spar * self.chord_length
        
        for segment in self.element_functions['upper']:
            if segment['x_start'] <= front_x <= segment['x_end']:
                front_spar_top = self.top_spar_margin*(segment['slope'] * front_x + segment['intercept'])
            if segment['x_start'] <= rear_x <= segment['x_end']:
                rear_spar_top = self.top_spar_margin*(segment['slope'] * rear_x + segment['intercept'])

        for segment in self.element_functions['lower']:
            if segment['x_start'] <= front_x <= segment['x_end']:
                front_spar_bottom = self.bottom_spar_margin*(segment['slope'] * front_x + segment['intercept'])
            if segment['x_start'] <= rear_x <= segment['x_end']:
                rear_spar_bottom = self.bottom_spar_margin*(segment['slope'] * rear_x + segment['intercept'])

        if self.n_cells == 1:
            self.spar_info = {
                'front_spar_height': front_spar_top - front_spar_bottom,
                'rear_spar_height': rear_spar_top - rear_spar_bottom,
                'front_spar_top': front_spar_top,
                'front_spar_bottom': front_spar_bottom,
                'rear_spar_top': rear_spar_top,
                'rear_spar_bottom': rear_spar_bottom,
                'front_spar_x': front_x,
                'rear_spar_x': rear_x
            }

        elif self.n_cells == 2:
            mid_x = self.mid_spar_1 * self.chord_length

            for segment in self.element_functions['upper']:
                if segment['x_start'] <= mid_x <= segment['x_end']:
                    mid_spar_top = self.top_spar_margin*(segment['slope'] * mid_x + segment['intercept'])

            for segment in self.element_functions['lower']:
                if segment['x_start'] <= mid_x <= segment['x_end']:
                    mid_spar_bottom = self.bottom_spar_margin*(segment['slope'] * mid_x + segment['intercept'])

            self.spar_info = {
                'front_spar_height': front_spar_top - front_spar_bottom,
                'rear_spar_height': rear_spar_top - rear_spar_bottom,
                'front_spar_top': front_spar_top,
                'front_spar_bottom': front_spar_bottom,
                'rear_spar_top': rear_spar_top,
                'rear_spar_bottom': rear_spar_bottom,
                'mid_spar_top_1': mid_spar_top,
                'mid_spar_bottom_1': mid_spar_bottom,
                'front_spar_x': front_x,
                'mid_spar_x_1': mid_x,
                'rear_spar_x': rear_x,
                'mid_spar_height_1': mid_spar_top - mid_spar_bottom,
            }

        elif self.n_cells == 3:
            mid_x_1 = self.mid_spar_1 * self.chord_length
            mid_x_2 = self.mid_spar_2 * self.chord_length

            for segment in self.element_functions['upper']:
                if segment['x_start'] <= mid_x_1 <= segment['x_end']:
                    mid_spar_top_1 = self.top_spar_margin*(segment['slope'] * mid_x_1 + segment['intercept'])
                if segment['x_start'] <= mid_x_2 <= segment['x_end']:
                    mid_spar_top_2 = self.top_spar_margin*(segment['slope'] * mid_x_2 + segment['intercept'])

            for segment in self.element_functions['lower']:
                if segment['x_start'] <= mid_x_1 <= segment['x_end']:
                    mid_spar_bottom_1 = self.bottom_spar_margin*(segment['slope'] * mid_x_1 + segment['intercept'])
                if segment['x_start'] <= mid_x_2 <= segment['x_end']:
                    mid_spar_bottom_2 = self.bottom_spar_margin*(segment['slope'] * mid_x_2 + segment['intercept'])

            self.spar_info = {
                'front_spar_height': front_spar_top - front_spar_bottom,
                'rear_spar_height': rear_spar_top - rear_spar_bottom,
                'front_spar_top': front_spar_top,
                'front_spar_bottom': front_spar_bottom,
                'rear_spar_top': rear_spar_top,
                'rear_spar_bottom': rear_spar_bottom,
                'mid_spar_top_1': mid_spar_top_1,
                'mid_spar_bottom_1': mid_spar_bottom_1,
                'mid_spar_top_2': mid_spar_top_2,
                'mid_spar_bottom_2': mid_spar_bottom_2,
                'front_spar_x': front_x,
                'mid_spar_x_1': mid_x_1,
                'mid_spar_x_2': mid_x_2,
                'rear_spar_x': rear_x,
                'mid_spar_height_1': mid_spar_top_1 - mid_spar_bottom_1,
                'mid_spar_height_2': mid_spar_top_2 - mid_spar_bottom_2
            }


    def compute_centroid(self):
        """
        Compute the geometric centroid (x̄, ȳ) of the airfoil cross-section.
        Assumes airfoil is a closed shape formed by upper then lower surface.
        """
        # Create closed polygon by concatenating upper and lower surfaces
        x_coords = np.concatenate([self.x_array, self.x_array[::-1]])
        y_coords = np.concatenate([self.y_upper, self.y_lower[::-1]])

        # Shoelace formula components
        A = 0  # Signed area
        Cx = 0
        Cy = 0
        n = len(x_coords)

        for i in range(n):
            x0, y0 = x_coords[i], y_coords[i]
            x1, y1 = x_coords[(i + 1) % n], y_coords[(i + 1) % n]
            cross = x0 * y1 - x1 * y0
            A += cross
            Cx += (x0 + x1) * cross
            Cy += (y0 + y1) * cross

        A *= 0.5
        if A == 0:
            raise ValueError("Degenerate airfoil shape: area is zero.")

        Cx /= (6 * A)
        Cy /= (6 * A)

        self.centroid = (Cx, Cy)
        #print(f"Airfoil Centroid: x̄ = {Cx:.4f}, ȳ = {Cy:.4f}")

    def get_wingbox_panels(self):
        front_x = self.front_spar * self.chord_length
        rear_x = self.rear_spar * self.chord_length

        spar_info = self.spar_info

        if self.n_cells == 1:
            top_panel_length = np.sqrt((spar_info['front_spar_top'] - spar_info['rear_spar_top']) ** 2 + (front_x - rear_x) ** 2)
            bottom_panel_length = np.sqrt((spar_info['front_spar_bottom'] - spar_info['rear_spar_bottom']) ** 2 + (front_x - rear_x) ** 2)

            top_panel_angle = np.arctan2(spar_info['front_spar_top'] - spar_info['rear_spar_top'], front_x - rear_x)
            bottom_panel_angle = np.arctan2(spar_info['front_spar_bottom'] - spar_info['rear_spar_bottom'], front_x - rear_x)
            
            self.panel_info = {
                'top_panel_length': top_panel_length,
                'bottom_panel_length': bottom_panel_length,
                'top_panel_angle': top_panel_angle,
                'bottom_panel_angle': bottom_panel_angle,
            }
        
        elif self.n_cells == 2:
            top_panel_length_1 = np.sqrt((spar_info['front_spar_top'] - spar_info['mid_spar_top_1']) ** 2 + (front_x - spar_info['mid_spar_x_1']) ** 2)
            bottom_panel_length_1 = np.sqrt((spar_info['front_spar_bottom'] - spar_info['mid_spar_bottom_1']) ** 2 + (front_x - spar_info['mid_spar_x_1']) ** 2)

            top_panel_length_2 = np.sqrt((spar_info['mid_spar_top_1'] - spar_info['rear_spar_top']) ** 2 + (spar_info['mid_spar_x_1'] - rear_x) ** 2)
            bottom_panel_length_2 = np.sqrt((spar_info['mid_spar_bottom_1'] - spar_info['rear_spar_bottom']) ** 2 + (spar_info['mid_spar_x_1'] - rear_x) ** 2)

            top_panel_angle_1 = np.arctan2(spar_info['front_spar_top'] - spar_info['mid_spar_top_1'], front_x - spar_info['mid_spar_x_1'])
            bottom_panel_angle_1 = np.arctan2(spar_info['front_spar_bottom'] - spar_info['mid_spar_bottom_1'], front_x - spar_info['mid_spar_x_1'])
            top_panel_angle_2 = np.arctan2(spar_info['mid_spar_top_1'] - spar_info['rear_spar_top'], spar_info['mid_spar_x_1'] - rear_x)
            bottom_panel_angle_2 = np.arctan2(spar_info['mid_spar_bottom_1'] - spar_info['rear_spar_bottom'], spar_info['mid_spar_x_1'] - rear_x)

            self.panel_info = {
                'top_panel_length_1': top_panel_length_1,
                'bottom_panel_length_1': bottom_panel_length_1,
                'top_panel_length_2': top_panel_length_2,
                'bottom_panel_length_2': bottom_panel_length_2,
                'top_panel_angle_1': top_panel_angle_1,
                'bottom_panel_angle_1': bottom_panel_angle_1,
                'top_panel_angle_2': top_panel_angle_2,
                'bottom_panel_angle_2': bottom_panel_angle_2,
            }

        elif self.n_cells == 3:
            top_panel_length_1 = np.sqrt((spar_info['front_spar_top'] - spar_info['mid_spar_top_1']) ** 2 + (front_x - spar_info['mid_spar_x_1']) ** 2)
            bottom_panel_length_1 = np.sqrt((spar_info['front_spar_bottom'] - spar_info['mid_spar_bottom_1']) ** 2 + (front_x - spar_info['mid_spar_x_1']) ** 2)

            top_panel_length_2 = np.sqrt((spar_info['mid_spar_top_1'] - spar_info['mid_spar_top_2']) ** 2 + (spar_info['mid_spar_x_1'] - spar_info['mid_spar_x_2']) ** 2)
            bottom_panel_length_2 = np.sqrt((spar_info['mid_spar_bottom_1'] - spar_info['mid_spar_bottom_2']) ** 2 + (spar_info['mid_spar_x_1'] - spar_info['mid_spar_x_2']) ** 2)

            top_panel_length_3 = np.sqrt((spar_info['mid_spar_top_2'] - spar_info['rear_spar_top']) ** 2 + (spar_info['mid_spar_x_2'] - rear_x) ** 2)
            bottom_panel_length_3 = np.sqrt((spar_info['mid_spar_bottom_2'] - spar_info['rear_spar_bottom']) ** 2 + (spar_info['mid_spar_x_2'] - rear_x) ** 2)

            top_panel_angle_1 = np.arctan2(spar_info['front_spar_top'] - spar_info['mid_spar_top_1'], front_x - spar_info['mid_spar_x_1'])
            bottom_panel_angle_1 = np.arctan2(spar_info['front_spar_bottom'] - spar_info['mid_spar_bottom_1'], front_x - spar_info['mid_spar_x_1'])
            top_panel_angle_2 = np.arctan2(spar_info['mid_spar_top_1'] - spar_info['mid_spar_top_2'], spar_info['mid_spar_x_1'] - spar_info['mid_spar_x_2'])
            bottom_panel_angle_2 = np.arctan2(spar_info['mid_spar_bottom_1'] - spar_info['mid_spar_bottom_2'], spar_info['mid_spar_x_1'] - spar_info['mid_spar_x_2'])
            top_panel_angle_3 = np.arctan2(spar_info['mid_spar_top_2'] - spar_info['rear_spar_top'], spar_info['mid_spar_x_2'] - rear_x)
            bottom_panel_angle_3 = np.arctan2(spar_info['mid_spar_bottom_2'] - spar_info['rear_spar_bottom'], spar_info['mid_spar_x_2'] - rear_x)

            self.panel_info = {
                'top_panel_length_1': top_panel_length_1,
                'bottom_panel_length_1': bottom_panel_length_1,
                'top_panel_length_2': top_panel_length_2,
                'bottom_panel_length_2': bottom_panel_length_2,
                'top_panel_length_3': top_panel_length_3,
                'bottom_panel_length_3': bottom_panel_length_3,
                'top_panel_angle_1': top_panel_angle_1,
                'bottom_panel_angle_1': bottom_panel_angle_1,
                'top_panel_angle_2': top_panel_angle_2,
                'bottom_panel_angle_2': bottom_panel_angle_2,
                'top_panel_angle_3': top_panel_angle_3,
                'bottom_panel_angle_3': bottom_panel_angle_3,
            }

    
    def beam_standard_Ixx(self, length, thickness, distance, angle):
        A_web = length * thickness/1000
        I_web = (thickness/1000*length**3*np.sin(angle)**2) / 12
        I_xx = I_web + A_web * distance ** 2

        return I_xx
    
    def beam_standard_Iyy(self, length, thickness, distance, angle):
        A_web = length * thickness/1000
        I_web = (thickness/1000*length**3*np.cos(angle)**2) / 12
        I_xx = I_web + A_web * distance ** 2

        return I_xx
    
    def beam_standard_Ixy(self, length, thickness, xy, angle):

        A_web = length * thickness/1000

        I_web = (thickness/1000*length**3*np.sin(angle)*np.cos(angle)) / 12

        I_xx = I_web + A_web * xy

        return I_xx
    
    def get_moment_of_inertia(self):


        front_spar_x = self.spar_info['front_spar_x']
        rear_spar_x = self.spar_info['rear_spar_x']
        mid_x = (front_spar_x + rear_spar_x) / 2
        front_spar_height = self.spar_info['front_spar_height']
        rear_spar_height = self.spar_info['rear_spar_height']
        front_spar_top = self.spar_info['front_spar_top']
        front_spar_bottom = self.spar_info['front_spar_bottom']

        I_xx_front_spar = self.beam_standard_Ixx(front_spar_height, self.t_front_spar, abs(front_spar_height/2-self.centroid[1]), np.pi/2)
        I_yy_front_spar = self.beam_standard_Iyy(front_spar_height, self.t_front_spar, abs(front_spar_x-self.centroid[0]), np.pi/2)
        I_xy_front_spar = self.beam_standard_Ixy(front_spar_height, self.t_front_spar, (front_spar_x - self.centroid[0])*(front_spar_height/2-self.centroid[1]), np.pi/2)

        I_xx_rear_spar = self.beam_standard_Ixx(rear_spar_height, self.t_rear_spar, abs(rear_spar_height/2-self.centroid[1]), np.pi/2)
        I_yy_rear_spar = self.beam_standard_Iyy(rear_spar_height, self.t_rear_spar, abs(rear_spar_x-self.centroid[0]), np.pi/2)
        I_xy_rear_spar = self.beam_standard_Ixy(rear_spar_height, self.t_rear_spar, (rear_spar_x - self.centroid[0])*(rear_spar_height/2-self.centroid[1]), np.pi/2)

        I_xx_panels = 0
        I_yy_panels = 0
        I_xy_panels = 0

        if self.n_cells == 1:

            top_panel_angle = self.panel_info['top_panel_angle']
            bottom_panel_angle = self.panel_info['bottom_panel_angle']

            top_panel_length = self.panel_info['top_panel_length']
            bottom_panel_length = self.panel_info['bottom_panel_length']
            I_xx_panels += self.beam_standard_Ixx(top_panel_length, self.t_skin, abs(0.5*top_panel_length*np.sin(top_panel_angle)+front_spar_top-self.centroid[1]), top_panel_angle)
            I_yy_panels += self.beam_standard_Iyy(top_panel_length, self.t_skin, abs(0.5*top_panel_length*np.cos(top_panel_angle)- self.centroid[0]), top_panel_angle)
            I_xy_panels += self.beam_standard_Ixy(top_panel_length, self.t_skin, (0.5*top_panel_length*np.cos(top_panel_angle)-self.centroid[0])*(0.5*top_panel_length*np.sin(top_panel_angle)+front_spar_top-self.centroid[1]), top_panel_angle)

            I_xx_panels += self.beam_standard_Ixx(bottom_panel_length, self.t_skin, abs(0.5*bottom_panel_length*np.sin(bottom_panel_angle)+front_spar_bottom-self.centroid[1]), bottom_panel_angle)
            I_yy_panels += self.beam_standard_Iyy(bottom_panel_length, self.t_skin, abs(0.5*bottom_panel_length*np.cos(bottom_panel_angle)-self.centroid[0]), bottom_panel_angle)
            I_xy_panels += self.beam_standard_Ixy(bottom_panel_length, self.t_skin, (0.5*bottom_panel_length*np.cos(bottom_panel_angle)-self.centroid[0])*(0.5*bottom_panel_length*np.sin(bottom_panel_angle)+front_spar_bottom-self.centroid[1]), bottom_panel_angle)


        elif self.n_cells > 1:
            
            I_xx_mid_spars = 0
            I_yy_mid_spars = 0
            I_xy_mid_spars = 0

            I_xx_panels = 0
            I_yy_panels = 0
            I_xy_panels = 0

            for i in range(1, self.n_cells):
                front_spar_x = self.spar_info[f'front_spar_x']
                rear_spar_x = self.spar_info[f'rear_spar_x']
                mid_x = (front_spar_x + rear_spar_x) / 2
                front_spar_height = self.spar_info[f'front_spar_height']
                rear_spar_height = self.spar_info[f'rear_spar_height']
                front_spar_top = self.spar_info[f'front_spar_top']
                front_spar_bottom = self.spar_info[f'front_spar_bottom']

                top_panel_angle = self.panel_info[f'top_panel_angle_{i}']
                bottom_panel_angle = self.panel_info[f'bottom_panel_angle_{i}']

                top_panel_length = self.panel_info[f'top_panel_length_{i}']
                bottom_panel_length = self.panel_info[f'bottom_panel_length_{i}']

                mid_spar_top = self.spar_info[f'mid_spar_top_{i}']
                mid_spar_bottom = self.spar_info[f'mid_spar_bottom_{i}']
                mid_spar_x = self.spar_info[f'mid_spar_x_{i}']
                mid_spar_height = self.spar_info[f'mid_spar_height_{i}']

                I_xx_mid_spars += self.beam_standard_Ixx(mid_spar_height, self.t_mid_spar_1, abs(mid_spar_height/2-self.centroid[1]), np.pi/2)
                I_yy_mid_spars += self.beam_standard_Iyy(mid_spar_height, self.t_mid_spar_1, abs(mid_spar_x-self.centroid[0]), np.pi/2)
                I_xy_mid_spars += self.beam_standard_Ixy(mid_spar_height, self.t_mid_spar_1, (mid_spar_x - self.centroid[0])*(mid_spar_height/2-self.centroid[1]), np.pi/2)

                I_xx_panels += self.beam_standard_Ixx(top_panel_length, self.t_skin, abs(0.5*top_panel_length*np.sin(top_panel_angle)+front_spar_top-self.centroid[1]), top_panel_angle)
                I_yy_panels += self.beam_standard_Iyy(top_panel_length, self.t_skin, abs(0.5*top_panel_length*np.cos(top_panel_angle)- self.centroid[0]), top_panel_angle)
                I_xy_panels += self.beam_standard_Ixy(top_panel_length, self.t_skin, (0.5*top_panel_length*np.cos(top_panel_angle)-self.centroid[0])*(0.5*top_panel_length*np.sin(top_panel_angle)+front_spar_top-self.centroid[1]), top_panel_angle)

                I_xx_panels += self.beam_standard_Ixx(bottom_panel_length, self.t_skin, abs(0.5*bottom_panel_length*np.sin(bottom_panel_angle)+front_spar_bottom-self.centroid[1]), bottom_panel_angle)
                I_yy_panels += self.beam_standard_Iyy(bottom_panel_length, self.t_skin, abs(0.5*bottom_panel_length*np.cos(bottom_panel_angle)-self.centroid[0]), bottom_panel_angle)
                I_xy_panels += self.beam_standard_Ixy(bottom_panel_length, self.t_skin, (0.5*bottom_panel_length*np.cos(bottom_panel_angle)-self.centroid[0])*(0.5*bottom_panel_length*np.sin(bottom_panel_angle)+front_spar_bottom-self.centroid[1]), bottom_panel_angle)

        I_xx_wing_top = 0
        I_yy_wing_top = 0
        I_xy_wing_top = 0
        for i in self.element_functions['upper']:
            length = i['length']
            x_start = i['x_start']
            x_end = i['x_end']

            slope = i['slope']
            intercept = i['intercept']
            x_centroid = (x_start + x_end) / 2
            y_centroid = slope * x_centroid + intercept

            distance_xx = y_centroid - self.centroid[1]
            distance_yy = x_centroid - self.centroid[0]
            xy = (x_centroid - self.centroid[0])*(y_centroid - self.centroid[1])

            I_xx_wing_top += self.beam_standard_Ixx(length, self.t_skin, abs(distance_xx), np.arctan(slope))
            I_yy_wing_top += self.beam_standard_Iyy(length, self.t_skin, abs(distance_yy), np.arctan(slope))
            I_xy_wing_top += self.beam_standard_Ixy(length, self.t_skin, xy, np.arctan(slope))

        I_xx_wing_bottom = 0
        I_yy_wing_bottom = 0
        I_xy_wing_bottom = 0
        for i in self.element_functions['lower']:
            length = i['length']
            x_start = i['x_start']
            x_end = i['x_end']

            slope = i['slope']
            intercept = i['intercept']
            x_centroid = (x_start + x_end) / 2
            y_centroid = slope * x_centroid + intercept

            distance_xx = y_centroid - self.centroid[1]
            distance_yy = x_centroid - self.centroid[0]
            xy = (x_centroid - self.centroid[0])*(y_centroid - self.centroid[1])

            I_xx_wing_bottom += self.beam_standard_Ixx(length, self.t_skin, abs(distance_xx), np.arctan(slope))
            I_yy_wing_bottom += self.beam_standard_Iyy(length, self.t_skin, abs(distance_yy), np.arctan(slope))
            I_xy_wing_bottom += self.beam_standard_Ixy(length, self.t_skin, xy, np.arctan(slope))
            
        self.I_xx = I_xx_front_spar + I_xx_rear_spar + I_xx_panels + I_xx_wing_top + I_xx_wing_bottom
        self.I_yy = I_yy_front_spar + I_yy_rear_spar + I_yy_panels + I_yy_wing_top + I_yy_wing_bottom
        self.I_xy = I_xy_front_spar + I_xy_rear_spar + I_xy_panels + I_xy_wing_top + I_xy_wing_bottom

    def get_wing_structure(self):
        self.wing_structure = {}
    
        for idx, chord in enumerate(self.chord_array):
            self.chord_length = chord

            self.wing_structure[idx] = {}

            self.split_airfoil_surfaces()
            self.get_element_functions()  
            self.compute_centroid()
            self.get_spar_heights()
            self.get_wingbox_panels()
            self.get_moment_of_inertia()

            self.wing_structure[idx]['elements'] = self.element_functions
            self.wing_structure[idx]['spar_info'] = self.spar_info
            self.wing_structure[idx]['centroid'] = self.centroid
            self.wing_structure[idx]['panel_info'] = self.panel_info
            self.wing_structure[idx]['chord'] = chord
            self.wing_structure[idx]['x_coords'] = self.x_array
            self.wing_structure[idx]['y_upper'] = self.y_upper
            self.wing_structure[idx]['y_lower'] = self.y_lower
            self.wing_structure[idx]['I_xx'] = self.I_xx
            self.wing_structure[idx]['I_yy'] = self.I_yy
            self.wing_structure[idx]['I_xy'] = self.I_xy

        self.plot_moment_of_inertia()

    def plot_moment_of_inertia(self):

        fig, ax = plt.subplots()

        ax.plot(self.b_array, [wing['I_xx'] for wing in self.wing_structure.values()], label='I_xx', color='blue')
        #ax.plot(self.b_array, [wing['I_yy'] for wing in self.wing_structure.values()], label='I_yy', color='red')
        ax.plot(self.b_array, [wing['I_xy'] for wing in self.wing_structure.values()], label='I_xy', color='green')
        ax.set_xlabel('Chord Length (m)')
        ax.set_ylabel('Moment of Inertia (m^4)')
        ax.set_title('Moment of Inertia vs Chord Length')
        ax.legend()
        ax.grid()
        plt.show()

    
    def plot_airfoil(self,chord_idx=0):

        if self.n_cells == 1:
            front_spar_top = self.wing_structure[chord_idx]['spar_info']['front_spar_top']
            front_spar_bottom = self.wing_structure[chord_idx]['spar_info']['front_spar_bottom']
            rear_spar_top = self.wing_structure[chord_idx]['spar_info']['rear_spar_top']
            rear_spar_bottom = self.wing_structure[chord_idx]['spar_info']['rear_spar_bottom']
            

            x_array = self.wing_structure[chord_idx]['x_coords']
            y_upper = self.wing_structure[chord_idx]['y_upper']
            y_lower = self.wing_structure[chord_idx]['y_lower']

            centroid = self.wing_structure[chord_idx]['centroid']
            chord = self.chord_array[chord_idx]
            front_x = self.front_spar * chord
            rear_x = self.rear_spar * chord

            plt.figure(figsize=(10, 5))
            plt.plot(x_array, y_upper, label='Airfoil Profile', color='blue')
            plt.plot(x_array, y_lower, color='blue')
            plt.plot([front_x, front_x], [front_spar_bottom, front_spar_top], color='red', label='Front Spar')
            plt.plot([rear_x, rear_x], [rear_spar_bottom, rear_spar_top], color='red', label='Rear Spar')
            plt.plot([front_x, rear_x], [front_spar_top, rear_spar_top], color='red')
            plt.plot([front_x, rear_x], [front_spar_bottom, rear_spar_bottom], color='red')
            plt.fill_between([front_x, rear_x], [front_spar_bottom, rear_spar_bottom], [front_spar_top, rear_spar_top], color='gray', alpha=0.5, label='Wing Box')


        elif self.n_cells == 2:
            front_spar_top = self.wing_structure[chord_idx]['spar_info']['front_spar_top']
            front_spar_bottom = self.wing_structure[chord_idx]['spar_info']['front_spar_bottom']
            rear_spar_top = self.wing_structure[chord_idx]['spar_info']['rear_spar_top']
            rear_spar_bottom = self.wing_structure[chord_idx]['spar_info']['rear_spar_bottom']
            mid_spar_top = self.wing_structure[chord_idx]['spar_info']['mid_spar_top']
            mid_spar_bottom = self.wing_structure[chord_idx]['spar_info']['mid_spar_bottom']

            x_array = self.wing_structure[chord_idx]['x_coords']
            y_upper = self.wing_structure[chord_idx]['y_upper']
            y_lower = self.wing_structure[chord_idx]['y_lower']

            centroid = self.wing_structure[chord_idx]['centroid']
            chord = self.chord_array[chord_idx]
            front_x = self.front_spar * chord
            rear_x = self.rear_spar * chord
            mid_x = self.mid_spar_1 * chord

            plt.figure(figsize=(10, 5))
            plt.plot(x_array, y_upper, label='Airfoil Profile', color='blue')
            plt.plot(x_array, y_lower, color='blue')
            plt.plot([front_x, front_x], [front_spar_bottom, front_spar_top], color='red', label='Front Spar')
            plt.plot([rear_x, rear_x], [rear_spar_bottom, rear_spar_top], color='red', label='Rear Spar')
            plt.plot([mid_x, mid_x], [mid_spar_bottom, mid_spar_top], color='red', label='Mid Spar')
            plt.plot([front_x, mid_x], [front_spar_top, mid_spar_top], color='red')
            plt.plot([front_x, mid_x], [front_spar_bottom, mid_spar_bottom], color='red')
            plt.plot([mid_x, rear_x], [mid_spar_top, rear_spar_top], color='red')
            plt.plot([mid_x, rear_x], [mid_spar_bottom, rear_spar_bottom], color='red')
            plt.fill_between([front_x, mid_x], [front_spar_bottom, mid_spar_bottom], [front_spar_top, mid_spar_top], color='gray', alpha=0.5, label='Wing Box')
            plt.fill_between([mid_x, rear_x], [mid_spar_bottom, rear_spar_bottom], [mid_spar_top, rear_spar_top], color='gray', alpha=0.5, label='Wing Box')
        
        elif self.n_cells == 3:
            front_spar_top = self.wing_structure[chord_idx]['spar_info']['front_spar_top']
            front_spar_bottom = self.wing_structure[chord_idx]['spar_info']['front_spar_bottom']
            rear_spar_top = self.wing_structure[chord_idx]['spar_info']['rear_spar_top']
            rear_spar_bottom = self.wing_structure[chord_idx]['spar_info']['rear_spar_bottom']
            mid_spar_top_1 = self.wing_structure[chord_idx]['spar_info']['mid_spar_top_1']
            mid_spar_bottom_1 = self.wing_structure[chord_idx]['spar_info']['mid_spar_bottom_1']
            mid_spar_top_2 = self.wing_structure[chord_idx]['spar_info']['mid_spar_top_2']
            mid_spar_bottom_2 = self.wing_structure[chord_idx]['spar_info']['mid_spar_bottom_2']

            x_array = self.wing_structure[chord_idx]['x_coords']
            y_upper = self.wing_structure[chord_idx]['y_upper']
            y_lower = self.wing_structure[chord_idx]['y_lower']

            centroid = self.wing_structure[chord_idx]['centroid']
            chord = self.chord_array[chord_idx]
            front_x = self.front_spar * chord
            rear_x = self.rear_spar * chord
            mid_x_1 = self.mid_spar_1 * chord
            mid_x_2 = self.mid_spar_2 * chord

            plt.figure(figsize=(10, 5))
            plt.plot(x_array, y_upper, label='Airfoil Profile', color='blue')
            plt.plot(x_array, y_lower, color='blue')
            plt.plot([front_x, front_x], [front_spar_bottom, front_spar_top], color='red', label='Front Spar')
            plt.plot([rear_x, rear_x], [rear_spar_bottom, rear_spar_top], color='red', label='Rear Spar')
            plt.plot([mid_x_1, mid_x_1], [mid_spar_bottom_1, mid_spar_top_1], color='red', label='Mid Spar 1')
            plt.plot([mid_x_2, mid_x_2], [mid_spar_bottom_2, mid_spar_top_2], color='red', label='Mid Spar 2')
            plt.plot([front_x, mid_x_1], [front_spar_top, mid_spar_top_1], color='red')
            plt.plot([front_x, mid_x_1], [front_spar_bottom, mid_spar_bottom_1], color='red')
            plt.plot([mid_x_1, mid_x_2], [mid_spar_top_1, mid_spar_top_2], color='red')
            plt.plot([mid_x_1, mid_x_2], [mid_spar_bottom_1, mid_spar_bottom_2], color='red')
            plt.plot([mid_x_2, rear_x], [mid_spar_top_2, rear_spar_top], color='red')
            plt.plot([mid_x_2, rear_x], [mid_spar_bottom_2, rear_spar_bottom], color='red')
            plt.fill_between([front_x, mid_x_1], [front_spar_bottom, mid_spar_bottom_1], [front_spar_top, mid_spar_top_1], color='gray', alpha=0.5, label='Wing Box')
            plt.fill_between([mid_x_1, mid_x_2], [mid_spar_bottom_1, mid_spar_bottom_2], [mid_spar_top_1, mid_spar_top_2], color='gray', alpha=0.5, label='Wing Box')
            plt.fill_between([mid_x_2, rear_x], [mid_spar_bottom_2, rear_spar_bottom], [mid_spar_top_2, rear_spar_top], color='gray', alpha=0.5, label='Wing Box')

        plt.scatter(centroid[0], centroid[1], color='green', label='Centroid')
        plt.title(f"Airfoil with Spar Positions and Wing Box chord = {chord:.2f} m")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()


if __name__ == "__main__":
    airfoil_data = Data("airfoil_data.dat")
    aircraft_data = Data("design3.json")

    wing_structure = WingStructure(aircraft_data, airfoil_data)
    wing_structure.get_wing_structure()
    wing_structure.plot_airfoil(chord_idx=wing_structure.chord_array.shape[0]-1)