import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data


class WingStructure:
    def __init__(self, aircraft_data: Data, airfoil_data: Data):
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
        self.chord_array = self.chord_span_function(self.b_array)

        self.chord_length = self.chord_root

        self.S = self.aircraft_data.data['outputs']['wing_design']['S']

        self.n_stringers = self.aircraft_data.data['inputs']['structures']['wing_box']['n_stringers']
        self.stringer_area = self.aircraft_data.data['inputs']['structures']['wing_box']['stringer_area']/1000000
        self.stringer_radius = np.sqrt(self.stringer_area / np.pi)
        
        self.material = self.aircraft_data.data['inputs']['structures']['materials']['Al7075']
        self.material_stringer = self.aircraft_data.data['inputs']['structures']['materials']['Ti-6Al-4V']
        self.E_stringer = self.material_stringer['E']
        self.sigma_y_stringer = self.material_stringer['sigma_y']
        self.poisson_ratio_stringer = self.material_stringer['poisson_ratio']
        self.n_cells = self.aircraft_data.data['inputs']['structures']['wing_box']['n_cells']
        self.fuel_volume = self.aircraft_data.data['outputs']['max']['max_fuel_L']/1000

        self.C = self.aircraft_data.data['inputs']['structures']['wing_box']['C']

        self.fuel_density = 0.82 # TODO link to json data

        self.fuselage_fuel = 0 #TODO link to json etc.

        self.fuel_wing = (1- self.fuselage_fuel)/2*self.fuel_volume

        self.mid_spar_positions = [
            self.front_spar + (self.rear_spar - self.front_spar) * i / self.n_cells
            for i in range(1, self.n_cells)
        ]

        self.bottom_spar_margin = 0.05
        self.top_spar_margin = 0.98

        self.wing_mass = 30000

        self.engine_positions = self.aircraft_data.data['outputs']['engine_positions']['y_engines']
        self.engine_weight = self.aircraft_data.data['inputs']['engine']['engine_weight']
        self.I_xx_list = []

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
        def get_y_top(x, segments, margin):
            for segment in segments:
                if segment['x_start'] <= x <= segment['x_end']:
                    return margin * (segment['slope'] * x + segment['intercept'])
            raise ValueError(f"x={x} not in any segment")

        def get_y_bottom(x, segments, margin):
            for segment in segments:
                if segment['x_start'] <= x <= segment['x_end']:
                    return (segment['slope'] * x + segment['intercept']) + margin
            raise ValueError(f"x={x} not in any segment")

        spar_positions = [self.front_spar]
        if self.n_cells > 1:
            spar_positions.extend(self.mid_spar_positions)
        spar_positions.append(self.rear_spar)

        spar_xs = [pos * self.chord_length for pos in spar_positions]

        spar_data = {}
        spar_data['max_y'] = []
        spar_data['min_y'] = []
        spar_data['spar_heights'] = []
        spar_data['spar_heights_t'] = []
        for i, x in enumerate(spar_xs):
            spar_top = get_y_top(x, self.element_functions['upper'], self.top_spar_margin)
            spar_bottom = get_y_bottom(x, self.element_functions['lower'], self.bottom_spar_margin)

            if i == 0:
                label = 'front_spar'
            elif i == len(spar_xs) - 1:
                label = 'rear_spar'
            else:
                label = f'mid_spar_{i}'

            spar_data[f'{label}_x'] = x
            spar_data[f'{label}_top'] = spar_top
            spar_data[f'{label}_bottom'] = spar_bottom
            spar_data[f'{label}_height'] = spar_top - spar_bottom
            spar_data['max_y'].append(spar_top)
            spar_data['min_y'].append(spar_bottom)
            spar_data['spar_heights'].append(spar_top - spar_bottom)
            spar_data['spar_heights_t'].append((spar_top - spar_bottom)*self.t_spar)

        spar_data['max_y'] = max(spar_data['max_y'])
        spar_data['max_x'] = spar_positions[np.argmax(spar_data['max_y'])]
        spar_data['min_y'] = min(spar_data['min_y'])
        spar_data['min_x'] = spar_positions[np.argmin(spar_data['min_y'])]
        spar_data['spar_heights'] = np.array(spar_data['spar_heights'])
        spar_data['spar_heights_t'] = np.array(spar_data['spar_heights_t'])
        self.spar_info = spar_data


    def get_wingbox_panels(self):
        spar_info = self.spar_info

        spar_labels = [self.get_spar_label(i) for i in range(self.n_cells + 1)]
        spar_xs = [spar_info[f"{label}_x"] for label in spar_labels]

        panel_info = {}
        top_skin_length = []
        bottom_skin_length = []
        for i in range(self.n_cells):
            label_1 = spar_labels[i]
            label_2 = spar_labels[i + 1]
            x1 = spar_xs[i]
            x2 = spar_xs[i + 1]

            top_length = np.sqrt((spar_info[f'{label_1}_top'] - spar_info[f'{label_2}_top'])**2 + (x1 - x2)**2)
            bottom_length = np.sqrt((spar_info[f'{label_1}_bottom'] - spar_info[f'{label_2}_bottom'])**2 + (x1 - x2)**2)

            top_angle = np.arctan2(spar_info[f'{label_1}_top'] - spar_info[f'{label_2}_top'], x1 - x2)
            bottom_angle = np.arctan2(spar_info[f'{label_1}_bottom'] - spar_info[f'{label_2}_bottom'], x1 - x2)
            
            panel_info[f'top_panel_length_{i+1}'] = top_length
            panel_info[f'bottom_panel_length_{i+1}'] = bottom_length
            panel_info[f'top_panel_angle_{i+1}'] = top_angle
            panel_info[f'bottom_panel_angle_{i+1}'] = bottom_angle
            top_skin_length.append(top_length)
            bottom_skin_length.append(bottom_length)

        panel_info['top_skin_length'] = np.array(top_skin_length)
        panel_info['bottom_skin_length'] = np.array(bottom_skin_length)
        panel_info['widths_top'] = np.array([panel_info[f'top_panel_length_{i+1}'] for i in range(self.n_cells)])
        panel_info['widths_bottom'] = np.array([panel_info[f'bottom_panel_length_{i+1}'] for i in range(self.n_cells)])
        self.panel_info = panel_info

    def compute_wing_box_areas(self):
        if not hasattr(self, 'spar_info'):
            self.get_spar_heights()
        if not hasattr(self, 'panel_info'):
            self.get_wingbox_panels()

        spar_labels = [self.get_spar_label(i) for i in range(self.n_cells + 1)]

        total_area = 0
        Cx_total = 0
        Cy_total = 0
        self.cell_areas = []
        for i in range(self.n_cells):
            # Get spar endpoints
            label_1 = spar_labels[i]
            label_2 = spar_labels[i + 1]

            x1 = self.spar_info[f'{label_1}_x']
            x2 = self.spar_info[f'{label_2}_x']
            y1_top = self.spar_info[f'{label_1}_top']
            y2_top = self.spar_info[f'{label_2}_top']
            y1_bot = self.spar_info[f'{label_1}_bottom']
            y2_bot = self.spar_info[f'{label_2}_bottom']

            x_coords = [x1, x2, x2, x1]
            y_coords = [y1_top, y2_top, y2_bot, y1_bot]

            n = len(x_coords)
            A = 0
            Cx = 0
            Cy = 0
            for j in range(n):
                x0, y0 = x_coords[j], y_coords[j]
                x1_, y1_ = x_coords[(j + 1) % n], y_coords[(j + 1) % n]
                cross = x0 * y1_ - x1_ * y0
                A += cross
                Cx += (x0 + x1_) * cross
                Cy += (y0 + y1_) * cross

            A *= 0.5
            if A == 0:
                continue 
            Cx /= (6 * A)
            Cy /= (6 * A)

            self.cell_areas.append(abs(A))
            total_area += abs(A)
            Cx_total += Cx * abs(A)
            Cy_total += Cy * abs(A)

        if total_area == 0:
            raise ValueError("Degenerate wingbox: total area is zero.")

        #self.centroid = (Cx_total / total_area, Cy_total / total_area)
        self.wingbox_area = total_area

    def compute_centroid(self):

        x_coords = np.concatenate([self.x_array, self.x_array[::-1]])
        y_coords = np.concatenate([self.y_upper, self.y_lower[::-1]])

        A = 0 
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


    def get_spar_label(self, i):
        if i == 0:
            return 'front_spar'
        elif i == self.n_cells:
            return 'rear_spar'
        else:
            return f'mid_spar_{i}'

    def integral(self, x1, x2, intersect, slope):

        def integrand(x):
            return slope * x + intersect

        area, error = quad(integrand, x1, x2)
        return area


    def beam_standard_Ixx(self, length, thickness, distance, angle):
        A_web = length * thickness
        I_web = (thickness*length**3*np.sin(angle)**2) / 12
        I_xx = I_web + A_web * distance ** 2

        return I_xx
    
    def beam_standard_Iyy(self, length, thickness, distance, angle):
        A_web = length * thickness
        I_web = (thickness*length**3*np.cos(angle)**2) / 12
        I_yy = I_web + A_web * distance ** 2

        return I_yy
    
    def beam_standard_Ixy(self, length, thickness, xy, angle):

        A_web = length * thickness
        I_web = (thickness*length**3*np.sin(angle)*np.cos(angle)) / 12

        I_xy = I_web + A_web * xy

        return I_xy

    def get_polar_moment(self):

        polar_moment_wingbox = 0
        for i in range(1, self.n_cells + 1):
            top_panel_length = self.panel_info[f'top_panel_length_{i}']
            bottom_panel_length = self.panel_info[f'bottom_panel_length_{i}']
            cell_area = self.cell_areas[i - 1]

            left_spar_height = self.spar_info[f'{self.get_spar_label(i - 1)}_height']
            right_spar_height = self.spar_info[f'{self.get_spar_label(i)}_height']

            length_thickness_cell = (top_panel_length + bottom_panel_length) / self.t_skin + \
                (left_spar_height + right_spar_height) / self.t_wing
            
            polar_moment_wingbox += 4*cell_area**2 / length_thickness_cell

        self.J = polar_moment_wingbox

    def get_effective_sheet_width(self):
        
        self.stringer_width = self.t_skin*np.sqrt(self.C*np.pi**2/(12*(1-self.poisson_ratio_stringer**2)))*np.sqrt(self.E_stringer/self.sigma_y_stringer)
        return self.stringer_width
    
    def calculate_stringer_thickness(self):
        """Using an I shaped stringer, with fixed length ratios and constant thickness, a relationship is derived to calculate the stringer thickness."""
        A = self.stringer_area * 1000000  # Convert to mm^2
        L_stringer = 100 # mm

        D = 10.5625*(L_stringer**2) - (4*2*A)

        t_stringer1 = ((3.25*L_stringer) + np.sqrt(D)) / 4
        t_stringer2 = ((3.25*L_stringer) - np.sqrt(D)) / 4

        return L_stringer, min(t_stringer1, t_stringer2) 


    def crippling_stress_stringer(self):
        alpha = 0.8
        n = 0.6
        L_stringer0 = self.calculate_stringer_thickness()[0] # mm
        t_stringer0 = self.calculate_stringer_thickness()[1] # mm
        C_1234 = 0.425
        C_5 = 4.0
        crippling_yield_ratio_list = []
        crippling_stress_list = []
        areas_list = []

        for i in range(5):
            C = C_1234 if i < 4 else C_5
            if i < 2:
                L_stringer = 0.5*L_stringer0 - 0.5*t_stringer0
                t_stringer = t_stringer0 
            elif i < 4 and i >= 2:
                L_stringer = 0.75*L_stringer0 - 0.5*t_stringer0 
                t_stringer = t_stringer0
            else:
                L_stringer = t_stringer0
                t_stringer = 0.75*L_stringer0

            sigma_cc_sigma_y = alpha * ((C / self.sigma_y_stringer) * ((np.pi**2 * self.E_stringer) / (12 * (1 - self.poisson_ratio_stringer**2)))*(((t_stringer / L_stringer)**2)))**(1 - n)
            crippling_yield_ratio_list.append(sigma_cc_sigma_y)

        for i in crippling_yield_ratio_list:
            if i > 1.0:
                crippling_stress_list.append(self.sigma_y_stringer)
            else: 
                crippling_stress_list.append(i * self.sigma_y_stringer)

        for i in range(5):
            if i < 2:
                A = (0.5*L_stringer0 - 0.5*t_stringer0) * t_stringer0
                areas_list.append(A)
            elif i < 4 and i >= 2:
                A = (0.75*L_stringer0 - 0.5*t_stringer0) * t_stringer0
                areas_list.append(A)
            else:
                A = t_stringer0 * (0.75*L_stringer0)
                areas_list.append(A)

        areas_array = np.array(areas_list)
        crippling_stress_array = np.array(crippling_stress_list)

        crippling_stress_stringer = (np.sum(crippling_stress_array * areas_array) / np.sum(areas_array)) / 1000000 # Convert to MPa

        return crippling_stress_stringer
    
    def get_stringer_placement(self):
        spar_info = self.spar_info
        n_stringers = self.n_stringers
        r = self.stringer_radius
        n_cells = self.n_cells
        panel_info = self.panel_info
        clearance = 0.05
        min_width = 0.1
        self.stringer_width = self.get_effective_sheet_width()

        stringer_info = {
            'top': {'x': [], 'y': [], 'I_xx': 0, 'I_yy': 0, 'I_xy': 0},
            'bottom': {'x': [], 'y': [], 'I_xx': 0, 'I_yy': 0, 'I_xy': 0}
        }

        if n_stringers == 0:
            self.stringer_dict = stringer_info
            return

        spar_x_positions = [spar_info[f'{self.get_spar_label(i)}_x'] for i in range(n_cells + 1)]
        spar_tops = [spar_info[f'{self.get_spar_label(i)}_top'] for i in range(n_cells + 1)]
        spar_bottoms = [spar_info[f'{self.get_spar_label(i)}_bottom'] for i in range(n_cells + 1)]
        top_panel_angles = [panel_info[f'top_panel_angle_{i}'] for i in range(1, n_cells + 1)]
        bottom_panel_angles = [panel_info[f'bottom_panel_angle_{i}'] for i in range(1, n_cells + 1)]

        top_stringer_count = (n_stringers + 1) // 2
        bottom_stringer_count = n_stringers // 2

        top_stringers_per_cell = [top_stringer_count // n_cells] * n_cells
        for i in range(top_stringer_count % n_cells):
            top_stringers_per_cell[i] += 1

        bottom_stringers_per_cell = [bottom_stringer_count // n_cells] * n_cells
        for i in range(bottom_stringer_count % n_cells):
            bottom_stringers_per_cell[i] += 1

        for i in range(n_cells):
            left_x = spar_x_positions[i]
            right_x = spar_x_positions[i + 1]
            total_length = right_x - left_x
            available_length = total_length - 2 * (r + clearance)

            # --- Top ---
            current_top_width = panel_info['widths_top'][i]
            max_top_stringers = int((current_top_width - min_width) // self.stringer_width)
            n_top = min(top_stringers_per_cell[i], max_top_stringers)

            if n_top > 0:
                if n_top == 1:
                    x_positions = np.array([left_x + r + clearance + available_length / 2])
                elif n_top == 2:
                    x_positions = np.linspace(left_x + r + clearance, right_x - r - clearance, 4)[1:-1]
                else:
                    spacing = available_length / (n_top - 1)
                    x_positions = np.array([left_x + r + clearance + j * spacing for j in range(n_top)])
                y_positions = np.tan(top_panel_angles[i]) * (x_positions - left_x) + spar_tops[i] - r
                stringer_info['top']['x'].extend(x_positions)
                stringer_info['top']['y'].extend(y_positions)
                dx = x_positions - self.centroid[0]
                dy = y_positions - self.centroid[1]
                stringer_info['top']['I_xx'] += np.sum(self.stringer_area * dy**2)
                stringer_info['top']['I_yy'] += np.sum(self.stringer_area * dx**2)
                stringer_info['top']['I_xy'] += np.sum(self.stringer_area * dx * dy)

            # --- Bottom ---
            current_bottom_width = panel_info['widths_bottom'][i]
            max_bottom_stringers = int((current_bottom_width - min_width) // self.stringer_width)
            n_bottom = min(bottom_stringers_per_cell[i], max_bottom_stringers)

            if n_bottom > 0:
                if n_bottom == 1:
                    x_positions = np.array([left_x + r + clearance + available_length / 2])
                elif n_bottom == 2:
                    x_positions = np.linspace(left_x + r + clearance, right_x - r - clearance, 4)[1:-1]
                else:
                    spacing = available_length / (n_bottom - 1)
                    x_positions = np.array([left_x + r + clearance + j * spacing for j in range(n_bottom)])
                y_positions = np.tan(bottom_panel_angles[i]) * (x_positions - left_x) + spar_bottoms[i] + r
                stringer_info['bottom']['x'].extend(x_positions)
                stringer_info['bottom']['y'].extend(y_positions)
                dx = x_positions - self.centroid[0]
                dy = y_positions - self.centroid[1]
                stringer_info['bottom']['I_xx'] += np.sum(self.stringer_area * dy**2)
                stringer_info['bottom']['I_yy'] += np.sum(self.stringer_area * dx**2)
                stringer_info['bottom']['I_xy'] += np.sum(self.stringer_area * dx * dy)

        self.stringer_dict = stringer_info

    def get_moment_of_inertia(self):
        I_xx_mid_spars = 0
        I_yy_mid_spars = 0
        I_xy_mid_spars = 0

        I_xx_panels = 0
        I_yy_panels = 0
        I_xy_panels = 0

        front_spar_x = self.spar_info['front_spar_x']
        rear_spar_x = self.spar_info['rear_spar_x']
        front_spar_height = self.spar_info['front_spar_height']
        rear_spar_height = self.spar_info['rear_spar_height']
        front_spar_top = self.spar_info['front_spar_top']
        front_spar_bottom = self.spar_info['front_spar_bottom']

        I_xx_front_spar = self.beam_standard_Ixx(front_spar_height, self.t_spar,
                                                abs(front_spar_height / 2 - self.centroid[1]), np.pi / 2)
        I_yy_front_spar = self.beam_standard_Iyy(front_spar_height, self.t_spar,
                                                abs(front_spar_x - self.centroid[0]), np.pi / 2)
        I_xy_front_spar = self.beam_standard_Ixy(front_spar_height, self.t_spar,
                                                (front_spar_x - self.centroid[0]) * (front_spar_height / 2 - self.centroid[1]),
                                                np.pi / 2)
        
        I_xx_rear_spar = self.beam_standard_Ixx(rear_spar_height, self.t_spar,
                                                abs(rear_spar_height / 2 - self.centroid[1]), np.pi / 2)
        I_yy_rear_spar = self.beam_standard_Iyy(rear_spar_height, self.t_spar,
                                                abs(rear_spar_x - self.centroid[0]), np.pi / 2)
        I_xy_rear_spar = self.beam_standard_Ixy(rear_spar_height, self.t_spar,
                                                (rear_spar_x - self.centroid[0]) * (rear_spar_height / 2 - self.centroid[1]),
                                                np.pi / 2)

        for i in range(1, self.n_cells + 1):
            top_panel_angle = self.panel_info[f'top_panel_angle_{i}']
            bottom_panel_angle = self.panel_info[f'bottom_panel_angle_{i}']

            top_panel_length = self.panel_info[f'top_panel_length_{i}']
            bottom_panel_length = self.panel_info[f'bottom_panel_length_{i}']

            left_spar_top = self.spar_info[f'{self.get_spar_label(i - 1)}_top']
            left_spar_bottom = self.spar_info[f'{self.get_spar_label(i - 1)}_bottom']

            I_xx_panels += self.beam_standard_Ixx(top_panel_length, self.t_skin,
                                                abs(0.5 * top_panel_length * np.sin(top_panel_angle) + left_spar_top - self.centroid[1]),
                                                top_panel_angle)
            I_yy_panels += self.beam_standard_Iyy(top_panel_length, self.t_skin,
                                                abs(0.5 * top_panel_length * np.cos(top_panel_angle) - self.centroid[0]),
                                                top_panel_angle)
            I_xy_panels += self.beam_standard_Ixy(top_panel_length, self.t_skin,
                                                (0.5 * top_panel_length * np.cos(top_panel_angle) - self.centroid[0]) *
                                                (0.5 * top_panel_length * np.sin(top_panel_angle) + left_spar_top - self.centroid[1]),
                                                top_panel_angle)

            I_xx_panels += self.beam_standard_Ixx(bottom_panel_length, self.t_skin,
                                                abs(0.5 * bottom_panel_length * np.sin(bottom_panel_angle) + left_spar_bottom - self.centroid[1]),
                                                bottom_panel_angle)
            I_yy_panels += self.beam_standard_Iyy(bottom_panel_length, self.t_skin,
                                                abs(0.5 * bottom_panel_length * np.cos(bottom_panel_angle) - self.centroid[0]),
                                                bottom_panel_angle)
            I_xy_panels += self.beam_standard_Ixy(bottom_panel_length, self.t_skin,
                                                (0.5 * bottom_panel_length * np.cos(bottom_panel_angle) - self.centroid[0]) *
                                                (0.5 * bottom_panel_length * np.sin(bottom_panel_angle) + left_spar_bottom - self.centroid[1]),
                                                bottom_panel_angle)
            if i < self.n_cells:
                mid_spar_height = self.spar_info[f'mid_spar_{i}_height']
                mid_spar_x = self.spar_info[f'mid_spar_{i}_x']

                I_xx_mid_spars += self.beam_standard_Ixx(mid_spar_height, self.t_spar,
                                                        abs(mid_spar_height / 2 - self.centroid[1]), np.pi / 2)
                I_yy_mid_spars += self.beam_standard_Iyy(mid_spar_height, self.t_spar,
                                                        abs(mid_spar_x - self.centroid[0]), np.pi / 2)
                I_xy_mid_spars += self.beam_standard_Ixy(mid_spar_height, self.t_spar,
                                                        (mid_spar_x - self.centroid[0]) * (mid_spar_height / 2 - self.centroid[1]),
                                                        np.pi / 2)
                
                # print(mid_spar_height, self.t_spar)
                # print(self.beam_standard_Ixx(mid_spar_height, self.t_spar,
                #                                         abs(mid_spar_height / 2 - self.centroid[1]), np.pi / 2))

        I_xx_wing_top = 0
        I_yy_wing_top = 0
        I_xy_wing_top = 0
        for elem in self.element_functions['upper']:
            length = elem['length']
            x_start = elem['x_start']
            x_end = elem['x_end']

            slope = elem['slope']
            intercept = elem['intercept']
            x_centroid = (x_start + x_end) / 2
            y_centroid = slope * x_centroid + intercept

            distance_xx = y_centroid - self.centroid[1]
            distance_yy = -x_centroid - self.centroid[0]
            xy = distance_yy * distance_xx

            angle = np.arctan(slope)

            I_xx_wing_top += self.beam_standard_Ixx(length, self.t_wing, abs(distance_xx), angle)
            I_yy_wing_top += self.beam_standard_Iyy(length, self.t_wing, abs(distance_yy), angle)
            I_xy_wing_top += self.beam_standard_Ixy(length, self.t_wing, xy, angle)

        I_xx_wing_bottom = 0
        I_yy_wing_bottom = 0
        I_xy_wing_bottom = 0
        for elem in self.element_functions['lower']:
            length = elem['length']
            x_start = elem['x_start']
            x_end = elem['x_end']

            slope = elem['slope']
            intercept = elem['intercept']
            x_centroid = (x_start + x_end) / 2
            y_centroid = slope * x_centroid + intercept

            distance_xx = y_centroid - self.centroid[1]
            distance_yy = -x_centroid - self.centroid[0]
            xy = distance_yy * distance_xx

            angle = np.arctan(slope)

            I_xx_wing_bottom += self.beam_standard_Ixx(length, self.t_wing, abs(distance_xx), angle)
            I_yy_wing_bottom += self.beam_standard_Iyy(length, self.t_wing, abs(distance_yy), angle)
            I_xy_wing_bottom += self.beam_standard_Ixy(length, self.t_wing, xy, angle)

        self.I_xx = (I_xx_front_spar + I_xx_rear_spar + I_xx_panels +
                 + I_xx_wing_bottom + I_xx_wing_top + I_xx_mid_spars + self.stringer_dict['top']['I_xx'] + self.stringer_dict['bottom']['I_xx'])
        self.I_yy = (I_yy_front_spar + I_yy_rear_spar + I_yy_panels +
                    + I_yy_wing_bottom + I_yy_wing_top + I_yy_mid_spars + self.stringer_dict['top']['I_yy'] + self.stringer_dict['bottom']['I_yy'])
        self.I_xy = (I_xy_front_spar + I_xy_rear_spar + I_xy_panels +
                    + I_xy_mid_spars + self.stringer_dict['top']['I_xy'] + self.stringer_dict['bottom']['I_xy'])

        # print(I_xx_front_spar, I_xx_rear_spar, I_xx_panels, I_xx_wing_bottom, I_xx_wing_top, I_xx_mid_spars, self.stringer_dict['top']['I_xx'], self.stringer_dict['bottom']['I_xx'])
        # print(self.I_xx)


    def get_wing_structure(self):
        self.wing_structure = {}
    
        for idx, chord in enumerate(self.chord_array):
            self.chord_length = chord

            self.wing_structure[idx] = {}

            self.split_airfoil_surfaces()
            self.get_element_functions()  
            self.get_spar_heights()
            self.get_wingbox_panels()
            self.compute_wing_box_areas()
            self.compute_centroid()
            self.get_stringer_placement()
            self.get_moment_of_inertia()
            self.get_polar_moment()

            self.wing_structure[idx]['elements'] = self.element_functions
            self.wing_structure[idx]['area'] = self.wingbox_area
            self.wing_structure[idx]['cell_areas'] = self.cell_areas
            self.wing_structure[idx]['spar_info'] = self.spar_info
            self.wing_structure[idx]['centroid'] = self.centroid
            self.wing_structure[idx]['panel_info'] = self.panel_info
            self.wing_structure[idx]['chord'] = chord
            self.wing_structure[idx]['quarter_chord_dist'] = self.centroid[0] - chord / 4 
            self.wing_structure[idx]['x_coords'] = self.x_array
            self.wing_structure[idx]['y_upper'] = self.y_upper
            self.wing_structure[idx]['y_lower'] = self.y_lower
            self.wing_structure[idx]['I_xx'] = self.I_xx
            self.wing_structure[idx]['I_yy'] = self.I_yy
            self.wing_structure[idx]['I_xy'] = self.I_xy
            self.wing_structure[idx]['J'] = self.J
            self.wing_structure[idx]['stringers'] = self.stringer_dict
        
        root_chord_data = self.wing_structure[0]

        self.normalized_data = {}
        self.normalized_data['spar_heights'] = root_chord_data['spar_info']['spar_heights']/self.chord_root
        self.normalized_data['top_skin_lengths'] = root_chord_data['panel_info']['top_skin_length']/self.chord_root
        self.normalized_data['bottom_skin_lengths'] = root_chord_data['panel_info']['bottom_skin_length']/self.chord_root


        # self.plot_moment_of_inertia()
        # self.plot_polar_moment()
        # self.plot_wing_weight()

    def get_fuel_length(self, slope, intercept):
        a = slope/2
        b = intercept
        c = -self.fuel_wing

        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            raise ValueError("No real roots, check the slope and intercept values.")
        sqrt_discriminant = np.sqrt(discriminant)
        root1 = (-b + sqrt_discriminant) / (2 * a)
        root2 = (-b - sqrt_discriminant) / (2 * a)
        if root1 < 0 and root2 < 0:
            raise ValueError("Both roots are negative, check the slope and intercept values.")
        root = min(round(root1,2), round(root2,2))
        if root <= 0:
            return round(root2, 2)
        else:
            return root
        
    def get_fuel_mass_distribution(self):
        area1 = 0.75*sum(self.wing_structure[0]['cell_areas'][i] for i in range(self.n_cells))
        area2 = 0.75*sum(self.wing_structure[self.chord_array.shape[0]-1]['cell_areas'][i] for i in range(self.n_cells))
        area_slope = (area2 - area1) / (self.b / 2)
        area_intercept = area1

        fuel_length = self.get_fuel_length(area_slope, area_intercept)

        self.fuel_mass_distribution = np.zeros_like(self.b_array)

        for i, y in enumerate(self.b_array):
            if y <= fuel_length:
                area_y = area_intercept + area_slope * y
                chord_y = self.chord_span_function(y)
                self.fuel_mass_distribution[i] = (
                    self.fuel_density * self.fuel_wing*1000 / self.S * chord_y * 9.81
                )
        return self.fuel_mass_distribution

    def gaussian_peak(self,x, x0, A, sigma=0.1):
        return A * np.exp(-((x - x0)**2) / (2 * sigma**2)) / (sigma * np.sqrt(2 * np.pi))
    
    def wing_weight_dist(self):
        self.weight_dist = self.wing_mass/self.S*self.chord_span_function(self.b_array)*9.81 + self.get_fuel_mass_distribution()
        for engine_y in self.engine_positions:
            self.weight_dist[int(round(engine_y,2)*100)] += self.engine_weight*9.81

        return self.weight_dist
                                         

    def plot_wing_weight(self):
        self.wing_weight_dist()
        fig, ax = plt.subplots()
        ax.plot(self.b_array, self.weight_dist, label='Weight Distribution', color='blue')
        ax.set_xlabel('Chord Length (m)')
        ax.set_ylabel('Weight (N)')
        ax.set_title('Weight Distribution vs Chord Length')
        ax.legend()
        ax.grid()
        plt.show()

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

    def plot_polar_moment(self):
        fig, ax = plt.subplots()

        ax.plot(self.b_array, [wing['J'] for wing in self.wing_structure.values()], label='Polar Moment of Inertia', color='purple')
        ax.set_xlabel('Chord Length (m)')
        ax.set_ylabel('Polar Moment of Inertia (m^4)')
        ax.set_title('Polar Moment of Inertia vs Chord Length')
        ax.legend()
        ax.grid()
        plt.show()

    
    def plot_airfoil(self, chord_idx=0):
        wing_data = self.wing_structure[chord_idx]
        
        spar_info = wing_data['spar_info']

        x_array = wing_data['x_coords']
        y_upper = wing_data['y_upper']
        y_lower = wing_data['y_lower']
        centroid = wing_data['centroid']
        area = wing_data['area']
        cell_areas = wing_data['cell_areas']
        chord = self.chord_array[chord_idx]

        plt.figure(figsize=(10, 5))
        plt.plot(x_array, y_upper, label='Airfoil Profile', color='blue')
        plt.plot(x_array, y_lower, color='blue')

        spar_x_positions = []
        spar_tops = []
        spar_bottoms = []

        front_x = self.front_spar * chord
        spar_x_positions.append(front_x)
        spar_tops.append(spar_info['front_spar_top'])
        spar_bottoms.append(spar_info['front_spar_bottom'])

        for i in range(1, self.n_cells):
            mid_x = self.mid_spar_positions[i - 1] * chord
            spar_x_positions.append(mid_x)
            spar_tops.append(spar_info[f'mid_spar_{i}_top'])
            spar_bottoms.append(spar_info[f'mid_spar_{i}_bottom'])

        rear_x = self.rear_spar * chord
        spar_x_positions.append(rear_x)
        spar_tops.append(spar_info['rear_spar_top'])
        spar_bottoms.append(spar_info['rear_spar_bottom'])

        for i, (x, top, bottom) in enumerate(zip(spar_x_positions, spar_tops, spar_bottoms)):
            label = None
            if i == 0:
                label = 'Front Spar'
            elif i == len(spar_x_positions) - 1:
                label = 'Rear Spar'
            else:
                label = f'Mid Spar {i}'
            plt.plot([x, x], [bottom, top], color='red', label=label)

        for i in range(len(spar_x_positions) - 1):
            x_left = spar_x_positions[i]
            x_right = spar_x_positions[i + 1]
            top_left = spar_tops[i]
            top_right = spar_tops[i + 1]
            bottom_left = spar_bottoms[i]
            bottom_right = spar_bottoms[i + 1]

            plt.plot([x_left, x_right], [top_left, top_right], color='red')
            plt.plot([x_left, x_right], [bottom_left, bottom_right], color='red')
            plt.fill_between([x_left, x_right], [bottom_left, bottom_right], [top_left, top_right],
                            color='gray', alpha=0.5, label='Wing Box' if i == 0 else None)

        stringers = self.wing_structure[chord_idx]['stringers']

        stringer_xs_top = stringers['top']['x']
        for x,y in zip(stringers['top']['x'], stringers['top']['y']):
            circle = plt.Circle((x,y), self.stringer_radius, color='black', label='Stringer (Top)')
            plt.gca().add_patch(circle)

        for x, y in zip(stringers['bottom']['x'], stringers['bottom']['y']):
            circle = plt.Circle((x, y), self.stringer_radius, color='purple', label='Stringer (Bottom)')
            plt.gca().add_patch(circle)

        plt.scatter(centroid[0], centroid[1], color='green', label='Centroid')
        plt.title(f"Airfoil with Spar Positions, Wing Box and Stringers, chord = {chord:.2f} m")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    airfoil_data = Data("airfoil_data.dat","airfoil_geometry")
    aircraft_data = Data("design3.json")

    wing_structure = WingStructure(aircraft_data, airfoil_data)
    wing_structure.get_wing_structure()
    #3187
    wing_structure.plot_airfoil(chord_idx=0)
    wing_structure.plot_moment_of_inertia()
