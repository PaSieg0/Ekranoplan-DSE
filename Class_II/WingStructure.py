import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from scipy.integrate import quad

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, Materials, EvaluateType

class WingStructure:
    def __init__(self, aircraft_data: Data, wingbox_mat: Materials, 
                 stringer_mat: Materials, wing_mat: Materials, evaluate: EvaluateType):
        self.aircraft_data = aircraft_data
        self.evaluate = evaluate
        self.front_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['rear_spar']

        if self.evaluate == EvaluateType.WING:
            self.airfoil_data = Data('Airfoil_data.dat', 'airfoil_geometry')
            self.data = self.airfoil_data.data
            self.b = self.aircraft_data.data['outputs']['wing_design']['b']
            self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
            self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']
            self.b_array = np.arange(0, self.b/2+0.01, 0.01)
            self.chord_array = self.chord_span_function(self.b_array)
            self.chord_length = self.chord_root
            self.S = self.aircraft_data.data['outputs']['wing_design']['S']
            self.min_skin_thickness = self.aircraft_data.data['inputs']['structures']['wing_box']['min_skin_thickness']/1000
            self.min_spar_thickness = self.aircraft_data.data['inputs']['structures']['wing_box']['min_spar_thickness']/1000
            self.thickness_threshold = int(self.aircraft_data.data['inputs']['structures']['wing_box']['thickness_threshold']*self.b/2*100)
            self.n_cells = self.aircraft_data.data['inputs']['structures']['wing_box']['n_cells']
            self.fuel_volume = self.aircraft_data.data['outputs']['max']['max_fuel_L']/1000
            self.flap_start = self.aircraft_data.data['outputs']['HLD']['b1']
            self.flap_end = self.aircraft_data.data['outputs']['HLD']['b2']


            self.t_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['t_spar']/1000
            self.t_skin = self.aircraft_data.data['inputs']['structures']['wing_box']['t_skin']/1000
            self.t_wing = self.aircraft_data.data['inputs']['structures']['wing_box']['t_wing']/1000

            self.aileron_start = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b1']
            self.aileron_end = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b2']

            self.L_stringer = self.aircraft_data.data['inputs']['structures']['wing_box']['L_stringer']
            self.stringer_area = self.aircraft_data.data['inputs']['structures']['wing_box']['stringer_area']/1000000

            self.C = self.aircraft_data.data['inputs']['structures']['wing_box']['C']

            self.n_stringers = self.aircraft_data.data['inputs']['structures']['wing_box']['n_stringers']

            self.fuel_tank = self.aircraft_data.data['inputs']['fuel_tank']
            self.buoy_mass = 950 #TODO link to json
            self.fuselage_fuel = self.aircraft_data.data['inputs']['fuel_fuselage']
            self.cutout_spacing = self.aircraft_data.data['inputs']['structures']['wing_box']['cutout_spacing']

            self.fuel_wing = (1- self.fuselage_fuel)/2*self.fuel_volume
            self.engine_positions = self.aircraft_data.data['outputs']['engine_positions']['y_engines']
            self.engine_weight = self.aircraft_data.data['inputs']['engine']['engine_weight']
            self.fuel_tank_thickness = self.aircraft_data.data['inputs']['fuel_tank_thickness']/1000
            self.bottom_spar_margin = 0.05
            self.top_spar_margin = 0.98

        elif self.evaluate == EvaluateType.HORIZONTAL:
            self.airfoil_data = Data('VerticalTailfoil.dat', 'airfoil_geometry')
            self.data = self.airfoil_data.data
            self.b = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
            self.chord_root = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
            self.chord_tip = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip']
            self.b_array = np.arange(0, self.b/2+0.01, 0.01)
            self.chord_array = self.chord_span_function(self.b_array)
            self.chord_length = self.chord_root
            self.S = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']

            self.t_spar = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['t_spar']/1000
            self.t_skin = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['t_skin']/1000
            self.t_wing = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['t_wing']/1000
            self.fuel_tank = 0
            self.buoy_mass = 0
            self.fuel_wing = 0
            self.engine_positions = None
            self.cutout_spacing = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['cutout_spacing']
            self.bottom_spar_margin = 0.05
            self.top_spar_margin = 0.98
            self.min_skin_thickness = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['min_skin_thickness']/1000
            self.min_spar_thickness = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['min_spar_thickness']/1000
            self.thickness_threshold = int(self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['thickness_threshold']*self.b/2*100)
            self.n_cells = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['n_cells']

            self.L_stringer = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['L_stringer']
            self.stringer_area = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['stringer_area']/1000000

            self.C = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['C']

            self.n_stringers = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['n_stringers']

            self.elevator_start = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1']
            self.elevator_end = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2']

        elif self.evaluate == EvaluateType.VERTICAL:
            self.airfoil_data = Data('VerticalTailfoil.dat', 'airfoil_geometry')
            self.data = self.airfoil_data.data
            self.b = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
            self.chord_root = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
            self.chord_tip = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
            self.b_array = np.arange(0, self.b+0.01, 0.01)
            self.chord_array = self.chord_span_function(self.b_array)

            self.chord_length = self.chord_root
            self.S = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S']
            self.fuel_tank = 0
            self.buoy_mass = 0
            self.fuel_wing = 0
                
            self.t_spar = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['t_spar']/1000
            self.t_skin = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['t_skin']/1000
            self.t_wing = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['t_wing']/1000
            self.engine_positions = None
            self.cutout_spacing = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['cutout_spacing']
            self.bottom_spar_margin = 0.02
            self.top_spar_margin = 0.98
            self.min_skin_thickness = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['min_skin_thickness']/1000
            self.min_spar_thickness = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['min_spar_thickness']/1000
            self.thickness_threshold = int(self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['thickness_threshold']*self.b/2*100)
            self.n_cells = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['n_cells']

            self.L_stringer = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['L_stringer']
            self.stringer_area = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['stringer_area']/1000000

            self.C = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['C']

            self.n_stringers = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['n_stringers']

            self.rudder_start = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b1']
            self.rudder_end = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b2']

        self.material = self.aircraft_data.data['inputs']['structures']['materials'][wingbox_mat.name.lower()]
        self.G = self.material['G']
        self.E = self.material['E']
        self.sigma_y = self.material['sigma_y']
        self.poisson_ratio = self.material['poisson_ratio']

        self.rho_wingbox = self.material['rho']
        self.material_stringer = self.aircraft_data.data['inputs']['structures']['materials'][stringer_mat.name.lower()]
        self.E_stringer = self.material_stringer['E']
        self.sigma_y_stringer = self.material_stringer['sigma_y']
        self.rho_stringer = self.material_stringer['rho']

        self.y = 0
        self.idx = 0
        self.poisson_ratio_stringer = self.material_stringer['poisson_ratio']

        self.fuel_density = self.aircraft_data.data['inputs']['rho_fuel']

        self.wing_material = self.aircraft_data.data['inputs']['structures']['materials'][wing_mat.name.lower()]
        self.rho_wing = self.wing_material['rho']

        self.w_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        self.fuel_start = self.w_fuselage/2 + 0.3

        self.leading_edge = self.chord_root - np.tan(np.deg2rad(self.aircraft_data.data['outputs']['wing_design']['sweep_x_c'])) * self.b_array

        self.mid_spar_positions = [
            self.front_spar + (self.rear_spar - self.front_spar) * i / self.n_cells
            for i in range(1, self.n_cells)
        ]

        self.I_xx_list = []
        print(self.evaluate)

    def chord_span_function(self,y):
        if self.evaluate == EvaluateType.VERTICAL:
            return self.chord_root + (self.chord_tip - self.chord_root) / (self.b) * y
        return self.chord_root + (self.chord_tip - self.chord_root) / (self.b/2) * y
    
    def smoothstep(self,x, edge0, edge1):
        x = np.clip((x - edge0) / (edge1 - edge0), 0, 1)
        return x * x * (3 - 2 * x)
    
    def spar_thickness_span_function(self, y):
        # Compute base and tip values
        t_root = self.t_spar
        t_tip = self.min_spar_thickness
        
        # Define where smoothing starts and ends
        y_start = self.thickness_threshold / (len(self.b_array)-1) * (self.b / 2)
        y_end = self.b / 2

        # Interpolation factor
        t = self.smoothstep(y, y_start, y_end)
        
        # Linear interpolation with smoothing
        return (1 - t) * t_root + t * t_tip


    def skin_thickness_span_function(self, y):
        t_root = self.t_skin
        t_tip = self.min_skin_thickness

        y_start = self.thickness_threshold / (len(self.b_array)-1) * (self.b / 2)
        y_end = self.b / 2

        t = self.smoothstep(y, y_start, y_end)
        
        return (1 - t) * t_root + t * t_tip

            
    def draw_I_beam(self, x, y, L, thickness, color, top=True, angle=0):
        if top:
            top_w = L
            bottom_w = 0.75 * L
        else:
            top_w = 0.75 * L
            bottom_w = L

        web_h = 0.75 * L

        # Web centered at (x, y)
        top_y = y + web_h / 2
        bottom_y = y - web_h / 2

        # Create the rectangles (unrotated)
        top_flange = patches.Rectangle(
            (x - top_w / 2, top_y - thickness),
            top_w,
            thickness*5,
            facecolor=color
        )

        bottom_flange = patches.Rectangle(
            (x - bottom_w / 2, bottom_y),
            bottom_w,
            thickness*5,
            facecolor=color
        )

        web = patches.Rectangle(
            (x - thickness / 2, bottom_y + thickness),
            thickness*5,
            web_h - 2 * thickness,
            facecolor=color
        )

        # Create a transform that rotates around the center (x, y)
        trans = transforms.Affine2D().rotate_deg_around(x, y, np.rad2deg(angle)) + plt.gca().transData

        # Apply transform to each part
        for part in [top_flange, bottom_flange, web]:
            part.set_transform(trans)
            plt.gca().add_patch(part)

    def split_symmetric_airfoil(self):
        x = np.array(self.data['x']) * self.chord_length
        y = np.array(self.data['y']) * self.chord_length
        midpoint = len(x) // 2
        if not np.allclose(x[:midpoint], x[midpoint:]):
            raise ValueError("Airfoil data is not in symmetric format (0→1, 0→1)")

        x_surf = x[:midpoint]
        y_upper = y[:midpoint]
        y_lower = y[midpoint:]

        # Common x array (from 0 to chord)
        self.x_array = x_surf
        self.y_upper = y_upper
        self.y_lower = y_lower

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
            spar_data['spar_heights_t'].append((spar_top - spar_bottom)*self.spar_thickness_span_function(self.y))

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

    def compute_total_centroid(self):
        if not hasattr(self, 'spar_info'):
            self.get_spar_heights()
        if not hasattr(self, 'panel_info'):
            self.get_wingbox_panels()

        spar_labels = [self.get_spar_label(i) for i in range(self.n_cells + 1)]

        wingbox_area = 0
        Cx_wingbox_total = 0
        Cy_wingbox_total = 0
        self.cell_areas = []

        for i in range(self.n_cells):
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

            A_abs = abs(A)
            self.cell_areas.append(A_abs)
            wingbox_area += A_abs
            Cx_wingbox_total += Cx * A_abs
            Cy_wingbox_total += Cy * A_abs

        if wingbox_area == 0:
            raise ValueError("Degenerate wingbox: total area is zero.")

        # Compute airfoil area and centroid
        x_coords_airfoil = np.concatenate([self.x_array, self.x_array[::-1]])
        y_coords_airfoil = np.concatenate([self.y_upper, self.y_lower[::-1]])

        A_airfoil = 0
        Cx_airfoil = 0
        Cy_airfoil = 0
        n = len(x_coords_airfoil)

        for i in range(n):
            x0, y0 = x_coords_airfoil[i], y_coords_airfoil[i]
            x1, y1 = x_coords_airfoil[(i + 1) % n], y_coords_airfoil[(i + 1) % n]
            cross = x0 * y1 - x1 * y0
            A_airfoil += cross
            Cx_airfoil += (x0 + x1) * cross
            Cy_airfoil += (y0 + y1) * cross

        A_airfoil *= 0.5
        if A_airfoil == 0:
            raise ValueError("Degenerate airfoil shape: area is zero.")

        Cx_airfoil /= (6 * A_airfoil)
        Cy_airfoil /= (6 * A_airfoil)
        A_airfoil_abs = abs(A_airfoil)

        # Total area and centroid
        total_area = wingbox_area + A_airfoil_abs
        Cx_total = (Cx_wingbox_total + Cx_airfoil * A_airfoil_abs) / total_area
        Cy_total = (Cy_wingbox_total + Cy_airfoil * A_airfoil_abs) / total_area

        self.wingbox_area = wingbox_area
        self.airfoil_area = A_airfoil_abs
        self.total_area = total_area
        self.centroid = (Cx_total, Cy_total)



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
        
        self.stringer_width = self.skin_thickness_span_function(self.y)*np.sqrt(self.C*np.pi**2/(12*(1-self.poisson_ratio_stringer**2)))*np.sqrt(self.E_stringer/self.cr_stringer)
        return self.stringer_width
    
    def calculate_stringer_thickness(self):
        A = self.stringer_area * 1000000  # Convert to mm^2
        L_stringer = self.L_stringer

        D = 10.5625*(L_stringer**2) - (4*2*A)

        t_stringer1 = ((3.25*L_stringer) + np.sqrt(D)) / 4
        t_stringer2 = ((3.25*L_stringer) - np.sqrt(D)) / 4

        self.t_stringer = min(t_stringer1, t_stringer2)
        return self.t_stringer
    
    def crippling_stress_stringer(self):
        alpha = 0.8
        n = 0.6
        t0 = self.calculate_stringer_thickness()  # mm
        L0 = self.L_stringer  # mm
        C_values = [0.425, 0.425, 0.425, 0.425, 4.0]
        
        E = self.E_stringer
        nu = self.poisson_ratio_stringer
        sigma_y = self.sigma_y_stringer
        
        crippling_stress = []
        areas = []

        for i, C in enumerate(C_values):
            if i < 2:
                L = 0.5 * L0 - 0.5 * t0
                t = t0
            elif i < 4:
                L = 0.75 * L0 - 0.5 * t0
                t = t0
            else:
                L = t0
                t = 0.75 * L0

            # Crippling stress to yield ratio
            sigma_cr_sigma_y = alpha * (((C / sigma_y) * ((np.pi**2 * E) / (12 * (1 - nu**2))) * ((t / L) ** 2)) ** (1 - n))

            # Effective crippling stress
            stress = min(sigma_cr_sigma_y, 1.0) * sigma_y
            crippling_stress.append(stress)

            # Area
            A = L * t
            areas.append(A)

        # Weighted average and conversion to MPa
        crippling_stress = np.array(crippling_stress)
        areas = np.array(areas)
        stringer_cr = np.sum(crippling_stress * areas) / np.sum(areas)

        return stringer_cr
    
    def calculate_stringer_moments_inertia(self):
        L = self.L_stringer / 1000  # Convert to meters
        t = self.t_stringer/1000
        self.x_centroid_stringer = 0.5*L
        self.y_centroid_stringer = 0.3375*L

        self.I_xx_stringer = 0.3235609375*(L**3)*t # mm^4
        self.I_yy_stringer = 0.1184895833*(L**3)*t # mm^4
        self.I_xy_stringer = 0                     # mm^4

    def get_stringer_placement(self):
        spar_info = self.spar_info
        panel_info = self.panel_info
        n_stringers = self.n_stringers
        n_cells = self.n_cells
        clearance = 0.02  # reduced for tighter spacing
        min_width = 0.2
        self.stringer_width = self.get_effective_sheet_width()
        l_stringer = self.L_stringer / 1000  # horizontal length
        stringer_info = {
            'top': {'x': [], 'y': [], 'angle': [], 'spacing': [], 'n_stringers': [], 'I_xx': 0, 'I_yy': 0, 'I_xy': 0},
            'bottom': {'x': [], 'y': [], 'angle': [], 'spacing': [], 'n_stringers': [], 'I_xx': 0, 'I_yy': 0, 'I_xy': 0}
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

        any_stringers_placed = False

        for i in range(n_cells):
            left_x = spar_x_positions[i]
            right_x = spar_x_positions[i + 1]
            total_length = right_x - left_x
            usable_length = total_length - 2 * clearance

            # --- Top ---
            current_top_width = panel_info['widths_top'][i]
            max_top_stringers = int((current_top_width - min_width) // l_stringer)
            fit_top_stringers = int((usable_length + clearance) // (l_stringer + clearance))
            n_top = min(top_stringers_per_cell[i], max_top_stringers, fit_top_stringers)

            if n_top > 0:
                any_stringers_placed = True
                spacing = (usable_length - n_top * l_stringer) / (n_top - 1) if n_top > 1 else 0
                x_positions = np.array([
                    left_x + clearance + j * (l_stringer + spacing) + l_stringer / 2 for j in range(n_top)
                ])
                y_positions = np.tan(top_panel_angles[i]) * (x_positions - left_x) + spar_tops[i] - 0.375 * l_stringer
                stringer_info['top']['n_stringers'].append(n_top)
                stringer_info['top']['spacing'].append(spacing)
                stringer_info['top']['angle'].extend([top_panel_angles[i]] * n_top)
                stringer_info['top']['x'].extend(x_positions)
                stringer_info['top']['y'].extend(y_positions)
                dx = x_positions - self.centroid[0]
                dy = y_positions - self.centroid[1]
                stringer_info['top']['I_xx'] += np.sum(self.stringer_area * dy ** 2) + self.I_xx_stringer * n_top
                stringer_info['top']['I_yy'] += np.sum(self.stringer_area * dx ** 2) + self.I_yy_stringer * n_top
                stringer_info['top']['I_xy'] += np.sum(self.stringer_area * dx * dy) + self.I_xy_stringer * n_top

            # --- Bottom ---
            current_bottom_width = panel_info['widths_bottom'][i]
            max_bottom_stringers = int((current_bottom_width - min_width) // l_stringer)
            fit_bottom_stringers = int((usable_length + clearance) // (l_stringer + clearance))
            n_bottom = min(bottom_stringers_per_cell[i], max_bottom_stringers, fit_bottom_stringers)

            if n_bottom > 0:
                any_stringers_placed = True
                spacing = (usable_length - n_bottom * l_stringer) / (n_bottom - 1) if n_bottom > 1 else 0
                x_positions = np.array([
                    left_x + clearance + j * (l_stringer + spacing) + l_stringer / 2 for j in range(n_bottom)
                ])
                y_positions = np.tan(bottom_panel_angles[i]) * (x_positions - left_x) + spar_bottoms[i] + 0.375 * l_stringer
                stringer_info['bottom']['n_stringers'].append(n_bottom)
                stringer_info['bottom']['spacing'].append(spacing)
                stringer_info['bottom']['angle'].extend([bottom_panel_angles[i]] * n_bottom)
                stringer_info['bottom']['x'].extend(x_positions)
                stringer_info['bottom']['y'].extend(y_positions)
                dx = x_positions - self.centroid[0]
                dy = y_positions - self.centroid[1]
                stringer_info['bottom']['I_xx'] += np.sum(self.stringer_area * dy ** 2) + self.I_xx_stringer * n_bottom
                stringer_info['bottom']['I_yy'] += np.sum(self.stringer_area * dx ** 2) + self.I_yy_stringer * n_bottom
                stringer_info['bottom']['I_xy'] += np.sum(self.stringer_area * dx * dy) + self.I_xy_stringer * n_bottom


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

        I_xx_front_spar = self.beam_standard_Ixx(front_spar_height, self.spar_thickness_span_function(self.y),
                                                abs(front_spar_height / 2 - self.centroid[1]), np.pi / 2)
        I_yy_front_spar = self.beam_standard_Iyy(front_spar_height, self.spar_thickness_span_function(self.y),
                                                abs(front_spar_x - self.centroid[0]), np.pi / 2)
        I_xy_front_spar = self.beam_standard_Ixy(front_spar_height, self.spar_thickness_span_function(self.y),
                                                (front_spar_x - self.centroid[0]) * (front_spar_height / 2 - self.centroid[1]),
                                                np.pi / 2)
        
        I_xx_rear_spar = self.beam_standard_Ixx(rear_spar_height, self.spar_thickness_span_function(self.y),
                                                abs(rear_spar_height / 2 - self.centroid[1]), np.pi / 2)
        I_yy_rear_spar = self.beam_standard_Iyy(rear_spar_height, self.spar_thickness_span_function(self.y),
                                                abs(rear_spar_x - self.centroid[0]), np.pi / 2)
        I_xy_rear_spar = self.beam_standard_Ixy(rear_spar_height, self.spar_thickness_span_function(self.y),
                                                (rear_spar_x - self.centroid[0]) * (rear_spar_height / 2 - self.centroid[1]),
                                                np.pi / 2)

        for i in range(1, self.n_cells + 1):
            top_panel_angle = self.panel_info[f'top_panel_angle_{i}']
            bottom_panel_angle = self.panel_info[f'bottom_panel_angle_{i}']

            top_panel_length = self.panel_info[f'top_panel_length_{i}']
            bottom_panel_length = self.panel_info[f'bottom_panel_length_{i}']

            left_spar_top = self.spar_info[f'{self.get_spar_label(i - 1)}_top']
            left_spar_bottom = self.spar_info[f'{self.get_spar_label(i - 1)}_bottom']

            I_xx_panels += self.beam_standard_Ixx(top_panel_length, self.skin_thickness_span_function(self.y),
                                                abs(0.5 * top_panel_length * np.sin(top_panel_angle) + left_spar_top - self.centroid[1]),
                                                top_panel_angle)
            I_yy_panels += self.beam_standard_Iyy(top_panel_length, self.skin_thickness_span_function(self.y),
                                                abs(0.5 * top_panel_length * np.cos(top_panel_angle) - self.centroid[0]),
                                                top_panel_angle)
            I_xy_panels += self.beam_standard_Ixy(top_panel_length, self.skin_thickness_span_function(self.y),
                                                (0.5 * top_panel_length * np.cos(top_panel_angle) - self.centroid[0]) *
                                                (0.5 * top_panel_length * np.sin(top_panel_angle) + left_spar_top - self.centroid[1]),
                                                top_panel_angle)

            I_xx_panels += self.beam_standard_Ixx(bottom_panel_length, self.skin_thickness_span_function(self.y),
                                                abs(0.5 * bottom_panel_length * np.sin(bottom_panel_angle) + left_spar_bottom - self.centroid[1]),
                                                bottom_panel_angle)
            I_yy_panels += self.beam_standard_Iyy(bottom_panel_length, self.skin_thickness_span_function(self.y),
                                                abs(0.5 * bottom_panel_length * np.cos(bottom_panel_angle) - self.centroid[0]),
                                                bottom_panel_angle)
            I_xy_panels += self.beam_standard_Ixy(bottom_panel_length, self.skin_thickness_span_function(self.y),
                                                (0.5 * bottom_panel_length * np.cos(bottom_panel_angle) - self.centroid[0]) *
                                                (0.5 * bottom_panel_length * np.sin(bottom_panel_angle) + left_spar_bottom - self.centroid[1]),
                                                bottom_panel_angle)
            if i < self.n_cells:
                mid_spar_height = self.spar_info[f'mid_spar_{i}_height']
                mid_spar_x = self.spar_info[f'mid_spar_{i}_x']

                I_xx_mid_spars += self.beam_standard_Ixx(mid_spar_height, self.spar_thickness_span_function(self.y),
                                                        abs(mid_spar_height / 2 - self.centroid[1]), np.pi / 2)
                I_yy_mid_spars += self.beam_standard_Iyy(mid_spar_height, self.spar_thickness_span_function(self.y),
                                                        abs(mid_spar_x - self.centroid[0]), np.pi / 2)
                I_xy_mid_spars += self.beam_standard_Ixy(mid_spar_height, self.spar_thickness_span_function(self.y),
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

        #print(I_xx_front_spar + I_xx_rear_spar + I_xx_mid_spars)
        self.I_xx = (I_xx_front_spar + I_xx_rear_spar + I_xx_panels +
                 + I_xx_wing_bottom + I_xx_wing_top + I_xx_mid_spars + self.stringer_dict['top']['I_xx'] + self.stringer_dict['bottom']['I_xx'])
        self.I_yy = (I_yy_front_spar + I_yy_rear_spar + I_yy_panels +
                    + I_yy_wing_bottom + I_yy_wing_top + I_yy_mid_spars + self.stringer_dict['top']['I_yy'] + self.stringer_dict['bottom']['I_yy'])
        self.I_xy = (I_xy_front_spar + I_xy_rear_spar + I_xy_panels +
                    + I_xy_mid_spars + self.stringer_dict['top']['I_xy'] + self.stringer_dict['bottom']['I_xy'])

        # print(I_xx_front_spar, I_xx_rear_spar, I_xx_panels, I_xx_wing_bottom, I_xx_wing_top, I_xx_mid_spars, self.stringer_dict['top']['I_xx'], self.stringer_dict['bottom']['I_xx'])
        # print(self.I_xx)

    def standard_MMOI_beam(self, length, thickness, distance, effective_mass):

        Ip = 1/12 * effective_mass * (length**2 + thickness**2) + effective_mass * distance**2
        return Ip

    def get_mass_MOI(self):

        Ip_front_spar = self.standard_MMOI_beam(self.spar_info['front_spar_height'], self.spar_thickness_span_function(self.y),
                                                 abs(self.spar_info['front_spar_height'] / 2 - self.centroid[1]), self.rho_wingbox*self.spar_info['front_spar_height']* self.spar_thickness_span_function(self.y))
        
        Ip_rear_spar = self.standard_MMOI_beam(self.spar_info['rear_spar_height'], self.spar_thickness_span_function(self.y),
                                                 abs(self.spar_info['rear_spar_height'] / 2 - self.centroid[1]), self.rho_wingbox*self.spar_info['rear_spar_height']* self.spar_thickness_span_function(self.y))
        
        Ip_mid_spars = 0
        Ip_panels = 0
        for i in range(1, self.n_cells + 1):

            top_panel_length = self.panel_info[f'top_panel_length_{i}']
            bottom_panel_length = self.panel_info[f'bottom_panel_length_{i}']
            left_spar_top = self.spar_info[f'{self.get_spar_label(i - 1)}_top']
            left_spar_bottom = self.spar_info[f'{self.get_spar_label(i - 1)}_bottom']

            # Euclidean distance from centroid to panel centroid (top)
            top_panel_angle = self.panel_info[f'top_panel_angle_{i}']
            top_panel_centroid_x = 0.5 * top_panel_length * np.cos(top_panel_angle)
            top_panel_centroid_y = 0.5 * top_panel_length * np.sin(top_panel_angle) + left_spar_top
            d_top = np.sqrt((top_panel_centroid_x - self.centroid[0])**2 + (top_panel_centroid_y - self.centroid[1])**2)

            # Euclidean distance from centroid to panel centroid (bottom)
            bottom_panel_angle = self.panel_info[f'bottom_panel_angle_{i}']
            bottom_panel_centroid_x = 0.5 * bottom_panel_length * np.cos(bottom_panel_angle)
            bottom_panel_centroid_y = 0.5 * bottom_panel_length * np.sin(bottom_panel_angle) + left_spar_bottom
            d_bottom = np.sqrt((bottom_panel_centroid_x - self.centroid[0])**2 + (bottom_panel_centroid_y - self.centroid[1])**2)

            Ip_panels += self.standard_MMOI_beam(top_panel_length, self.skin_thickness_span_function(self.y),
                             d_top,
                             self.rho_wingbox * top_panel_length * self.skin_thickness_span_function(self.y))
            Ip_panels += self.standard_MMOI_beam(bottom_panel_length, self.skin_thickness_span_function(self.y),
                             d_bottom,
                             self.rho_wingbox * bottom_panel_length * self.skin_thickness_span_function(self.y))

            if i < self.n_cells:
                mid_spar_height = self.spar_info[f'mid_spar_{i}_height']
                mid_spar_x = self.spar_info[f'mid_spar_{i}_x']
                # Euclidean distance from centroid to mid spar centroid
                mid_spar_centroid_x = mid_spar_x
                mid_spar_centroid_y = mid_spar_height / 2
                d_mid = np.sqrt((mid_spar_centroid_x - self.centroid[0])**2 + (mid_spar_centroid_y - self.centroid[1])**2)

                Ip_mid_spars += self.standard_MMOI_beam(mid_spar_height, self.spar_thickness_span_function(self.y),
                                    d_mid,
                                    self.rho_wingbox * mid_spar_height * self.spar_thickness_span_function(self.y))

    def get_wing_rib(self, ribs: dict = None):  

        available_length = self.chord_length - 2 * self.cutout_spacing

        self.cutout_amount = int(available_length // self.cutout_spacing)

        tot_cutout_area = self.airfoil_area/3.5

        self.cutout_area = tot_cutout_area / self.cutout_amount
        self.cutout_diameter = np.sqrt(self.cutout_area / (np.pi / 4)) * 2  

    def calculate_rib_masses(self, ribs: dict = None):
        if ribs:
            self.x_positions = ribs['x_positions']
            self.thicknesses = ribs['thicknesses']
            self.rib_masses = []
            self.cutout_positioning = {}
            for i in range(len(self.x_positions)):
                idx = np.argmin(np.abs(self.b_array - self.x_positions[i]))
                self.cutout_positioning[i] = {'x': [], 'y': []}
                spar_height = max(self.wing_structure[idx]['spar_info']['spar_heights'])
                front_spar_x = self.wing_structure[idx]['spar_info']['front_spar_x']
                rear_spar_x = self.wing_structure[idx]['spar_info']['rear_spar_x']
                rib_area = self.wing_structure[idx]['airfoil_area'] - self.wing_structure[idx]['ribs']['area'] * self.wing_structure[idx]['ribs']['amount']
                self.rib_volume = rib_area * self.thicknesses[i]
                self.rib_masses.append(self.rib_volume * self.rho_wingbox)

                first_x = front_spar_x - 2*self.cutout_spacing 
                last_x = rear_spar_x 

                for j in range(self.cutout_amount):
                    cutout_x = first_x + j * (self.cutout_spacing+self.wing_structure[idx]['ribs']['diameter'])
                    if cutout_x < front_spar_x:
                        continue
                    if cutout_x > last_x:
                        break

                    cutout_y = spar_height / 2 
                    self.cutout_positioning[i]['x'].append(cutout_x)
                    self.cutout_positioning[i]['y'].append(cutout_y)

    def get_wing_structure(self):
        self.wing_structure = {}
        self.cr_stringer = self.crippling_stress_stringer()
        self.calculate_stringer_moments_inertia()
        self.we2 = []
        for idx, chord in enumerate(self.chord_array):
            self.chord_length = chord
            self.y = self.b_array[idx]
            self.idx = idx
            self.wing_structure[idx] = {}

            if self.evaluate == EvaluateType.WING:
                self.split_airfoil_surfaces()
            elif self.evaluate == EvaluateType.VERTICAL or self.evaluate == EvaluateType.HORIZONTAL:
                self.split_symmetric_airfoil()
            self.get_element_functions()  
            self.get_spar_heights()
            self.get_wingbox_panels()
            self.compute_total_centroid()
            self.get_stringer_placement()
            self.get_moment_of_inertia()
            self.get_polar_moment()
            self.get_wing_rib()

            self.wing_structure[idx]['elements'] = self.element_functions
            self.wing_structure[idx]['area'] = self.wingbox_area
            self.wing_structure[idx]['airfoil_area'] = self.airfoil_area
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
            self.wing_structure[idx]['ribs'] = {
                'diameter': self.cutout_diameter,
                'amount': self.cutout_amount,
                'spacing': self.cutout_spacing,
                'area': self.cutout_area,
            }

            self.we2.append(self.stringer_width)
        
        root_chord_data = self.wing_structure[0]
        self.normalized_data = {}
        self.normalized_data['spar_heights'] = root_chord_data['spar_info']['spar_heights']/self.chord_root
        self.normalized_data['top_skin_lengths'] = root_chord_data['panel_info']['top_skin_length']/self.chord_root
        self.normalized_data['bottom_skin_lengths'] = root_chord_data['panel_info']['bottom_skin_length']/self.chord_root

        self.calculate_wing_mass()
        # self.plot_moment_of_inertia()
        # self.plot_polar_moment()

        self.plot_wing_weight()
        self.spar_thicknesses = self.spar_thickness_span_function(self.b_array)
        self.skin_thicknesses = self.skin_thickness_span_function(self.b_array)
        #self.plot_spacing_2we()
        
    def plot_spacing_2we(self):

        fig, ax = plt.subplots()
        ax.plot(self.b_array, self.we2, label='Stringer Width Distribution', color='blue')
        ax.plot(self.b_array, [self.wing_structure[i]['stringers']['top']['spacing'][0] for i in range(len(self.b_array))], label='Stringer Spacing Top', color='orange')
        ax.set_xlabel('Spanwise Position (m)')
        ax.set_ylabel('Stringer Width (m)')
        ax.set_title('Stringer Width Distribution vs Spanwise Position')
        ax.legend()
        ax.grid()
        plt.show()

    def calculate_wing_mass(self):

        wing_area = []
        wing_box_area = []
        stringers_area = []
        for i in range(len(self.b_array)):
            elements = self.wing_structure[i]['elements']

            upper_length = sum(elem['length'] for elem in elements['upper'])
            lower_length = sum(elem['length'] for elem in elements['lower'])
            total_length = upper_length + lower_length
            airfoil_area = total_length * self.t_wing

            spar_heights = self.wing_structure[i]['spar_info']['spar_heights']
            spar_area = sum(spar_heights) * self.spar_thickness_span_function(i)

            panel_lengths_top = self.wing_structure[i]['panel_info']['widths_top']
            panel_lengths_bottom = self.wing_structure[i]['panel_info']['widths_bottom']
            panel_area = sum(panel_lengths_top) * self.skin_thickness_span_function(i) + sum(panel_lengths_bottom) * self.skin_thickness_span_function(i)
            stringer_info = self.wing_structure[i]['stringers']

            n_stringers = sum(stringer_info['top']['n_stringers']) + sum(stringer_info['bottom']['n_stringers'])
            stringer_area = n_stringers * self.stringer_area
            wing_area.append(airfoil_area)
            wing_box_area.append(panel_area + spar_area + stringer_area)
            stringers_area.append(stringer_area)

        wing_area = np.array(wing_area)
        wing_box_area = np.array(wing_box_area)
        stringers_area = np.array(stringers_area)



        wing_volume = np.trapz(wing_area, self.b_array)
        wing_box_volume = np.trapz(wing_box_area, self.b_array)
        stringers_volume = np.trapz(stringers_area, self.b_array)
        self.wing_mass = (wing_volume * self.rho_wing + wing_box_volume * self.rho_wingbox + stringers_volume * self.rho_stringer)

        return self.wing_mass
    
    def get_fuel_length(self, slope, intercept):
        a = slope/2
        b = intercept
        c = -self.fuel_wing - slope/2 * self.fuel_start**2 - intercept * self.fuel_start

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
        area1 = self.fuel_tank*sum(self.wing_structure[0]['cell_areas'][i] for i in range(self.n_cells))
        area2 = self.fuel_tank*sum(self.wing_structure[self.chord_array.shape[0]-1]['cell_areas'][i] for i in range(self.n_cells))
        area_slope = (area2 - area1) / (self.b / 2)
        area_intercept = area1

        self.fuel_length = self.get_fuel_length(area_slope, area_intercept)
        self.fuel_mass_distribution = np.zeros_like(self.b_array)

        for i, y in enumerate(self.b_array):
            if self.fuel_start <= y <= self.fuel_length:
                area_y = area_intercept + area_slope * y
                chord_y = self.chord_span_function(y)
                self.fuel_mass_distribution[i] = (
                    self.fuel_density * self.fuel_wing*1000 / self.S * chord_y * 9.81
                )

        front_idx = np.argmin(np.abs(self.b_array - self.fuel_start))
        rear_idx = np.argmin(np.abs(self.b_array - self.fuel_length))

        front_length = np.sqrt(self.fuel_tank)*(self.rear_spar - self.front_spar) * self.chord_span_function(self.fuel_start)
        rear_length = np.sqrt(self.fuel_tank)*(self.rear_spar - self.front_spar) * self.chord_span_function(self.fuel_length)
        mid_panel_area = (front_length + rear_length) / 2 * (self.fuel_length - self.fuel_start)*2

        front_height_front_spar = self.wing_structure[front_idx]['spar_info']['front_spar_height']
        front_height_rear_spar = self.wing_structure[front_idx]['spar_info']['rear_spar_height']
        front_panel_area = (front_height_front_spar + front_height_rear_spar) / 2 * front_length

        rear_height_front_spar = self.wing_structure[rear_idx]['spar_info']['front_spar_height']
        rear_height_rear_spar = self.wing_structure[rear_idx]['spar_info']['rear_spar_height']
        rear_panel_area = (rear_height_front_spar + rear_height_rear_spar) / 2 * rear_length

        TE_panel_area = (front_height_rear_spar + rear_height_rear_spar) / 2 * (self.fuel_length - self.fuel_start)
        LE_panel_area = (front_height_front_spar + rear_height_front_spar) / 2 * (self.fuel_length - self.fuel_start)

        if self.evaluate == EvaluateType.WING:
            self.tank_mass = (mid_panel_area + front_panel_area + rear_panel_area + TE_panel_area + LE_panel_area)*self.fuel_tank_thickness*self.rho_wingbox
        else:
            self.tank_mass = 0
        return self.fuel_mass_distribution

    def gaussian_peak(self,x, x0, A, sigma=0.1):
        return A * np.exp(-((x - x0)**2) / (2 * sigma**2)) / (sigma * np.sqrt(2 * np.pi))
    
    def wing_weight_dist(self):
        if self.evaluate == EvaluateType.WING:
            fuel_dist = self.get_fuel_mass_distribution()
        else:
            fuel_dist = np.zeros_like(self.b_array)
            self.tank_mass = 0
        if hasattr(self, 'rib_masses'):
            print(f'tot rib mass: {sum(self.rib_masses)} kg')
            self.wing_mass = self.calculate_wing_mass() + sum(self.rib_masses) + self.tank_mass
        else:
            self.wing_mass = self.calculate_wing_mass() + self.tank_mass

        if self.evaluate == EvaluateType.VERTICAL:
            self.weight_dist = self.wing_mass/(self.S)*self.chord_span_function(self.b_array)*9.81 + fuel_dist
        else:
            self.weight_dist = self.wing_mass/(self.S/2)*self.chord_span_function(self.b_array)*9.81 + fuel_dist

        if self.evaluate == EvaluateType.WING:
            for engine_y in self.engine_positions:
                self.weight_dist[int(round(engine_y,2)*100)] += self.engine_weight*9.81
        
        self.wing_mass = self.wing_mass + self.tank_mass
        print(f"Wing mass without fuel: {self.wing_mass:.2f} kg")
        self.weight_dist[-1] += self.buoy_mass*9.81

        self.tot_weight = np.trapz(self.weight_dist, self.b_array)
        print(f"Wing mass all inc: {self.tot_weight/9.81:.2f} kg")
        return self.weight_dist

    def plot_wing_ribs(self):
        fig, ax = plt.subplots()
        for i in range(len(self.x_positions)):
            rib_x = self.x_positions[i]
            idx = np.argmin(np.abs(self.b_array - rib_x))
            rib_length = self.chord_span_function(rib_x)
            if self.evaluate == EvaluateType.WING:
                if self.flap_start <= rib_x <= self.flap_end:
                    rib_length -= (1-self.rear_spar)*rib_length
                if self.aileron_start <= rib_x <= self.aileron_end:
                    rib_length -= (1-self.rear_spar)*rib_length
            elif self.evaluate == EvaluateType.VERTICAL:
                if self.rudder_start <= rib_x <= self.rudder_end:
                    rib_length -= (1-self.rear_spar)*rib_length
            elif self.evaluate == EvaluateType.HORIZONTAL:
                if self.elevator_start <= rib_x <= self.elevator_end:
                    rib_length -= (1-self.rear_spar)*rib_length

            ax.plot(self.b_array, self.leading_edge, color='blue', label='Leading Edge' if i == 0 else "")
            ax.plot(self.b_array, self.leading_edge - self.chord_span_function(self.b_array), color='blue', label='Trailing Edge' if i == 0 else "")
            ax.plot([rib_x, rib_x], [self.leading_edge[idx], self.leading_edge[idx]-rib_length], color='green')

        if self.evaluate == EvaluateType.WING:
            front_idx = np.argmin(np.abs(self.b_array - self.fuel_start))
            rear_idx = np.argmin(np.abs(self.b_array - self.fuel_length))

            fuel_tank_start_LE = self.leading_edge[front_idx] - np.sqrt(self.fuel_tank)/2*(self.rear_spar-self.front_spar)* self.chord_span_function(self.fuel_start)
            fuel_tank_start_TE = fuel_tank_start_LE - (1-np.sqrt(self.fuel_tank)/2)*(self.rear_spar - self.front_spar)* self.chord_span_function(self.fuel_start)
            fuel_tank_rear_LE = self.leading_edge[rear_idx] - np.sqrt(self.fuel_tank)/2*(self.rear_spar - self.front_spar)* self.chord_span_function(self.fuel_length)
            fuel_tank_rear_TE = fuel_tank_rear_LE - (1-np.sqrt(self.fuel_tank)/2)*(self.rear_spar - self.front_spar)* self.chord_span_function(self.fuel_length)
            x_coords = [self.fuel_start, self.fuel_length, self.fuel_length, self.fuel_start]
            y_coords = [fuel_tank_start_LE, fuel_tank_rear_LE, fuel_tank_rear_TE, fuel_tank_start_TE]

            ax.fill(x_coords, y_coords, color='red', alpha=0.3, label='Fuel Tank')
            ax.plot([self.fuel_start, self.fuel_start], [fuel_tank_start_LE, fuel_tank_start_TE], color='red')
            ax.plot([self.fuel_length, self.fuel_length], [fuel_tank_rear_LE, fuel_tank_rear_TE], color='red')
            ax.plot([self.fuel_start, self.fuel_length], [fuel_tank_start_LE, fuel_tank_rear_LE], color='red')
            ax.plot([self.fuel_start, self.fuel_length], [fuel_tank_start_TE, fuel_tank_rear_TE], color='red')
            ax.axvline(self.w_fuselage/2, color='red', linestyle='--')
        ax.set_xlabel('Lateral Position (m)')
        ax.set_ylabel('Longitudinal Position (m)')
        ax.set_title('Wing Ribs along Span and Fuel Tank')
        ax.set_aspect('equal')
        ax.grid()

        plt.show()

    def plot_thickness_distribution(self):
        fig, ax = plt.subplots()
        ax.plot(self.b_array, self.spar_thickness_span_function(self.b_array), label='Thickness Distribution', color='blue')
        ax.plot(self.b_array, self.skin_thickness_span_function(self.b_array), label='Skin Thickness Distribution', color='orange')
        ax.set_xlabel('Chord Length (m)')
        ax.set_ylabel('Thickness (m)')
        ax.set_title('Wing Thickness Distribution vs Chord Length')
        ax.legend()
        ax.grid()
        plt.show()
                                         

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

    def plot_rib(self,id=40):
        cutout_positions = self.cutout_positioning[id]
        idx = np.argmin(np.abs(self.b_array - self.x_positions[id]))
        x_coords = self.wing_structure[idx]['x_coords']
        y_upper = self.wing_structure[idx]['y_upper']
        y_lower = self.wing_structure[idx]['y_lower']
        fig, ax = plt.subplots()

        ax.plot(x_coords, y_upper, label='Airfoil Upper Surface', color='blue')
        ax.plot(x_coords, y_lower, label='Airfoil Lower Surface', color='blue')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')

        for i in cutout_positions['x']:
            cutout_x = i
            cutout_y = cutout_positions['y'][cutout_positions['x'].index(i)]
            circle = plt.Circle((cutout_x, cutout_y), self.cutout_diameter/2, color='red', alpha=0.5)
            ax.add_artist(circle)
        ax.set_title(f"Rib Cutouts for Chord Index {idx}")
        ax.set_aspect('equal')
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

        for x,y,a in zip(stringers['top']['x'], stringers['top']['y'], stringers['top']['angle']):
            self.draw_I_beam(x, y, self.L_stringer/1000, self.t_stringer/1000, top=True, color='black',angle=a)
        for x,y,a in zip(stringers['bottom']['x'], stringers['bottom']['y'], stringers['bottom']['angle']):
            self.draw_I_beam(x, y, self.L_stringer/1000, self.t_stringer/1000, top=False, color='purple',angle=0)

        plt.scatter(centroid[0], centroid[1], color='green', label='Centroid')
        centroid_x, centroid_z = centroid
        arrow_length = 0.05 * chord
        plt.arrow(centroid_x, centroid_z, 0, arrow_length,
                head_width=0.05 * arrow_length, head_length=0.05 * arrow_length,
                fc='black', ec='black', linewidth=2, zorder=20)
        plt.text(centroid_x, centroid_z + arrow_length + 0.02 * arrow_length,
                'z', fontsize=10, ha='center', va='bottom')

        # Arrow: Left in X (negative X direction)
        plt.arrow(centroid_x, centroid_z, -arrow_length, 0,
                head_width=0.05 * arrow_length, head_length=0.05 * arrow_length,
                fc='black', ec='black', linewidth=2, zorder=20)
        plt.text(centroid_x - arrow_length - 0.08 * arrow_length, centroid_z,
                'x', fontsize=10, ha='right', va='center')
        plt.title(f"Airfoil with Spar Positions, Wing Box and Stringers, chord = {chord:.2f} m")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    aircraft_data = Data("design3.json")
    stringer_material = Materials.Al7075
    wingbox_material = Materials.Al7075
    wing_material = Materials.Al5052
    wing_structure = WingStructure(aircraft_data, wingbox_mat=wingbox_material,
                                   wing_mat=wing_material, stringer_mat=stringer_material, evaluate=EvaluateType.WING)
    wing_structure.get_wing_structure()
    #3187
    wing_structure.plot_airfoil(chord_idx=0)
    wing_structure.plot_moment_of_inertia()
    wing_structure.plot_thickness_distribution()
    print(f"Stringer Length, thickness in mm:{wing_structure.calculate_stringer_thickness()}")
    print(f"Crippling stress stringer in MPa: {wing_structure.crippling_stress_stringer()}")