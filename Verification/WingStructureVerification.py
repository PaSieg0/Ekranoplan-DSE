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
                 stringer_mat: Materials, wing_mat: Materials, evaluate: EvaluateType,
                 plot: bool = True):
        self.aircraft_data = aircraft_data
        self.evaluate = evaluate
        self.front_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['front_spar']
        self.rear_spar = self.aircraft_data.data['inputs']['structures']['wing_box']['rear_spar']
        self.plot = plot
        self.rho_epoxy = self.aircraft_data.data['inputs']['structures']['materials']['Epoxy']['rho']

        if self.evaluate == EvaluateType.WING:
            self.airfoil_data = Data('Airfoil_data.dat', 'airfoil_geometry')
            self.data = self.airfoil_data.data
            self.b = self.aircraft_data.data['outputs']['wing_design']['b']
            self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
            self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']
            self.b_array = np.arange(0, self.b/2+0.01, 0.01)
            self.chord_array = self.chord_span_function(self.b_array)
            self.chord_length = self.chord_root
            self.epoxy_out = self.aircraft_data.data['inputs']['structures']['wing_box']['epoxy_out']
            self.epoxy_in = self.aircraft_data.data['inputs']['structures']['wing_box']['epoxy_in']
            self.S = self.aircraft_data.data['outputs']['wing_design']['S']
            self.min_skin_thickness = self.aircraft_data.data['inputs']['structures']['wing_box']['min_skin_thickness']/1000
            self.min_spar_thickness = self.aircraft_data.data['inputs']['structures']['wing_box']['min_spar_thickness']/1000
            self.thickness_threshold = int(self.aircraft_data.data['inputs']['structures']['wing_box']['thickness_threshold']*self.b/2*100)
            self.n_cells = self.aircraft_data.data['inputs']['structures']['wing_box']['n_cells']
            self.fuel_volume = self.aircraft_data.data['outputs']['max']['max_fuel_L']/1000
            self.flap_start = self.aircraft_data.data['outputs']['HLD']['b1']
            self.flap_end = self.aircraft_data.data['outputs']['HLD']['b2']
            self.y_MAC = self.aircraft_data.data['outputs']['wing_design']['y_MAC']

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
            self.bottom_spar_margin = 0.0
            self.top_spar_margin = 1

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
            self.y_MAC = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['y_MAC']
            self.epoxy_out = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['epoxy_out']
            self.epoxy_in = self.aircraft_data.data['inputs']['structures']['horizontal_wing_box']['epoxy_in']
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
            self.airfoil_data = Data('Verify_foil.dat', 'airfoil_geometry')
            self.data = self.airfoil_data.data
            self.b = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
            self.chord_root = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
            self.chord_tip = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
            self.b_array = np.arange(0, self.b+0.01, 0.01)
            self.chord_array = self.chord_span_function(self.b_array)
            self.epoxy_out = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['epoxy_out']
            self.epoxy_in = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['epoxy_in']
            self.chord_length = self.chord_root
            self.S = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S']
            self.fuel_tank = 0
            self.buoy_mass = 0
            self.fuel_wing = 0
            
            self.y_MAC = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['z_MAC']
            self.t_spar = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['t_spar']/1000
            self.t_skin = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['t_skin']/1000
            self.t_wing = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['t_wing']/1000
            self.engine_positions = None
            self.cutout_spacing = self.aircraft_data.data['inputs']['structures']['vertical_wing_box']['cutout_spacing']
            self.bottom_spar_margin = 0.0
            self.top_spar_margin = 1
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
        # print(self.evaluate)

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
        x = np.array(self.data['x'])
        y = np.array(self.data['y'])
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
        x = np.array(self.data['x'])
        y = np.array(self.data['y'])

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
        spar_xs = [pos for pos in spar_positions]
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

        I_xx_front_spar = self.beam_standard_Ixx(front_spar_height, 0.001,
                                                abs(front_spar_height / 2 - self.centroid[1]), np.pi / 2)
        I_yy_front_spar = self.beam_standard_Iyy(front_spar_height, 0.001,
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
        
        I_xx_panels = self.beam_standard_Ixx(0.5,0.001,0.5,0)*2
        I_yy_panels = self.beam_standard_Iyy(1,0.001,0,0)*2
        

                # print(mid_spar_height, self.t_spar)
                # print(self.beam_standard_Ixx(mid_spar_height, self.t_spar,
                #                                          abs(mid_spar_height / 2 - self.centroid[1]), np.pi / 2))

        I_xx_wing = self.beam_standard_Ixx(1, 0.001, 0.5, 0)*2 + self.beam_standard_Ixx(1,0.001, 0, np.pi/2)*2
        I_yy_wing = self.beam_standard_Iyy(1, 0.001, 0.5, np.pi/2)*2 + self.beam_standard_Iyy(0.5,0.001, 0, 0)*2

        # print(I_xx_front_spar + I_xx_rear_spar + I_xx_mid_spars)
        self.I_xx = (I_xx_front_spar + I_xx_rear_spar + I_xx_panels +
                 + I_xx_wing + I_xx_mid_spars)
        self.I_yy = (I_yy_front_spar + I_yy_rear_spar + I_yy_panels +
                    + I_yy_wing + I_yy_mid_spars)
        self.I_xy = (I_xy_front_spar + I_xy_rear_spar + I_xy_panels +
                    + I_xy_mid_spars)

        # print(I_xx_front_spar, I_xx_rear_spar, I_xx_panels, I_xx_wing_bottom, I_xx_wing_top, I_xx_mid_spars, self.stringer_dict['top']['I_xx'], self.stringer_dict['bottom']['I_xx'])
        # print(self.I_xx)

    def get_wing_structure(self):
        self.wing_structure = {}
        self.we2 = []
        for idx, chord in enumerate(self.chord_array[:1]):
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
            self.get_moment_of_inertia()

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

        
        root_chord_data = self.wing_structure[0]
        self.normalized_data = {}
        self.normalized_data['spar_heights'] = root_chord_data['spar_info']['spar_heights']/self.chord_root
        self.normalized_data['top_skin_lengths'] = root_chord_data['panel_info']['top_skin_length']/self.chord_root
        self.normalized_data['bottom_skin_lengths'] = root_chord_data['panel_info']['bottom_skin_length']/self.chord_root
        print(root_chord_data['I_xx'], root_chord_data['I_yy'], root_chord_data['I_xy'])

        # if self.plot:
            # self.plot_Ip()
            # self.plot_moment_of_inertia()
            # self.plot_polar_moment()

            #self.plot_spacing_2we()


if __name__ == "__main__":
    aircraft_data = Data("verification.json")
    stringer_material = Materials.Al7075
    wingbox_material = Materials.Al7075
    wing_material = Materials.Al5052
    wing_structure = WingStructure(aircraft_data, wingbox_mat=wingbox_material,
                                   wing_mat=wing_material, stringer_mat=stringer_material, evaluate=EvaluateType.VERTICAL)
    wing_structure.get_wing_structure()
    # print(f"Stringer Length, thickness in mm:{wing_structure.calculate_stringer_thickness()}")
    # print(f"Crippling stress stringer in MPa: {wing_structure.crippling_stress_stringer()}")