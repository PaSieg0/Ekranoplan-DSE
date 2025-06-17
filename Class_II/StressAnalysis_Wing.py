import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, StressOutput, Materials, EvaluateType
from Class_II.AerodynamicForces import AerodynamicForces
from Class_II.WingStructure import WingStructure
import numpy as np
from Class_II.FlutterAnalysis import FlutterAnalysis
class StressAnalysisWing(AerodynamicForces, WingStructure):

    def __init__(self, aircraft_data: Data,
                 wing_mat: Materials, wingbox_mat: Materials, stringer_mat: Materials, 
                 evaluate: EvaluateType, PLOT: bool = False):
        AerodynamicForces.__init__(self, aircraft_data, evaluate=evaluate)
        WingStructure.__init__(self, aircraft_data, wingbox_mat=wingbox_mat, wing_mat=wing_mat, stringer_mat=stringer_mat, evaluate=evaluate, plot=PLOT)
        self.get_wing_structure()
        self.safety_factor = 1.5
        self.PLOT = PLOT
        self.runs = 0
        self.k_s = 15
        self.engine_power = self.aircraft_data.data['inputs']['engine']['engine_power']
        self.engine_thrust = self.engine_power / self.V
        self.dy = np.gradient(self.b_array)

        self.max_load_factor = self.aircraft_data.data['outputs']['general']['nmax']
        self.min_load_factor = self.aircraft_data.data['outputs']['general']['nmin']
        self.evaluate_case = 'max'
        self.margins = {}
        self.load_factor = self.max_load_factor
        self.I_xx_array = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
        self.I_yy_array = np.array([self.wing_structure[i]['I_yy'] for i in range(len(self.wing_structure))])
        self.I_xy_array = np.array([self.wing_structure[i]['I_xy'] for i in range(len(self.wing_structure))])
        self.k_v = 1.5 
        self.spar_heights = np.array([self.wing_structure[i]['spar_info']['spar_heights'] for i in range(len(self.wing_structure))])
        self.l_stringer = self.b/2
        self.widths_top = np.array([self.wing_structure[i]['stringers']['top']['spacing'] for i in range(len(self.wing_structure))])
        self.widths_bottom = np.array([self.wing_structure[i]['stringers']['bottom']['spacing'] for i in range(len(self.wing_structure))])
        self.poisson_ratio_wing = self.wing_material['poisson_ratio']
        self.bv = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        self.bh = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.mass_vertical = self.aircraft_data.data['outputs']['component_weights']['vertical_tail']/9.81
        self.mass_horizontal = self.aircraft_data.data['outputs']['component_weights']['horizontal_tail']/9.81
        self.mass_wing = self.aircraft_data.data['outputs']['component_weights']['wing']/9.81
        self.E_wing = self.wing_material['E']
        self.load_data = {}
        self.rib_iteration = 0
        self.max_rib_iteration = 10
        self.rho_water = self.aircraft_data.data['rho_water']
        self.wing_weight = self.wing_weight_dist()
        self.MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        self.buoy_width = 1 #TODO link these to json
        self.buoy_length = 4.5
        self.buoy_height = 5.8
        self.buoy_projected_area = self.buoy_length*self.buoy_width
        self.roll_rate = np.deg2rad(self.aircraft_data.data['outputs']['control_surfaces']['aileron']['roll_rate'])
        self.Cs = 1.5
        self.quarter_chord_dist = [self.wing_structure[i]['quarter_chord_dist'] for i in range(len(self.wing_structure))]

        # self.mass_I_xx = 1/8*(self.mass_wing*self.b**2 + self.mass_vertical*self.bv**2 + self.mass_horizontal*self.bh**2) #TODO check with Shuard
        self.kinematic_viscosity = self.aircraft_data.data['kinematic_viscosity']
        self.h_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_station1']
        self.dihedral = self.aircraft_data.data['outputs']['wing_design']['dihedral']
        self.last_centroid = self.wing_structure[len(self.b_array)-1]['centroid'][1] + self.h_fuselage + np.tan(np.deg2rad(self.dihedral))*self.b/2
        self.buoy_arm = self.buoy_height - self.last_centroid
        self.buoy_drag = self.get_buoy_drag()

    def estimate_slam_depth(self):
        A = self.buoy_projected_area
        m_eff = self.buoy_length * self.buoy_width * self.buoy_height * self.rho_water
        d = m_eff / (self.Cs * self.rho_water * A)

        return d

    def calculate_buoy_surface(self):
        
        v_tip_z = self.roll_rate * (self.b / 2)
        A_impact = self.buoy_projected_area 
        self.F_slam = 0.5 * self.Cs * self.rho_water * A_impact * v_tip_z**2 
        depth = self.estimate_slam_depth()

        A_wet = self.buoy_projected_area + depth * self.buoy_length * 2 + depth * self.buoy_width * 2
        return A_wet
    
    def get_buoy_drag(self):
        A_buoy = self.calculate_buoy_surface()
        rho_water = self.aircraft_data.data['rho_water']
        Cd = self.calculate_Cd()
        buoy_drag = 0.5 * rho_water * Cd * A_buoy * self.V_land**2
        return buoy_drag
    
    def calculate_Re(self):
        return self.V_land*self.buoy_length / self.kinematic_viscosity
         
    def calculate_Cd(self):
        self.Re = self.calculate_Re()
        Cd = 0.075 / (np.log10(self.Re) - 2)**2
        return Cd
    
    def main_stresses(self):
        if self.evaluate_case == 'max':
            self.get_max_aero_dist()
        elif self.evaluate_case == 'min':
            self.get_nominal_aero_dist()

        if self.evaluate == EvaluateType.WING:
            self.lift_function = self.L_y
            self.drag_array = -self.D_y
            self.moment_function = self.M_y

        elif self.evaluate == EvaluateType.HORIZONTAL:
            self.lift_function = self.get_horizontal_tail_lift_distribution()
        
        elif self.evaluate == EvaluateType.VERTICAL:
            self.lift_function = self.get_vertical_tail_lift_distribution()


        self.resultant_vertical_distribution()
        if self.evaluate == EvaluateType.WING:
            self.resultant_horizontal_distribution()
            self.internal_bending_moment_y()
        self.internal_bending_moment_x()
        self.calculate_top_bending_stress_wing()
        self.calculate_critical_web_stress()
        self.internal_vertical_shear_force()
        self.internal_torque()


        if self.evaluate == EvaluateType.WING:
            self.internal_horizontal_shear_force()
            self.calculate_horizontal_shear_stress()
        
        self.calculate_tot_shear_stress()
        self.calculate_rib_spacing_skin()
        self.map_rib_thicknesses_to_positions()

    def main_analysis(self, run_all = True):
        self.main_stresses()
        self.run_all = run_all
        self.prev_tot_wing_weight = self.tot_weight

        while True:
            self.rib_iteration += 1
            # print(f'Rib Iteration: {self.rib_iteration}, {self.tot_weight}')
            self.calculate_rib_masses(ribs={'x_positions': self.rib_positions, 'thicknesses': self.rib_thicknesses})
            self.wing_weight = self.wing_weight_dist()
            self.curr_weight = self.tot_weight

            stop_condition = abs(self.curr_weight - self.prev_tot_wing_weight) < 0.001 or self.rib_iteration >= self.max_rib_iteration

            if stop_condition:
                self.finalize_analysis()
                break
            
            self.prev_tot_wing_weight = self.curr_weight

            self.main_stresses()

    def finalize_analysis(self):
        self.calculate_top_bending_stress()
        self.calculate_bottom_bending_stress()
        self.calculate_wing_deflection()
        self.calculate_critical_buckling_stress()
        self.column_buckling()
        self.get_margins()
        self.runs += 1

        self.load_factor = self.min_load_factor
        if self.runs < 2 and self.evaluate == EvaluateType.WING and self.run_all:
            self.evaluate_case = 'min'

            self.main_stresses()
            self.finalize_analysis()

    def resultant_vertical_distribution(self):
        if self.evaluate_case == 'max':
            if self.evaluate == EvaluateType.WING:
                self.vertical_distribution = self.max_load_factor*self.lift_function - self.wing_weight
            else:
                self.vertical_distribution = self.max_load_factor*self.lift_function - self.wing_weight if self.evaluate == EvaluateType.HORIZONTAL else self.lift_function
            return self.vertical_distribution
        elif self.evaluate_case == 'min':
            if self.evaluate == EvaluateType.WING:
                self.vertical_distribution = self.lift_function - self.wing_weight
                self.vertical_distribution[-1] += self.F_slam
            else:
                self.vertical_distribution = self.min_load_factor*self.lift_function - self.wing_weight if self.evaluate == EvaluateType.HORIZONTAL else self.lift_function
            return self.vertical_distribution
    
    def resultant_torque_distribution(self):
        if self.evaluate_case == 'max':
            if self.evaluate == EvaluateType.WING:
                self.torque_distribution = self.max_load_factor*self.lift_function * self.quarter_chord_dist + self.moment_function
            else:
                self.torque_distribution = self.max_load_factor*self.lift_function * self.quarter_chord_dist
            return self.torque_distribution
        elif self.evaluate_case == 'min':
            if self.evaluate == EvaluateType.WING:
                self.torque_distribution = self.lift_function * self.quarter_chord_dist + self.moment_function
                self.torque_distribution[-1] += self.buoy_drag * self.buoy_arm
            else:
                self.torque_distribution = self.min_load_factor*self.lift_function * self.quarter_chord_dist
            return self.torque_distribution
        
    def resultant_horizontal_distribution(self):
        self.horizontal_distribution = self.drag_array.copy()
        for engine_y in self.engine_positions:
            self.horizontal_distribution[int(round(engine_y,2)*100)] += self.engine_thrust

        if self.evaluate_case == 'min' and self.evaluate == EvaluateType.WING:
            self.horizontal_distribution[-1] -= self.buoy_drag

        return self.horizontal_distribution

    def internal_vertical_shear_force(self):

        load = self.resultant_vertical_distribution()
        load = load.copy()
        if self.evaluate == EvaluateType.WING and self.evaluate_case == 'min':
            load[-1] = load[-1]/self.dy[-1]

        Vy_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.Vy_internal = -Vy_flipped[::-1]

        return self.Vy_internal
    
    def internal_horizontal_shear_force(self):
        engine_load = np.zeros_like(self.b_array)
        for engine_y in self.engine_positions:
            engine_load[int(round(engine_y,2)*100)] = self.engine_thrust
        Vx_flipped_engines = np.cumsum(engine_load[::-1] * self.dy[::-1])

        load = self.drag_array
        load = load.copy()
        if self.evaluate == EvaluateType.WING and self.evaluate_case == 'min':
            load[-1] = load[-1]/self.dy[-1]
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
    
    def internal_torque(self):
        load = self.resultant_torque_distribution()
        load = load.copy()
        if self.evaluate == EvaluateType.WING and self.evaluate_case == 'min':
            load[-1] = load[-1]/self.dy[-1]
        T_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.T_internal = T_flipped[::-1]

        return self.T_internal
    
    def calculate_top_bending_stress(self): 
        y = [self.wing_structure[i]['spar_info']['max_y'] - self.wing_structure[i]['centroid'][1] for i in range(len(self.wing_structure))]
        x = [self.wing_structure[i]['centroid'][0] -self.wing_structure[i]['spar_info']['max_x'] for i in range(len(self.wing_structure))]
        if self.evaluate == EvaluateType.WING:
            self.top_bending_stress = self.safety_factor*((self.internal_bending_moment_x()*self.I_yy_array -(self.internal_bending_moment_y()*self.I_xy_array))*y + (self.internal_bending_moment_y()*self.I_xx_array - (self.internal_bending_moment_x()*self.I_xy_array))*x) / (self.I_xx_array*self.I_yy_array - self.I_xy_array**2)/1000000
        else:
            self.top_bending_stress = self.safety_factor*(self.internal_bending_moment_x()*y/self.I_xx_array)/1000000
        return self.top_bending_stress
    
    def calculate_top_bending_stress_wing(self): 
        y = [max(self.wing_structure[i]['y_upper']) - self.wing_structure[i]['centroid'][1] for i in range(len(self.wing_structure))]
        x = [self.wing_structure[i]['centroid'][0] - self.wing_structure[i]['x_coords'][list(self.wing_structure[i]['y_upper']).index(max(self.wing_structure[i]['y_upper']))] for i in range(len(self.wing_structure))]
        if self.evaluate == EvaluateType.WING:
            self.top_bending_stress_wing = self.safety_factor*((self.internal_bending_moment_x()*self.I_yy_array -(self.internal_bending_moment_y()*self.I_xy_array))*y + (self.internal_bending_moment_y()*self.I_xx_array - (self.internal_bending_moment_x()*self.I_xy_array))*x) / (self.I_xx_array*self.I_yy_array - self.I_xy_array**2)
        else:
            self.top_bending_stress_wing = self.safety_factor*(self.internal_bending_moment_x()*y/self.I_xx_array)
        return self.top_bending_stress_wing
    
    def calculate_bottom_bending_stress(self):
        y = [self.wing_structure[i]['spar_info']['min_y'] - self.wing_structure[i]['centroid'][1] for i in range(len(self.wing_structure))]
        x = [self.wing_structure[i]['centroid'][0] - self.wing_structure[i]['spar_info']['min_x'] for i in range(len(self.wing_structure))]
        if self.evaluate == EvaluateType.WING:
            self.bottom_bending_stress = self.safety_factor*((self.internal_bending_moment_x()*self.I_yy_array -(self.internal_bending_moment_y()*self.I_xy_array))*y + (self.internal_bending_moment_y()*self.I_xx_array - (self.internal_bending_moment_x()*self.I_xy_array))*x) / (self.I_xx_array*self.I_yy_array - self.I_xy_array**2)/1000000
        else:
            self.bottom_bending_stress = self.safety_factor*(self.internal_bending_moment_x()*y/self.I_xx_array)/1000000
        return self.bottom_bending_stress
    
    def calculate_dtheta_dy(self,idx):
        coefficient_matrix = np.zeros((self.n_cells+1, self.n_cells+1))
        spar_heights = self.wing_structure[idx]['spar_info']['spar_heights']
        top_skin_lengths = self.wing_structure[idx]['panel_info']['top_skin_length']
        bottom_skin_lengths = self.wing_structure[idx]['panel_info']['bottom_skin_length']
        cell_areas = self.wing_structure[idx]['cell_areas']
        torque = self.T_internal[idx]
        skin_thickness = self.skin_thicknesses[idx]
        spar_thickness = self.spar_thicknesses[idx]
        for i in range(self.n_cells):
            area_factor = 1/(2*cell_areas[i])
            if i == 0:
                coefficient_left_spar = spar_heights[i] / (self.G*spar_thickness)
                coefficient_top = top_skin_lengths[i] / (self.G*skin_thickness)
                coefficient_bottom = bottom_skin_lengths[i] / (self.G*skin_thickness)
                coefficient_right_spar2 = -spar_heights[i+1] / (self.G*spar_thickness)
                coefficient_right_spar1 = spar_heights[i+1] / (self.G*spar_thickness)

                coefficient_matrix[i, 0] = (coefficient_left_spar + coefficient_top + coefficient_bottom + coefficient_right_spar1)*area_factor
                coefficient_matrix[i, i+1] = coefficient_right_spar2*area_factor
                coefficient_matrix[i,-1] = -1
                coefficient_matrix[-1, i] = 1/area_factor

            elif i == self.n_cells:
                coefficient_left_spar1 = -spar_heights[i] / (self.G*spar_thickness)
                coefficient_left_spar2 = spar_heights[i] / (self.G*spar_thickness)
                coefficient_top = top_skin_lengths[i] / (self.G*skin_thickness)
                coefficient_bottom = bottom_skin_lengths[i] / (self.G*skin_thickness)

                coefficient_matrix[i, i-1] = coefficient_left_spar1*area_factor
                coefficient_matrix[i, i] = (coefficient_left_spar2 + coefficient_top + coefficient_bottom)*area_factor
                coefficient_matrix[i, -1] = -1
                coefficient_matrix[-1, i] = 1/area_factor

            else:
                coefficient_left_spar1 = -spar_heights[i] / (self.G*spar_thickness)
                coefficient_left_spar2 = spar_heights[i] / (self.G*spar_thickness)
                coefficient_top = top_skin_lengths[i] / (self.G*skin_thickness)
                coefficient_bottom = bottom_skin_lengths[i] / (self.G*skin_thickness)
                coefficient_right_spar1 = spar_heights[i+1] / (self.G*spar_thickness)
                coefficient_right_spar2 = -spar_heights[i+1] / (self.G*spar_thickness)

                coefficient_matrix[i, i-1] = coefficient_left_spar1*area_factor
                coefficient_matrix[i, i] = (coefficient_left_spar2 + coefficient_top + coefficient_bottom + coefficient_right_spar1)*area_factor
                coefficient_matrix[i, i+1] = coefficient_right_spar2*area_factor
                coefficient_matrix[i, -1] = -1
                coefficient_matrix[-1, i] = 1/area_factor

        y = np.zeros(self.n_cells+1)
        y[-1] = torque

        solution = np.linalg.solve(coefficient_matrix, y)
        max_stress = max([i/self.spar_thicknesses[idx] for i in solution[:-1]])
        max_shear_flow = max_stress*self.spar_thicknesses[idx]
        spar_height = spar_heights[np.argmax(np.abs(solution[:-1]))]
        dtheta_dy = np.rad2deg(solution[-1])
        return max_stress, dtheta_dy, max_shear_flow, spar_height

    def calculate_shear_stress(self):
        Vy_internal = self.Vy_internal
        avg_shear_stress = Vy_internal / [sum(self.wing_structure[i]['spar_info']['spar_heights_t']) for i in range(len(self.wing_structure))]
        avg_shear_flow = Vy_internal/[sum(self.wing_structure[i]['spar_info']['spar_heights']) for i in range(len(self.wing_structure))]
        self.vertical_shear_flow = self.safety_factor*avg_shear_flow * self.k_v/1000000
        self.max_shear_stress = self.safety_factor*avg_shear_stress * self.k_v/1000000
        return self.max_shear_stress
    
    def calculate_horizontal_shear_stress(self):
        Vx_internal = self.Vx_internal
        avg_shear_stress_top = Vx_internal / [sum(self.wing_structure[i]['panel_info']['top_skin_length'])*self.skin_thicknesses[i] for i in range(len(self.wing_structure))]
        avg_shear_stress_bot = Vx_internal / [sum(self.wing_structure[i]['panel_info']['bottom_skin_length'])*self.skin_thicknesses[i] for i in range(len(self.wing_structure))]
        self.max_horizontal_shear_stress_top = self.safety_factor*avg_shear_stress_top * self.k_v/1000000
        self.max_horizontal_shear_stress_bottom = self.safety_factor*avg_shear_stress_bot * self.k_v/1000000
        self.horizontal_shear_flow = Vx_internal / [sum(self.wing_structure[i]['panel_info']['top_skin_length']) for i in range(len(self.wing_structure))]
        return self.max_horizontal_shear_stress_top, self.max_horizontal_shear_stress_bottom
    
    def calculate_torsion(self):
        self.torsion = np.zeros(len(self.b_array))
        self.dtheta_dy = np.zeros(len(self.b_array))
        self.torsional_shear_flow = np.zeros(len(self.b_array))
        self.critical_spar_heights = np.zeros(len(self.b_array))
        for i in range(len(self.b_array)):
            stress, dtheta_dy, shear_flow, spar_height = self.calculate_dtheta_dy(i)
            self.torsion[i] = self.safety_factor*stress/1000000
            self.dtheta_dy[i] = dtheta_dy
            self.torsional_shear_flow[i] = shear_flow / 1000000
            self.critical_spar_heights[i] = spar_height

        self.twist = np.cumsum(self.dtheta_dy * self.dy)
        return self.torsion, self.twist, self.torsional_shear_flow
    
    def calculate_tot_shear_stress(self):
        self.shear_stress = self.calculate_shear_stress() + self.calculate_torsion()[0]
        self.max_vertical_shear_flow = self.vertical_shear_flow + self.torsional_shear_flow
        if self.evaluate == EvaluateType.WING:
            self.max_horizontal_shear_flow = self.horizontal_shear_flow + self.torsional_shear_flow
        return self.shear_stress

    def calculate_wing_deflection(self):
        moment = self.M_internal
        moment_flipped = np.cumsum(moment[::-1]*self.dy[::-1])
        self.wing_deflection = np.cumsum(moment_flipped*self.dy) / (-self.E * np.average(self.I_xx_array))

        return self.wing_deflection

    def column_buckling(self):
        k = 1/4 # for one fixed, one free end
        stringer_I_xx_top = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
        stringer_I_xx_bottom = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
        n_stringers_top = np.array([self.wing_structure[i]['stringers']['top']['n_stringers'] for i in range(len(self.wing_structure))])
        n_stringers_bottom = np.array([self.wing_structure[i]['stringers']['bottom']['n_stringers'] for i in range(len(self.wing_structure))])
        top_column_buckling = []
        bottom_column_buckling = []
        for y in range(len(self.b_array)):
            sigma_col_top = np.min((k * np.pi**2 * self.E_stringer * stringer_I_xx_top[y]) / (self.l_stringer**2 * n_stringers_top[y]*self.stringer_area)/1000000)
            sigma_col_bottom = np.min((k * np.pi**2 * self.E_stringer * stringer_I_xx_bottom[y]) / (self.l_stringer**2 * n_stringers_bottom[y]*self.stringer_area)/1000000)
            top_column_buckling.append(sigma_col_top)
            bottom_column_buckling.append(sigma_col_bottom)

        self.sigma_col_top = np.array(top_column_buckling)
        self.sigma_col_bottom = np.array(bottom_column_buckling)

    def calculate_rib_spacing_skin(self):
        factor = (np.pi**2 * self.E_wing) / (12 * (1 - self.poisson_ratio_wing**2))
        
        self.rib_spacings = []
        self.rib_positions = []
        self.rib_span_indices = []

        x = 0.0  
        self.rib_positions.append(x)  
        for idx, y in enumerate(self.b_array):
            stress = abs(self.top_bending_stress_wing[idx])
            
            if abs(stress) < 1e6:
                b = 2.0
            else:
                b = self.skin_thicknesses[idx] * np.sqrt(self.C * factor / stress)
            self.rib_spacings.append(b)
            x += 2*b + 4*y**2

            if self.evaluate == EvaluateType.WING or self.evaluate == EvaluateType.HORIZONTAL:
                if x <= self.b/2:
                    self.rib_positions.append(x)
                    self.rib_span_indices.append(idx)
                else:
                    break 
            
            if self.evaluate == EvaluateType.VERTICAL:
                if x <= self.b:
                    self.rib_positions.append(x)
                    self.rib_span_indices.append(idx)
                else:
                    break
        
        self.rib_amount = len(self.rib_positions)

    def get_shear_buckling_thickness(self, tau_cr):
        b = self.cutout_spacing  
        factor = (12 * (1 - self.poisson_ratio_wing**2)) / (self.k_s * np.pi**2 * self.E_wing)

        t_required = b * np.sqrt(tau_cr * factor)
        return t_required
        
    def map_rib_thicknesses_to_positions(self):
        self.rib_thicknesses = []
        for pos in self.rib_positions:
            idx = np.argmin(np.abs(self.b_array - pos))  

            q_v = self.max_vertical_shear_flow[idx]
            if self.evaluate == EvaluateType.WING:
                q_h = self.max_horizontal_shear_flow[idx]
            else:
                q_h = 0.0
            d = self.cutout_diameter
            l_v = self.critical_spar_heights[idx]
            l_h = self.cutout_spacing
            tau_allowable = self.safety_factor * 0.577 * self.sigma_y
            tau_web = self.safety_factor*self.cr_web_stress[idx]
            tau_cr = min(tau_allowable, tau_web)

            t_v = (q_v * l_v) / (tau_cr * (l_v - d))
            t_h = (q_h * l_h) / (tau_cr * (l_h - d))

            t_buckling = self.get_shear_buckling_thickness(tau_cr)

            t_required = max(t_v, t_h, t_buckling)
            self.rib_thicknesses.append(t_required)

    def calculate_critical_buckling_stress(self):
        skin_stresses_top = []
        skin_stresses_bottom = []

        factor = self.C * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2)))

        for y in range(len(self.b_array)):
            skin_thickness = self.skin_thicknesses[y]
            spacing_factor_top = self.widths_top[y] - self.we2[y]
            if any(i <= 0.03 for i in spacing_factor_top):
                spacing_factor_top = [self.widths_top[y] - self.we2[y] if i > 0.03 else 1.5*self.widths_top[y] for i in spacing_factor_top]
            spacing_factor_bottom = self.widths_bottom[y] - self.we2[y]
            if any(i <=0.03 for i in spacing_factor_bottom):
                spacing_factor_bottom = [self.widths_bottom[y] - self.we2[y] if i > 0.03 else 1.5*self.widths_bottom[y] for i in spacing_factor_bottom]
            sigma_cr_top = factor * (skin_thickness / (spacing_factor_top))**2
            sigma_cr_bottom = factor * (skin_thickness / spacing_factor_bottom)**2
            buckling_stress_top_panel = ((self.stringer_area + self.we2[y]*skin_thickness)*self.cr_stringer + (abs(self.widths_top[y] - self.we2[y]))*skin_thickness*sigma_cr_top) / (self.stringer_area + (self.widths_top[y]*skin_thickness))
            buckling_stress_bottom_panel = ((self.stringer_area + self.we2[y]*skin_thickness)*self.cr_stringer + (abs(self.widths_bottom[y] - self.we2[y]))*skin_thickness*sigma_cr_bottom) / (self.stringer_area + (self.widths_bottom[y]*skin_thickness))
            skin_stresses_top.append(np.min(buckling_stress_top_panel))
            skin_stresses_bottom.append(np.min(buckling_stress_bottom_panel))
            
        self.buckling_stress_top = np.array(skin_stresses_top)
        self.buckling_stress_bottom = np.array(skin_stresses_bottom)
        
    def calculate_critical_web_stress(self):
        
        web_stresses = []
        for y in range(len(self.b_array)):
            shear_stress = []
            for i in range(self.n_cells+1):
                tau_cr = self.k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.spar_thicknesses[y] / self.spar_heights[y][i])**2
                shear_stress.append(tau_cr)
            web_stresses.append(min(shear_stress))
            self.cr_web_stress = np.array(web_stresses)
        return self.cr_web_stress
    
    def get_margins(self):

        relevant_stresses = [StressOutput.BENDING_STRESS, StressOutput.BENDING_STRESS_BOTTOM, StressOutput.SHEAR_STRESS, StressOutput.WING_BENDING_STRESS, StressOutput.INTERNAL_MOMENT_X]
        self.critical_margins = 0
        self.load_data[self.evaluate_case] = {}
        for i in relevant_stresses:
            output = self.get_output(i)
            main_stress = output['main']
            references = output['references']
            labels = output['labels']

            self.load_data[self.evaluate_case][i] = {
                'main': main_stress,
                'references': references,
                'labels': labels,
                'unit': output['unit']
            }
            
            for idx,ref in enumerate(references):
                margin = np.min(abs(ref)/abs(main_stress))
                if margin < 1:
                    self.plot_any(i)
                    print(f'Warning for {self.evaluate.name}: Margin for {i.name} (case={self.evaluate_case}) is below 1: {margin} ({labels[0]} vs {labels[1:][idx]})')
                    self.critical_margins += 1
                #print(f'Margin for {i.name} (n={self.load_factor:.2f}): {margin} ({labels[0]} vs {labels[1:][idx]})')
                self.margins[f"{i.name.lower()}_vs_{labels[1:][idx].replace(' ','_')}_{self.evaluate_case}"] = margin
            
        for i in relevant_stresses:
            self.margins[f'max_{i.name.lower()}'] = self.load_data['max'][i]['main'][0]

        self.margins[f'{self.evaluate_case}_moment_x'] = self.M_internal[0]
        if self.evaluate == EvaluateType.WING:
            self.margins[f'{self.evaluate_case}_moment_y'] = self.M_internal_y[0]
        self.margins[f'{self.evaluate_case}_torque'] = self.T_internal[0]
        self.margins[f'{self.evaluate_case}_vertical_shearforce'] = self.Vy_internal[0]

        if self.evaluate == EvaluateType.WING:
            # self.plot_any(StressOutput.DEFLECTION)
            # self.plot_any(StressOutput.TWIST)
            # self.plot_any(StressOutput.INTERNAL_SHEAR_VERTICAL)
            self.margins[f'{self.evaluate_case}_deflection'] = self.wing_deflection[-1]
            self.margins[f'{self.evaluate_case}_twist'] = self.twist[-1]
        if self.runs == 1 and self.PLOT:
            self.plot_output()
            self.plot_rib(id=0)
            self.plot_rib(id=len(self.rib_positions)-1)

        self.plot_wing_ribs()

        if self.runs == 1 and self.evaluate == EvaluateType.WING:
            self.update_attributes()
        else:
            self.update_attributes()

    def update_attributes(self):
        key_map = {
            EvaluateType.WING: 'wing_stress',
            EvaluateType.HORIZONTAL: 'horizontal_tail_stress',
            EvaluateType.VERTICAL: 'vertical_tail_stress'
        }
        component_weight_map = {
            EvaluateType.WING: 'wing',
            EvaluateType.HORIZONTAL: 'horizontal_tail',
            EvaluateType.VERTICAL: 'vertical_tail'
        }
        self.aircraft_data.data['outputs'][key_map[self.evaluate]] = self.margins
        self.aircraft_data.data['outputs'][key_map[self.evaluate]]['rib_amount'] = self.rib_amount

        key_string_map = {
            EvaluateType.WING: 'wing_box',
            EvaluateType.HORIZONTAL: 'horizontal_wing_box',
            EvaluateType.VERTICAL: 'vertical_wing_box'
        }

        epoxy_map = {
            EvaluateType.WING: 'epoxy_wing',
            EvaluateType.HORIZONTAL: 'epoxy_horizontal',
            EvaluateType.VERTICAL: 'epoxy_vertical'
        }
        key_string = key_string_map[self.evaluate]
        self.aircraft_data.data['outputs']['component_weights'][component_weight_map[self.evaluate]] = self.wing_mass*2*9.81 + self.epoxy_mass*2*9.81
        self.aircraft_data.data['inputs']['structures'][key_string]['Ip_MAC'] = self.MAC_Ip
        self.aircraft_data.data['inputs']['structures'][key_string]['J_MAC'] = self.MAC_J
        self.aircraft_data.data['inputs']['structures'][key_string]['section_mass_MAC'] = self.MAC_m
        self.aircraft_data.data['inputs']['structures'][key_string]['I_xx_MAC'] = self.I_xx_MAC
        self.aircraft_data.data['inputs']['structures'][key_string]['centroid_x'] = self.centroid_x_MAC
        # print(f'saving {key_map[self.evaluate]} design to {self.design_file}')
        self.aircraft_data.save_design(self.design_file)

    def get_output(self, output_type: StressOutput):
        output_map = {
            StressOutput.DEFLECTION: {
                'main': self.wing_deflection,
                'references': [np.full_like(self.wing_deflection, 0.15 * self.b/2)],
                'labels': ['Deflection', 'Max Allowable Deflection'],
                'unit': '[m]'
            },
            StressOutput.TWIST: {
                'main': self.twist,
                'references': [],
                'labels': ['Twist'],
                'unit': '[deg]'
            },
            StressOutput.BENDING_STRESS: {
                'main': self.top_bending_stress,
                'references': [
                    np.full_like(self.top_bending_stress, -self.sigma_y / 1e6)
                ] + (
                    [-self.sigma_col_top, -self.buckling_stress_top/1000000] if self.load_factor > 0 else []
                ),
                'labels': ['Top Bending Stress', 'Yield Stress']
                        + (['Column Buckling Stress', 'Buckling Stress'] if self.load_factor > 0 else []),
                'unit': '[MPa]'
            },
            StressOutput.BENDING_STRESS_BOTTOM: {
                'main': self.bottom_bending_stress,
                'references': [
                    np.full_like(self.bottom_bending_stress, self.sigma_y / 1e6)
                ],
                'labels': ['Bottom Bending Stress', 'Yield Stress'],
                'unit': '[MPa]'                 
            },
            StressOutput.SHEAR_STRESS: {
                'main': abs(self.shear_stress),
                'references': [0.577*self.sigma_y / 1e6 * np.ones_like(self.shear_stress), 
                              self.cr_web_stress/1000000],
                'labels': ['Shear Stress', 'Yield Stress', 'Critical Web Stress'],
                'unit': '[MPa]'
            },
            StressOutput.WING_BENDING_STRESS: {
                'main': self.top_bending_stress_wing/1000000,
                'references': [
                    np.full_like(self.top_bending_stress_wing, -self.sigma_y / 1e6)],
                'labels': ['Wing Bending Stress', 'Yield Stress'],
                'unit': '[MPa]'
            },
            StressOutput.TORSION: {
                'main': self.torsion,
                'references': [np.full_like(self.torsion, 0.1)],
                'labels': ['Torsion', 'Reference Limit'],
                'unit': '[MPa]'
            },
            StressOutput.RESULTANT_VERTICAL: {
                'main': self.vertical_distribution,
                'references': [],
                'labels': ['Resultant Vertical Load'],
                'unit': '[N/m]'
            },
            StressOutput.RESULTANT_TORQUE: {
                'main': self.torque_distribution,
                'references': [],
                'labels': ['Resultant Torque'],
                'unit': '[Nm/m]'
            },
            StressOutput.INTERNAL_SHEAR_VERTICAL: {
                'main': self.Vy_internal,
                'references': [],
                'labels': ['Internal Vertical Shear Force'],
                'unit': '[N/m]'
            },
            StressOutput.INTERNAL_MOMENT_X: {
                'main': self.M_internal,
                'references': [],
                'labels': ['Internal Bending Moment (X-axis)'],
                'unit': '[Nm/m]'
            },
            StressOutput.INTERNAL_TORQUE: {
                'main': self.T_internal,
                'references': [],
                'labels': ['Internal Torque'],
                'unit': '[Nm/m]'
            },
        }

        # Only include horizontal outputs if evaluate == EvaluateType.WING
        if self.evaluate == EvaluateType.WING:
            output_map.update({
                StressOutput.RESULTANT_HORIZONTAL: {
                    'main': self.horizontal_distribution,
                    'references': [],
                    'labels': ['Resultant Horizontal Load'],
                    'unit': '[N/m]'
                },
                StressOutput.INTERNAL_SHEAR_HORIZONTAL: {
                    'main': self.Vx_internal,
                    'references': [],
                    'labels': ['Internal Horizontal Shear Force'],
                    'unit': '[N]'
                },
                StressOutput.INTERNAL_MOMENT_Y: {
                    'main': self.M_internal_y,
                    'references': [],
                    'labels': ['Internal Bending Moment (Y-axis)'],
                    'unit': '[Nm]'
                },
                StressOutput.SHEAR_STRESS_TOP: {
                    'main': self.max_horizontal_shear_stress_top,
                    'references': [np.full_like(self.max_horizontal_shear_stress_top, 0.577*np.sign(self.load_factor)*self.sigma_y / 1e6)],
                    'labels': ['Horizontal Shear Stress', 'Yield Stress'],
                    'unit': '[MPa]'
                },
                StressOutput.SHEAR_STRESS_BOTTOM: {
                    'main': self.max_horizontal_shear_stress_bottom,
                    'references': [np.full_like(self.max_horizontal_shear_stress_bottom, 0.577*np.sign(self.load_factor)*self.sigma_y / 1e6)],
                    'labels': ['Horizontal Shear Stress', 'Yield Stress'],
                    'unit': '[MPa]'
                },
            })

        return output_map[output_type]

    def plot_output(self):
        plt.figure()
        if self.evaluate == EvaluateType.WING:
            color = {'max': 'blue', 'min': 'red'}
            n_dict = {'max': f'n={self.max_load_factor}', 'min': 'Buoy Slam'}
        else:
            color = {'max': 'blue'}
            if self.evaluate == EvaluateType.VERTICAL:
                n_dict = {'max': 'Max Sideslip and Rudder'}
            elif self.evaluate == EvaluateType.HORIZONTAL:
                n_dict = {'max': 'Max Trim and Elevator'}

        all_output_types = set()

        for output_dict in self.load_data.values():
            all_output_types.update(output_dict.keys())

        for output_type in all_output_types:
            for load_case in ['max', 'min']:
                output = self.load_data[load_case][output_type]
                main_line = output['main']
                references = output['references']
                labels = output['labels']
                unit = output['unit']

                plt.plot(self.b_array, main_line, label=f'{labels[0]} ({n_dict[load_case]})', color=color[load_case])
                for idx, ref in enumerate(references):
                    plt.plot(self.b_array, ref, label=f'{labels[1:][idx]} ({n_dict[load_case]})', linestyle='--',)

            plt.xlabel('Spanwise Position [m]')
            plt.ylabel(f'{output_type.name} {unit}')
            plt.title(f'{output_type.name} Distribution')
            plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            plt.grid()
            plt.tight_layout()
            plt.show()

    def plot_any(self, output_type: StressOutput):
        output = self.get_output(output_type)
        main_line = output['main']
        references = output['references']
        labels = output['labels']
        unit = output['unit']
        if self.evaluate == EvaluateType.WING:
            color = {'max': 'blue', 'min': 'red'}
            n_dict = {'max': f'n={self.max_load_factor}', 'min': 'Buoy Slam'}
        else:
            color = {'max': 'blue'}
            if self.evaluate == EvaluateType.VERTICAL:
                n_dict = {'max': 'Max Sideslip and Rudder'}
            elif self.evaluate == EvaluateType.HORIZONTAL:
                n_dict = {'max': 'Max Trim and Elevator'}
        plt.plot(self.b_array, main_line, label=f'{labels[0]} ({n_dict[self.evaluate_case]})', color='blue')
        for idx, ref in enumerate(references):
            plt.plot(self.b_array, ref, label=f'{labels[1:][idx]} ({n_dict[self.evaluate_case]})', linestyle='--')

        plt.xlabel('Spanwise Position [m]')
        if not self.evaluate == EvaluateType.VERTICAL:
            plt.ylabel(f'{output_type.name.lower().capitalize().replace('_', ' ')} {unit}')
        else:
            if output_type == StressOutput.RESULTANT_VERTICAL:
                plt.ylabel(f'Resultant lateral {unit}')
            else:
                plt.ylabel(f'{output_type.name.lower().capitalize().replace("_", " ")} {unit}')
        # plt.title(f'{output_type.name} Distribution')
        plt.grid()
        if output_type == StressOutput.DEFLECTION:
            plt.gca().set_aspect('equal')
        plt.tight_layout()
        # plt.legend(loc='best') 
        plt.grid(True)
        plt.show()

def main(all=True):
    stringer_material = Materials.Al7075
    wingbox_material = Materials.Al7075
    wing_material = Materials.Al5052
    
    if all:
        for evaluate in EvaluateType:

            stress_analysis = StressAnalysisWing(aircraft_data=Data("design3.json"),
                                                wingbox_mat=wingbox_material,
                                                wing_mat=wing_material, 
                                                stringer_mat=stringer_material,evaluate=evaluate,
                                                PLOT=False)
            
            stress_analysis.main_analysis()
            # stress_analysis.plot_any(StressOutput.RESULTANT_VERTICAL)
            flutter_analysis = FlutterAnalysis(aircraft_data=Data("design3.json"), wing_mat=wingbox_material, evaluate=evaluate)
            flutter_analysis.main(plot=False)
    else:
        stress_analysis = StressAnalysisWing(aircraft_data=Data("design3.json"), wingbox_mat=wingbox_material, wing_mat=wing_material, stringer_mat=stringer_material, evaluate=EvaluateType.HORIZONTAL, 
                                             PLOT=False)
        stress_analysis.main_analysis(run_all=True)
        stress_analysis.plot_any(StressOutput.RESULTANT_VERTICAL)
        stress_analysis.plot_any(StressOutput.INTERNAL_SHEAR_VERTICAL)
        stress_analysis.plot_any(StressOutput.INTERNAL_MOMENT_X)

        # stress_analysis.plot_any(StressOutput.INTERNAL_TORQUE)
        flutter_analysis = FlutterAnalysis(aircraft_data=Data("design3.json"), wing_mat=wingbox_material, evaluate=EvaluateType.WING)
        flutter_analysis.main(plot=True)
    
    return stress_analysis.critical_margins
        
if __name__ == "__main__":

    main(all=False)


    


        