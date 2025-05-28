import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, StressOutput
from AerodynamicForces import AerodynamicForces
from WingStructure import WingStructure
from scipy.integrate import quad
import numpy as np

class StressAnalysisWing(AerodynamicForces, WingStructure):

    def __init__(self, aircraft_data: Data, airfoil_aerodynamics: Data, airfoil_data: Data, PLOT: bool = False):
        AerodynamicForces.__init__(self, aircraft_data, airfoil_aerodynamics)
        WingStructure.__init__(self, aircraft_data, airfoil_data)
        self.get_wing_structure()
        self.safety_factor = 1.5
        self.lift_function = self.get_lift_function()
        self.drag_function = self.get_drag_function()
        self.moment_function = self.get_moment_function()

        self.PLOT = PLOT

        self.engine_power = self.aircraft_data.data['inputs']['engine']['engine_power']
        self.engine_thrust = self.engine_power / self.V
        self.dy = np.gradient(self.b_array)
        self.G = self.material['G']
        self.E = self.material['E']
        self.max_load_factor = self.aircraft_data.data['outputs']['general']['nmax']
        self.min_load_factor = self.aircraft_data.data['outputs']['general']['nmin']
        self.evaluate_case = 'max'
        self.drag_array = -self.drag_function(self.b_array)
        self.I_xx_array = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
        self.I_yy_array = np.array([self.wing_structure[i]['I_yy'] for i in range(len(self.wing_structure))])
        self.I_xy_array = np.array([self.wing_structure[i]['I_xy'] for i in range(len(self.wing_structure))])
        self.internal_torque()
        self.sigma_y = self.material['sigma_y']
        self.k_v = 1.5 
        self.spar_heights = np.array([self.wing_structure[i]['spar_info']['spar_heights'] for i in range(len(self.wing_structure))])
        self.poisson_ratio = self.material['poisson_ratio']
        self.l_stringer = self.b/2
        self.widths_top = np.array([self.wing_structure[i]['panel_info']['widths_top'] for i in range(len(self.wing_structure))])
        self.widths_bottom = np.array([self.wing_structure[i]['panel_info']['widths_bottom'] for i in range(len(self.wing_structure))])

    def main_analysis(self):
        self.resultant_horizontal_distribution()
        self.resultant_vertical_distribution()
        self.internal_vertical_shear_force()
        self.internal_horizontal_shear_force()
        self.internal_bending_moment_x()
        self.internal_bending_moment_y()
        self.internal_torque()
        self.calculate_torsion()
        self.calculate_top_bending_stress()
        self.calculate_bottom_bending_stress()
        self.calculate_wing_deflection()
        self.calculate_tot_shear_stress()
        self.calculate_critical_buckling_stress()
        self.calculate_critical_web_stress()
        self.column_buckling()
        self.calculate_horizontal_shear_stress()
        self.get_margins()

    def resultant_vertical_distribution(self):
        if self.evaluate_case == 'max':
            self.vertical_distribution = self.max_load_factor*self.lift_function(self.b_array) - self.wing_weight_dist()
            return self.vertical_distribution
        elif self.evaluate_case == 'min':
            self.vertical_distribution = self.min_load_factor*self.lift_function(self.b_array) + self.wing_weight_dist()
            return self.vertical_distribution
    
    def resultant_torque_distribution(self):
        quarter_chord_dist = [self.wing_structure[i]['quarter_chord_dist'] for i in range(len(self.wing_structure))]
        if self.evaluate_case == 'max':
            self.torque_distribution = self.max_load_factor*self.lift_function(self.b_array) * quarter_chord_dist + self.moment_function(self.b_array)
            return self.torque_distribution
        elif self.evaluate_case == 'min':
            self.torque_distribution = self.min_load_factor*self.lift_function(self.b_array) * quarter_chord_dist + self.moment_function(self.b_array)
            return self.torque_distribution
        
    def resultant_horizontal_distribution(self):
        self.horizontal_distribution = self.drag_array.copy()
        for engine_y in self.engine_positions:
            self.horizontal_distribution[int(round(engine_y,2)*100)] += self.engine_thrust

        return self.horizontal_distribution

    def internal_vertical_shear_force(self):

        load = self.resultant_vertical_distribution()

        Vy_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.Vy_internal = -Vy_flipped[::-1]

        return self.Vy_internal
    
    def internal_horizontal_shear_force(self):
        engine_load = np.zeros_like(self.b_array)
        for engine_y in self.engine_positions:
            engine_load[int(round(engine_y,2)*100)] = self.engine_thrust
        Vx_flipped_engines = np.cumsum(engine_load[::-1] * self.dy[::-1])

        load = self.drag_array
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
        T_flipped = np.cumsum(load[::-1] * self.dy[::-1])
        self.T_internal = T_flipped[::-1]
        return self.T_internal
    
    def calculate_top_bending_stress(self): 
        y = [self.wing_structure[i]['spar_info']['max_y'] - self.wing_structure[i]['centroid'][1] for i in range(len(self.wing_structure))]
        x = [self.wing_structure[i]['centroid'][0] -self.wing_structure[i]['spar_info']['max_x'] for i in range(len(self.wing_structure))]
        self.top_bending_stress = self.safety_factor*((self.internal_bending_moment_x()*self.I_yy_array -(self.internal_bending_moment_y()*self.I_xy_array))*y + (self.internal_bending_moment_y()*self.I_xx_array - (self.internal_bending_moment_x()*self.I_xy_array))*x) / (self.I_xx_array*self.I_yy_array - self.I_xy_array**2)/1000000
        return self.top_bending_stress
    
    def calculate_bottom_bending_stress(self):
        y = [self.wing_structure[i]['spar_info']['min_y'] - self.wing_structure[i]['centroid'][1] for i in range(len(self.wing_structure))]
        x = [self.wing_structure[i]['centroid'][0] - self.wing_structure[i]['spar_info']['min_x'] for i in range(len(self.wing_structure))]
        self.bottom_bending_stress = self.safety_factor*((self.internal_bending_moment_x()*self.I_yy_array -(self.internal_bending_moment_y()*self.I_xy_array))*y + (self.internal_bending_moment_y()*self.I_xx_array - (self.internal_bending_moment_x()*self.I_xy_array))*x) / (self.I_xx_array*self.I_yy_array - self.I_xy_array**2)/1000000
        return self.bottom_bending_stress
    
    def calculate_dtheta_dy(self,idx):
        coefficient_matrix = np.zeros((self.n_cells+1, self.n_cells+1))
        spar_heights = self.wing_structure[idx]['spar_info']['spar_heights']
        top_skin_lengths = self.wing_structure[idx]['panel_info']['top_skin_length']
        bottom_skin_lengths = self.wing_structure[idx]['panel_info']['bottom_skin_length']
        cell_areas = self.wing_structure[idx]['cell_areas']
        torque = self.T_internal[idx]

        for i in range(self.n_cells):
            area_factor = 1/(2*cell_areas[i])
            if i == 0:
                coefficient_left_spar = spar_heights[i] / (self.G*self.t_spar)
                coefficient_top = top_skin_lengths[i] / (self.G*self.t_skin)
                coefficient_bottom = bottom_skin_lengths[i] / (self.G*self.t_skin)
                coefficient_right_spar2 = -spar_heights[i+1] / (self.G*self.t_spar)
                coefficient_right_spar1 = spar_heights[i+1] / (self.G*self.t_spar)

                coefficient_matrix[i, 0] = (coefficient_left_spar + coefficient_top + coefficient_bottom + coefficient_right_spar1)*area_factor
                coefficient_matrix[i, i+1] = coefficient_right_spar2*area_factor
                coefficient_matrix[i,-1] = -1
                coefficient_matrix[-1, i] = 1/area_factor

            elif i == self.n_cells:
                coefficient_left_spar1 = -spar_heights[i] / (self.G*self.t_spar)
                coefficient_left_spar2 = spar_heights[i] / (self.G*self.t_spar)
                coefficient_top = top_skin_lengths[i] / (self.G*self.t_skin)
                coefficient_bottom = bottom_skin_lengths[i] / (self.G*self.t_skin)

                coefficient_matrix[i, i-1] = coefficient_left_spar1*area_factor
                coefficient_matrix[i, i] = (coefficient_left_spar2 + coefficient_top + coefficient_bottom)*area_factor
                coefficient_matrix[i, -1] = -1
                coefficient_matrix[-1, i] = 1/area_factor

            else:
                coefficient_left_spar1 = -spar_heights[i] / (self.G*self.t_spar)
                coefficient_left_spar2 = spar_heights[i] / (self.G*self.t_spar)
                coefficient_top = top_skin_lengths[i] / (self.G*self.t_skin)
                coefficient_bottom = bottom_skin_lengths[i] / (self.G*self.t_skin)
                coefficient_right_spar1 = spar_heights[i+1] / (self.G*self.t_spar)
                coefficient_right_spar2 = -spar_heights[i+1] / (self.G*self.t_spar)

                coefficient_matrix[i, i-1] = coefficient_left_spar1*area_factor
                coefficient_matrix[i, i] = (coefficient_left_spar2 + coefficient_top + coefficient_bottom + coefficient_right_spar1)*area_factor
                coefficient_matrix[i, i+1] = coefficient_right_spar2*area_factor
                coefficient_matrix[i, -1] = -1
                coefficient_matrix[-1, i] = 1/area_factor

        y = np.zeros(self.n_cells+1)
        y[-1] = torque

        solution = np.linalg.solve(coefficient_matrix, y)

        max_stress = max([i for i in solution[:-1]])
        dtheta_dy = np.rad2deg(solution[-1])
        return max_stress, dtheta_dy

    def calculate_shear_stress(self):
        Vy_internal = self.Vy_internal
        avg_shear_stress = Vy_internal / [sum(self.wing_structure[i]['spar_info']['spar_heights_t']) for i in range(len(self.wing_structure))]
        self.max_shear_stress = self.safety_factor*avg_shear_stress * self.k_v/1000000
        return self.max_shear_stress
    
    def calculate_horizontal_shear_stress(self):
        Vx_internal = self.Vx_internal
        avg_shear_stress_top = Vx_internal / [sum(self.wing_structure[i]['panel_info']['top_skin_length'])*self.t_skin for i in range(len(self.wing_structure))]
        avg_shear_stress_bot = Vx_internal / [sum(self.wing_structure[i]['panel_info']['bottom_skin_length'])*self.t_skin for i in range(len(self.wing_structure))]
        self.max_horizontal_shear_stress_top = self.safety_factor*avg_shear_stress_top * self.k_v/1000000
        self.max_horizontal_shear_stress_bottom = self.safety_factor*avg_shear_stress_bot * self.k_v/1000000
        return self.max_horizontal_shear_stress_top, self.max_horizontal_shear_stress_bottom
    
    def calculate_torsion(self):
        self.torsion = np.zeros(len(self.b_array))
        self.dtheta_dy = np.zeros(len(self.b_array))
        for i in range(len(self.b_array)):
            stress, dtheta_dy = self.calculate_dtheta_dy(i)
            self.torsion[i] = self.safety_factor*stress/1000000
            self.dtheta_dy[i] = dtheta_dy

        self.twist = np.cumsum(self.dtheta_dy * self.dy)
        return self.torsion, self.twist
    
    def calculate_tot_shear_stress(self):
        self.shear_stress = self.calculate_shear_stress() + self.calculate_torsion()[0]
        return self.shear_stress

    def calculate_wing_deflection(self):
        moment = self.M_internal
        moment_flipped = np.cumsum(moment[::-1]*self.dy[::-1])
        self.wing_deflection = np.cumsum(moment_flipped*self.dy[::-1]) / (-self.E * self.I_xx_array[::-1])

        return self.wing_deflection

    def column_buckling(self):
         k = 1/4 # for one fixed, one free end
         stringer_I_xx_top = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
         stringer_I_xx_bottom = np.array([self.wing_structure[i]['I_xx'] for i in range(len(self.wing_structure))])
         self.sigma_col_top = (k * np.pi**2 * self.E_stringer * stringer_I_xx_top) / (self.l_stringer**2 * self.stringer_area)/1000000
         self.sigma_col_bottom = (k * np.pi**2 * self.E_stringer * stringer_I_xx_bottom) / (self.l_stringer**2 * self.n_stringers/2*self.stringer_area)/1000000
     

    def calculate_critical_buckling_stress(self):
        # b spacing between stringers, only subtract 2we once and use areabalance
        skin_stresses_top = []
        skin_stresses_bottom = []

        b = 1 #TODO:Change this

        factor = self.C * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2)))

        for y in range(len(self.b_array)):
            sigma_cr_top = factor * (self.t_skin / self.widths_top[y])**2
            sigma_cr_bottom = factor * (self.t_skin / self.widths_bottom[y])**2
            skin_stresses_top.append(np.min(sigma_cr_top))
            skin_stresses_bottom.append(np.min(sigma_cr_bottom))

        self.critical_buckling_stress_top = np.array(skin_stresses_top)
        self.critical_buckling_stress_bottom = np.array(skin_stresses_bottom)

        self.buckling_stress_top_panel = ((self.stringer_area + self.stringer_width*self.t_skin)*self.crippling_stress_stringer + (b - self.stringer_width)*self.t_skin*self.critical_buckling_stress_top) / (self.stringer_area + (b*self.t_skin))
        self.buckling_stress_bottom_panel = ((self.stringer_area + self.stringer_width*self.t_skin)*self.crippling_stress_stringer + (b - self.stringer_width)*self.t_skin*self.critical_buckling_stress_bottom) / (self.stringer_area + (b*self.t_skin))


        return self.buckling_stress_top_panel, self.buckling_stress_bottom_panel
    
    def calculate_critical_web_stress(self):
      
        k_s = 10
        
        web_stresses = []
        for y in range(len(self.b_array)):
            shear_stress = []
            for i in range(self.n_cells+1):
                tau_cr = k_s * ((np.pi**2 * self.E) / (12 * (1 - self.poisson_ratio**2))) * (self.t_spar / self.spar_heights[y][i])**2
                shear_stress.append(tau_cr)
            web_stresses.append(min(shear_stress))
            self.cr_web_stress = np.array(web_stresses)/1000000
        return self.cr_web_stress
    
    def get_margins(self):

        relevant_stresses = [StressOutput.BENDING_STRESS, StressOutput.BENDING_STRESS_BOTTOM, StressOutput.SHEAR_STRESS]

        for i in relevant_stresses:
            if self.PLOT == True:
                self.plot(i)
            output = self.get_output(i)
            main_stress = output['main']
            references = output['references']
            labels = output['labels']

            for idx,ref in enumerate(references):
                margin = np.min(abs(ref)/abs(main_stress))
                print(f'Margin for {i.name}: {margin} ({labels[0]} vs {labels[1:][idx]})')

    def get_output(self, output_type: StressOutput):
        output_map = {
            StressOutput.DEFLECTION: {
                'main': self.wing_deflection,
                'references': [np.full_like(self.wing_deflection, 0.1 * self.b / 2)],
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
                    np.full_like(self.top_bending_stress, -self.sigma_y / 1e6),
                    -self.sigma_col_top,
                    -self.buckling_stress_top
                ],
                'labels': ['Top Bending Stress', 'Yield Stress', 'Column Buckling Stress', 'Critical Skin Buckling Stress'],
                'unit': '[MPa]'
            },
            StressOutput.BENDING_STRESS_BOTTOM: {
                'main': self.bottom_bending_stress,
                'references': [np.full_like(self.bottom_bending_stress, self.sigma_y / 1e6)],
                'labels': ['Bottom Bending Stress', 'Yield Stress'],
                'unit': '[MPa]'
            },
            StressOutput.SHEAR_STRESS: {
                'main': self.shear_stress,
                'references': [np.full_like(self.shear_stress, -self.sigma_y / 1e6), -self.cr_web_stress],
                'labels': ['Shear Stress', 'Yield Stress', 'Critical Web Stress'],
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
            StressOutput.RESULTANT_HORIZONTAL: {
                'main': self.horizontal_distribution,
                'references': [],
                'labels': ['Resultant Horizontal Load'],
                'unit': '[N/m]'
            },
            StressOutput.INTERNAL_SHEAR_VERTICAL: {
                'main': self.Vy_internal,
                'references': [],
                'labels': ['Internal Vertical Shear Force'],
                'unit': '[N]'
            },
            StressOutput.INTERNAL_SHEAR_HORIZONTAL: {
                'main': self.Vx_internal,
                'references': [],
                'labels': ['Internal Horizontal Shear Force'],
                'unit': '[N]'
            },
            StressOutput.INTERNAL_MOMENT_X: {
                'main': self.M_internal,
                'references': [],
                'labels': ['Internal Bending Moment (X-axis)'],
                'unit': '[Nm]'
            },
            StressOutput.INTERNAL_MOMENT_Y: {
                'main': self.M_internal_y,
                'references': [],
                'labels': ['Internal Bending Moment (Y-axis)'],
                'unit': '[Nm]'
            },
            StressOutput.INTERNAL_TORQUE: {
                'main': self.T_internal,
                'references': [],
                'labels': ['Internal Torque'],
                'unit': '[Nm]'
            },
            StressOutput.SHEAR_STRESS_TOP: {
                'main': self.max_horizontal_shear_stress_top,
                'references': [np.full_like(self.max_horizontal_shear_stress_top, self.sigma_y / 1e6)],
                'labels': ['Horizontal Shear Stress', 'Yield Stress'],
                'unit': '[MPa]'
            },
            StressOutput.SHEAR_STRESS_BOTTOM: {
                'main': self.max_horizontal_shear_stress_bottom,
                'references': [np.full_like(self.max_horizontal_shear_stress_bottom, self.sigma_y / 1e6)],
                'labels': ['Horizontal Shear Stress', 'Yield Stress'],
                'unit': '[MPa]'
            },
        }

        return output_map[output_type]



    def plot(self, output_type: StressOutput):
        plt.figure()
        
        output = self.get_output(output_type)

        main_line = output['main']
        references = output.get('references', [])
        labels = output.get('labels', [])

        plt.plot(self.b_array, main_line, label=labels[0], color='blue')
        for idx,i in enumerate(references):
            plt.plot(self.b_array, i, label=labels[idx+1] if output.get('labels') else 'Reference', linestyle='--')
        plt.xlabel('Spanwise Position (m)')
        plt.ylabel(output_type.name.replace('_', ' ').title())
        plt.title(f'{output_type.name.replace("_", " ").title()} Distribution')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()



if __name__ == "__main__":

    stress_analysis = StressAnalysisWing(
        aircraft_data=Data("design3.json"),
        airfoil_aerodynamics=Data("AeroForces.txt", 'aerodynamics'),
        airfoil_data=Data("Airfoil_data.dat", 'airfoil_geometry'),
        PLOT=True

    )

    stress_analysis.main_analysis()


    


        