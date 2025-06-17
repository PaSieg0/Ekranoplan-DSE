import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, EvaluateType
from numpy.polynomial import Polynomial
import numpy as np
from aero.lift_curve import lift_curve
from aero.lateral_centre import lateral_centre
import warnings
warnings.filterwarnings("ignore")
np.seterr(all='ignore')

class AerodynamicForces:

    def __init__(self, aircraft_data: Data, evaluate: EvaluateType = EvaluateType.WING):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.evaluate = evaluate

        self.aerodynamics = Data("AeroForces.txt", "aerodynamics")
        self.tail_aerodynamics = Data("TailForces.txt", "aerodynamics")
        self.vertical_tail_aerodynamics = Data("VerticalTailForces.txt", "aerodynamics")

        self.tailspan_values = self.tail_aerodynamics.data['yspan']
        self.tail_cl_values = self.tail_aerodynamics.data['cl']
        self.tail_induced_cd_values = self.tail_aerodynamics.data['induced_cd']
        self.tail_chord_values = self.tail_aerodynamics.data['chord']
        self.tail_cm_values = self.tail_aerodynamics.data['cm']

        self.vertical_tailspan_values = self.vertical_tail_aerodynamics.data['yspan']
        self.vertical_tail_cl_values = self.vertical_tail_aerodynamics.data['cl']
        self.vertical_tail_induced_cd_values = self.vertical_tail_aerodynamics.data['induced_cd']
        self.vertical_tail_chord_values = self.vertical_tail_aerodynamics.data['chord']
        self.vertical_tail_cm_values = self.vertical_tail_aerodynamics.data['cm']
        self.Cm = -0.40226
        self.yspan_values = self.aerodynamics.data['yspan']
        self.cl_values = self.aerodynamics.data['cl']
        self.induced_cd_values = self.aerodynamics.data['induced_cd']
        self.chord_values = self.aerodynamics.data['chord']
        self.cm_values = self.aerodynamics.data['cm']

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.b_array = np.arange(0, self.b/2+0.01, 0.01)
        self.Sr = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['Sr']

        self.b_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.b_h_array = np.arange(0, self.b_h/2+0.01, 0.01)

        self.b_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        self.b_v_array = np.arange(0, self.b_v+0.01, 0.01)

        self.Sh = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']
        self.Sv = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S']
        self.S = self.aircraft_data.data['outputs']['wing_design']['S']

        self.aileron_start = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b1']
        self.aileron_end = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b2']
        self.aileroned_area = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['Swa']

        self.elevator_start1 = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1_s1']
        self.elevator_end1 = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2_s1']
        self.elevator_start2 = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1_s2']
        self.elevator_end2 = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2_s2']
        self.elevatored_area = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['Se']
        self.elevator_array = np.arange(self.elevator_start1, self.elevator_end2+0.01, 0.01)

        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.aileron_array = np.arange(self.aileron_start, self.aileron_end+0.01, 0.01)

        self.l_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']

        self.rho = self.aircraft_data.data['rho_air']
        self.V = self.aircraft_data.data['requirements']['cruise_speed']

        self.cd0 = self.aircraft_data.data['inputs']['Cd0']

        self.lift_curve = lift_curve()
        self.tail_lift_slope = self.lift_curve.dcl_dalpha()[0]

        self.airfoil_Cd0 = self.aircraft_data.data['inputs']['airfoils']['cd0_wing']
        self.aileron_lift = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['aileron_lift']
        self.elevator_lift = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['elevator_lift']
        self.rudder_lift = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['rudder_lift']
        self.max_side_slip = self.aircraft_data.data['inputs']['max_sideslip']

        self.vertical_lift_coeff = -self.tail_lift_slope * self.max_side_slip/2*self.Sv/self.S
        self.rudder_coeff = self.rudder_lift / (0.5 * self.rho * self.V**2 * self.S)
        self.elevator_coeff = self.elevator_lift / (0.5 * self.rho * self.V**2 * self.Sh/2)

        self.rudder_start = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b1']
        self.rudder_end = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b2']
        self.rudder_array = np.arange(self.rudder_start, self.rudder_end+0.01, 0.01)
        
        self.aspect_ratio = self.aircraft_data.data['outputs']['wing_design']['aspect_ratio']
        self.oswald_factor = self.aircraft_data.data['inputs']['oswald_factor']

        self.l_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']
        self.l_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']

        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.xlemac = self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.xcg_aft = self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']

        self.lift_arm = self.xcg_aft - (self.xlemac + 0.25*self.MAC) 

        self.MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        self.CL_max = self.aircraft_data.data['inputs']['CLmax_landing']
        self.V_land = self.aircraft_data.data['requirements']['stall_speed_landing']*1.05

        self.L_max = 0.5 * self.rho * self.V_land**2 * self.CL_max * self.S/2
        self.L_nominal = self.MTOW / 2 

        self.CL_nominal = self.L_nominal / (0.5 * self.rho * self.V**2 * self.S)

        self.lateral_centre = lateral_centre()

        self.get_aileron_lift_distribution()
        self.get_elevator_lift_distribution()

        self.k = self.aircraft_data.data['outputs']['design']['k']

    def chord_span_function_aero(self,y):
        return self.chord_root + (self.chord_tip - self.chord_root) / (self.b/2) * y
    
    def get_aileron_lift_distribution(self):
        self.aileron_lift_array = np.zeros(len(self.b_array))
        self.aileron_lift_dist = self.aileron_lift/(self.aileroned_area)*self.chord_span_function_aero(self.aileron_array)
        for id, i in enumerate(self.aileron_lift_dist):
            idx = np.argmin(np.abs(self.b_array - self.aileron_array[id]))
            self.aileron_lift_array[idx] = i
        return self.aileron_lift_array

    def get_elevator_lift_distribution(self):
        self.elevator_lift_array = np.zeros(len(self.b_h_array))
        self.elevator_lift_dist = self.elevator_lift/(self.elevatored_area)*self.chord_span_h_function(self.elevator_array)
        for id, i in enumerate(self.elevator_lift_dist):
            idx = np.argmin(np.abs(self.b_h_array - self.elevator_array[id]))
            self.elevator_lift_array[idx] = i

    def elliptic_lift_distribution(self, y_vals, span, CL):
        elliptical_distribution = np.sqrt(1 - (y_vals / span)**2)
  
        CL0 = (4 * CL) / (np.pi * span)
        
        self.CL_y = CL0 * elliptical_distribution
        self.CL_y = np.nan_to_num(self.CL_y)

        # print(np.trapz(self.CL_y, y_vals))
        return self.CL_y
    
    def elliptic_drag_distribution(self, y_vals, span, CL, V):
        CDi = (CL**2) / (np.pi * self.aspect_ratio * self.oswald_factor)

        Cd_local = self.cd0 + CDi

        CD0 = (4* Cd_local) / (np.pi * span)

        self.CD_y = CD0 * np.sqrt(1 - (y_vals / span)**2)
        self.CD_y = np.nan_to_num(self.CD_y)
        
        return self.CD_y
    
    def static_moment_distribution(self):

        CM0 = (4 * self.Cm) / (np.pi * self.b/2)
        self.Cm_y = CM0 * np.sqrt(1 - (self.b_array / (self.b/2))**2)
        self.Cm_y = np.nan_to_num(self.Cm_y)
        # print(np.trapz(self.Cm_y, self.b_array))

        return self.Cm_y

    def plot_Lift_distribution(self):
        plt.figure()
        plt.plot(self.b_array, self.L_y, label="Fitted L")
        plt.xlabel("y-span")
        plt.ylabel("L [N]")
        plt.title("L Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_Drag_distribution(self):
        plt.figure()
        plt.plot(self.b_array, self.D_y, label="Fitted Cd")
        plt.xlabel("y-span")
        plt.ylabel("D [N]")
        plt.title("D Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_Moment_distribution(self):
        plt.figure()
        plt.plot(self.b_array, self.M_y)
        plt.xlabel("y-span")
        plt.ylabel("M [Nm]")
        plt.title("Moment Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.show()
    
    def get_max_aero_dist(self):
        # self.L_y = self.elliptic_lift_distribution(self.b_array, self.b/2, self.CL_max)* 0.5*self.rho*self.V_land**2*self.S
        y_vals, lift_vals, moment_vals, drag_vals = self.lateral_centre.determine_distr(self.b/2, self.chord_root, self.chord_tip, self.rho, self.V_land, self.CL_max)
        poly_lift = Polynomial.fit(y_vals, lift_vals, 8)
        self.L_y = poly_lift(self.b_array) + self.get_aileron_lift_distribution()
        poly_drag = Polynomial.fit(y_vals, drag_vals, 8)
        self.D_y = poly_drag(self.b_array) 
        poly_moment = Polynomial.fit(y_vals, moment_vals, 8)
        self.M_y = poly_moment(self.b_array) 
        return self.L_y
    
    def get_nominal_aero_dist(self):
        y_vals, lift_vals, moment_vals, drag_vals = self.lateral_centre.determine_distr(
            self.b/2, self.chord_root, self.chord_tip, self.rho, self.V, self.CL_nominal
        )
        poly_lift = Polynomial.fit(y_vals, lift_vals, 8)
        self.L_y = poly_lift(self.b_array)
        poly_drag = Polynomial.fit(y_vals, drag_vals, 8)
        self.D_y = poly_drag(self.b_array)
        poly_moment = Polynomial.fit(y_vals, moment_vals, 8)
        self.M_y = poly_moment(self.b_array)
        return self.L_y

    def moment_distribution(self):
        self.moment_values = []
        for cm, c in zip(self.cm_values, self.chord_values):
            m = cm * (0.5*self.rho*self.V**2*c**2)
            self.moment_values.append(m)

    def get_moment_function(self):
        self.moment_distribution()
        self.M_y = Polynomial.fit(self.yspan_values, self.moment_values, 10)
        self.fitted_M = self.M_y(self.b_array)
        return self.M_y
    
    def get_vertical_tail_lift_distribution(self):
        self.vertical_tail_lift_function = self.elliptic_lift_distribution(self.b_v_array, self.b_v, self.vertical_lift_coeff) * 0.5 * self.rho * self.V**2 * self.Sv
        
        self.rudder_lift_array = np.zeros(len(self.b_v_array))
        for i, pos in enumerate(self.b_v_array):
            if self.rudder_start <= pos <= self.rudder_end:
                idx = np.argmin(np.abs(self.b_v_array - pos))
                self.rudder_lift_array[idx] = self.rudder_lift/(self.Sr) * self.chord_span_v_function(pos)
        
        self.vertical_tail_lift = self.vertical_tail_lift_function + self.rudder_lift_array
        return self.vertical_tail_lift
    
    def get_horizontal_tail_lift_distribution(self):
        self.horizontal_tail_lift = (self.L_y * self.lift_arm + self.M_y)/self.l_h * self.Sh/self.S
        CL_horizontal = self.horizontal_tail_lift / (0.5 * self.rho * self.V_land**2 * self.Sh)
        CL_tot_horizontal = np.trapz(CL_horizontal, self.b_array)
        self.horizontal_tail_lift = self.elliptic_lift_distribution(self.b_h_array, self.b_h/2, CL_tot_horizontal) * 0.5 * self.rho * self.V**2 * self.Sh
        for i, pos in enumerate(self.b_h_array):
            if self.elevator_start1 <= pos <= self.elevator_end1 or self.elevator_start2 <= pos <= self.elevator_end2:
                idx = np.argmin(np.abs(self.b_h_array - pos))
                self.horizontal_tail_lift[idx] -= self.elevator_lift_array[i]

        return self.horizontal_tail_lift

    def chord_span_h_function(self,y):
        return self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root'] + \
               (self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip'] - \
                self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']) / \
               (self.b_h/2) * y
    
    def chord_span_v_function(self,y):
        return self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root'] + \
               (self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip'] - \
                self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']) / \
               (self.b_v) * y
    
    def plot_horizontal_tail_lift_distribution(self):
        plt.figure()
        plt.plot(self.b_h_array, self.horizontal_tail_lift, label="Horizontal Tail Lift Distribution")
        plt.xlabel("y-span")
        plt.ylabel("L [N]")
        plt.title("Horizontal Tail Lift Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.show()

    def plot_vertical_tail_lift_distribution(self):
        plt.figure()
        plt.plot(self.b_v_array, self.vertical_tail_lift, label="Vertical Tail Lift Distribution")
        plt.xlabel("y-span")
        plt.ylabel("L [N]")
        plt.title("Vertical Tail Lift Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    aerodynamics_data = Data("AeroForces.txt", "aerodynamics")
    aircraft_data = Data("design3.json", "design")
    evaluate = EvaluateType.WING
    aeroforces = AerodynamicForces(aircraft_data,evaluate= evaluate)

    L_y = aeroforces.get_max_aero_dist()
    aeroforces.get_horizontal_tail_lift_distribution()
    aeroforces.get_vertical_tail_lift_distribution()

    aeroforces.plot_Lift_distribution()
    aeroforces.plot_Drag_distribution()
    aeroforces.plot_Moment_distribution()
    aeroforces.plot_horizontal_tail_lift_distribution()
    aeroforces.plot_vertical_tail_lift_distribution()

#    def lift_distribution(self):

#         self.lift_values = []
#         for cl, c in zip(self.cl_values, self.chord_values):
#             if cl < 0 or c < 0:
#                 raise ValueError("Cl values must be non-negative.")
#             else:
#                 l = cl * (0.5*self.rho*self.V**2*c)
#                 self.lift_values.append(l)

#     def drag_distribution(self):
#         self.drag_values = []
#         for cdi, c in zip(self.induced_cd_values, self.chord_values):
#             idx = self.induced_cd_values.index(cdi)
#             if cdi < 0 or c < 0:
#                 raise ValueError("Cl values must be non-negative.")
#             else:
#                 cd = self.airfoil_Cd0 + cdi 
#                 d = cd * (0.5*self.rho*self.V**2*c)
#                 self.drag_values.append(d)


        # def get_aero_lift_function(self):
        #     self.lift_distribution()
        #     self.L_y = Polynomial.fit(self.yspan_values, self.lift_values, 9)
        #     self.fitted_l = self.L_y(self.b_array)
        #     if self.evaluate == EvaluateType.WING:
        #         self.fitted_l += self.aileron_lift_array
        #     return self.L_y
        
        # def get_aero_drag_function(self):
        #     self.drag_distribution()
        #     self.D_y = Polynomial.fit(self.yspan_values, self.drag_values, 10)
        #     self.fitted_d = self.D_y(self.b_array)
        #     return self.D_y

    # def vertical_tail_lift_distribution(self):
    #     self.vertical_tail_lift_values = []
    #     for cl, c in zip(self.vertical_tail_cl_values, self.vertical_tail_chord_values):
    #         l = cl * (0.5*self.rho*self.V**2*c)
    #         self.vertical_tail_lift_values.append(l)

    # def vertical_tail_drag_distribution(self):
    #     self.vertical_tail_drag_values = []
    #     for cdi, c in zip(self.vertical_tail_induced_cd_values, self.vertical_tail_chord_values):
    #         if cdi < 0 or c < 0:
    #             raise ValueError("Cd values must be non-negative.")
    #         else:
    #             cd = self.airfoil_Cd0 + cdi 
    #             d = cd * (0.5*self.rho*self.V**2*c)
    #             self.vertical_tail_drag_values.append(d)

    # def get_vertical_tail_drag_aero_function(self):
    #     self.vertical_tail_drag_distribution()
    #     self.vertical_tail_D_y = Polynomial.fit(self.vertical_tailspan_values, self.vertical_tail_drag_values, 10)
    #     self.fitted_vertical_tail_d = self.vertical_tail_D_y(self.b_v_array)
    #     return self.vertical_tail_D_y

    # def get_vertical_tail_lift_aero_function(self):
    #     self.vertical_tail_lift_distribution()
    #     self.vertical_tail_L_y = Polynomial.fit(self.vertical_tailspan_values, self.vertical_tail_lift_values, 9)
    #     self.fitted_vertical_tail_l = self.vertical_tail_L_y(self.b_v_array)
    #     return self.vertical_tail_L_y
