import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data, EvaluateType
from numpy.polynomial import Polynomial
import numpy as np


class AerodynamicForces:

    def __init__(self, aircraft_data: Data, airfoil_aerodynamics: Data, evaluate: EvaluateType):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.evaluate = evaluate

        self.aerodynamics = airfoil_aerodynamics

        self.yspan_values = self.aerodynamics.data['yspan']
        self.cl_values = self.aerodynamics.data['cl']
        self.induced_cd_values = self.aerodynamics.data['induced_cd']
        self.chord_values = self.aerodynamics.data['chord']
        self.cm_values = self.aerodynamics.data['cm']

        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.b_array = np.arange(0, self.b/2, 0.01)

        self.b_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.b_h_array = np.arange(0, self.b_h/2, 0.01)

        self.b_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        self.b_v_array = np.arange(0, self.b_v, 0.01)

        self.Sh = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']

        self.aileron_start = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b1']
        self.aileron_end = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b2']
        self.aileroned_area = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['Swa']

        self.elevator_start = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1']
        self.elevator_end = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2']
        self.elevatored_area = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['Se']
        self.elevator_array = np.arange(self.elevator_start, self.elevator_end+0.01, 0.01)

        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.aileron_array = np.arange(self.aileron_start, self.aileron_end+0.01, 0.01)

        self.l_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']

        self.rho = self.aircraft_data.data['rho_air']
        self.V = self.aircraft_data.data['requirements']['cruise_speed']

        self.airfoil_Cd0 = self.aircraft_data.data['inputs']['airfoils']['cd0_wing']
        self.aileron_lift = self.aircraft_data.data['outputs']['control_surfaces']['aileron']['aileron_lift']
        self.elevator_lift = self.aircraft_data.data['outputs']['control_surfaces']['elevator']['elevator_lift']
        self.rudder_lift = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['rudder_lift']

        self.rudder_start = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b1']
        self.rudder_end = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b2']
        self.rudder_array = np.arange(self.rudder_start, self.rudder_end+0.01, 0.01)
        
        self.aspect_ratio = self.aircraft_data.data['outputs']['wing_design']['aspect_ratio']
        self.oswald_factor = self.aircraft_data.data['inputs']['oswald_factor']

        self.l_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']
        self.l_fuselage = self.aircraft_data.data['outputs']['general']['l_fuselage']

        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.xlemac = self.aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.xcg_aft = -1*self.MAC + self.xlemac #TODO link to new lemac and cg

        self.lift_arm = self.xcg_aft - (self.xlemac + 0.25*self.MAC) #TODO link to new lemac

        self.get_aileron_lift_distribution()
        self.get_elevator_lift_distribution()
        self.get_vertical_tail_lift_distribution()

        self.k = self.aircraft_data.data['outputs']['design']['k']

    def chord_span_function_aero(self,y):
        return self.chord_root + (self.chord_tip - self.chord_root) / (self.b/2) * y
    
    def get_aileron_lift_distribution(self):
        self.aileron_lift_array = np.zeros(len(self.b_array))
        self.aileron_lift_dist = self.aileron_lift/(self.aileroned_area)*self.chord_span_function_aero(self.aileron_array)
        for id, i in enumerate(self.aileron_lift_dist):
            idx = np.argmin(np.abs(self.b_array - self.aileron_array[id]))
            self.aileron_lift_array[idx] = i

    def get_elevator_lift_distribution(self):
        self.elevator_lift_array = np.zeros(len(self.b_h_array))
        self.elevator_lift_dist = self.elevator_lift/(self.elevatored_area)*self.chord_span_function(self.elevator_array)
        for id, i in enumerate(self.elevator_lift_dist):
            idx = np.argmin(np.abs(self.b_h_array - self.elevator_array[id]))
            self.elevator_lift_array[idx] = i

    def rudder_lift_distribution(self,y_vals, span, rudder_lift):

        L0 = (4 * rudder_lift) / (np.pi * span)
        return L0 * np.sqrt(1 - (y_vals / span)**2)


    def plot_Lift_distribution(self):
        plt.figure()
        plt.plot(self.yspan_values, self.lift_values)
        plt.plot(self.b_array, self.fitted_l, label="Fitted L")
        plt.xlabel("y-span")
        plt.ylabel("L [N]")
        plt.title("L Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_Drag_distribution(self):
        plt.figure()
        plt.plot(self.yspan_values, self.drag_values)
        plt.plot(self.b_array, self.fitted_d, label="Fitted Cd")
        plt.xlabel("y-span")
        plt.ylabel("D [N]")
        plt.title("D Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_Moment_distribution(self):
        plt.figure()
        plt.plot(self.yspan_values, self.moment_values)
        plt.plot(self.b_array, self.fitted_M)
        plt.xlabel("y-span")
        plt.ylabel("M [Nm]")
        plt.title("Moment Distribution over Span")
        plt.xlim(left=0)
        plt.grid(True)
        plt.show()

    def lift_distribution(self):

        self.lift_values = []
        for cl, c in zip(self.cl_values, self.chord_values):
            if cl < 0 or c < 0:
                raise ValueError("Cl values must be non-negative.")
            else:
                l = cl * (0.5*self.rho*self.V**2*c)
                self.lift_values.append(l)

    def drag_distribution(self):
        self.drag_values = []
        for cdi, c in zip(self.induced_cd_values, self.chord_values):
            idx = self.induced_cd_values.index(cdi)
            if cdi < 0 or c < 0:
                raise ValueError("Cl values must be non-negative.")
            else:
                cd = self.airfoil_Cd0 + cdi 
                d = cd * (0.5*self.rho*self.V**2*c)
                self.drag_values.append(d)

    def moment_distribution(self):
        self.moment_values = []
        for cm, c in zip(self.cm_values, self.chord_values):
            m = cm * (0.5*self.rho*self.V**2*c**2)
            self.moment_values.append(m)
    
    def get_lift_function(self):
        self.lift_distribution()
        self.L_y = Polynomial.fit(self.yspan_values, self.lift_values, 9)
        self.fitted_l = self.L_y(self.b_array)
        if self.evaluate == EvaluateType.WING:
            self.fitted_l += self.aileron_lift_array
        return self.L_y
    
    def get_drag_function(self):
        self.drag_distribution()
        self.D_y = Polynomial.fit(self.yspan_values, self.drag_values, 10)
        self.fitted_d = self.D_y(self.b_array)
        return self.D_y
    
    def get_moment_function(self):
        self.moment_distribution()
        self.M_y = Polynomial.fit(self.yspan_values, self.moment_values, 10)
        self.fitted_M = self.M_y(self.b_array)
        return self.M_y
    
    def get_vertical_tail_lift_distribution(self):
        self.vertical_tail_lift = self.rudder_lift_distribution(self.b_v_array, self.b_v, self.rudder_lift)
        return self.vertical_tail_lift
    
    def get_horizontal_tail_lift_distribution(self):
        self.get_lift_function()
        self.get_moment_function()


        self.horizontal_tail_lift_array = (self.fitted_l * self.lift_arm + self.fitted_M)/self.l_h

        #self.horizontal_tail_lift_array = self.horizontal_tail_lift/(self.Sh)*self.chord_span_h_function(self.b_h_array)
        self.horizontal_tail_lift_array[:len(self.b_h_array)] += self.elevator_lift_array

        return self.horizontal_tail_lift_array
        
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
        plt.plot(self.b_h_array, self.horizontal_tail_lift_array[:len(self.b_h_array)], label="Horizontal Tail Lift Distribution")
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
    aeroforces = AerodynamicForces(aircraft_data, aerodynamics_data,evaluate= evaluate)

    L_y = aeroforces.get_lift_function()
    D_y = aeroforces.get_drag_function()
    M_y = aeroforces.get_moment_function()
    aeroforces.get_horizontal_tail_lift_distribution()

    aeroforces.plot_Lift_distribution()
    aeroforces.plot_Drag_distribution()
    aeroforces.plot_Moment_distribution()
    aeroforces.plot_horizontal_tail_lift_distribution()
    aeroforces.plot_vertical_tail_lift_distribution()
