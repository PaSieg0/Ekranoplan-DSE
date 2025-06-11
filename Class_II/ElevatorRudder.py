import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import EmpType, Data
from Class_I.empennage import Empennage
import numpy as np
from scipy.integrate import quad
from utils import ISA
import matplotlib.pyplot as plt
from aero.lift_curve import lift_curve
from AerodynamicForces import AerodynamicForces

class ElevatorRudder:

    def __init__(self, aircraft_data: Data, plot=False):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.engine_positions = self.aircraft_data.data['outputs']['engine_positions']['y_engines'] #TODO wait for new values

        self.plot = plot

        self.lift_curve = lift_curve()
        self.aeroforces = AerodynamicForces(self.aircraft_data)
        self.aeroforces.get_max_aero_dist()
        self.airfoil_cl_alpha = self.lift_curve.dcl_dalpha()
        self.tail_lift_slope = self.lift_curve.dcl_dalpha()

        self.rudder_chord_ratio = self.aircraft_data.data['inputs']['control_surfaces']['rudder_chord']
        self.elevator_chord_ratio = self.aircraft_data.data['inputs']['control_surfaces']['elevator_chord']

        self.rudder_deflection = self.aircraft_data.data['inputs']['control_surfaces']['rudder_deflection']
        self.elevator_deflection = self.aircraft_data.data['inputs']['control_surfaces']['elevator_deflection']

        self.engine_power = self.aircraft_data.data['inputs']['engine']['engine_power']
        self.prop_efficiency = self.aircraft_data.data['inputs']['prop_efficiency']
        self.V = self.aircraft_data.data['requirements']['cruise_speed']
        self.S = self.aircraft_data.data['outputs']['wing_design']['S']
        self.b = self.aircraft_data.data['outputs']['wing_design']['b']
        self.Sh = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']

        self.l_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v']
        self.l_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']
        self.chord_root_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
        self.chord_tip_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
        self.chord_root_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
        self.chord_tip_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip']
        self.b_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        self.b_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.elevator_start = self.aircraft_data.data['inputs']['control_surfaces']['elevator_start']
        self.rudder_start = self.aircraft_data.data['inputs']['control_surfaces']['rudder_start']*self.b_v
        self.sweep_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep']
        self.taper_v = self.chord_tip_v / self.chord_root_v
        self.sweep_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['sweep']
        self.w_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']

        self.vertical_tail_thickness = self.aircraft_data.data['inputs']['airfoils']['vertical_tail']*self.chord_root_v
        self.take_off_power = self.aircraft_data.data['outputs']['general']['take_off_power']
        self.V_lof = self.aircraft_data.data['requirements']['stall_speed_takeoff']*1.05
        self.i_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['i_h']

        self.take_off_drag = self.take_off_power / self.V_lof
        self.highest_cg = self.aircraft_data.data['outputs']['cg_range']['highest_cg']

        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']

        self.nmax = self.aircraft_data.data['outputs']['general']['nmax']
        self.climb_rate = self.aircraft_data.data['requirements']['climb_rate']
        self.n_engines = self.aircraft_data.data['inputs']['engine']['n_engines']

        self.engine_zs = np.array(self.aircraft_data.data['outputs']['engine_positions']['z_engines'])
        self.vertical_engine_arms = self.engine_zs - self.highest_cg #TODO update when prop position is known
        self.prop_diameter = self.aircraft_data.data['inputs']['engine']['prop_diameter'] 
        high_altitude = self.aircraft_data.data['requirements']['high_altitude']
        self.rho = self.aircraft_data.data['rho_air'] 
        self.isa = ISA(altitude=high_altitude)
        self.rho_high = self.isa.rho
        self.Cd0 = self.aircraft_data.data['inputs']['Cd0']
        self.engine_thrust = 0.5*self.rho*self.V**2*self.S*self.Cd0/4

        self.vertical_tail_first_x = self.w_fuselage/2 - self.vertical_tail_thickness/2
        self.vertical_tail_second_x = self.vertical_tail_first_x + self.vertical_tail_thickness

        self.main_lift_moment = np.trapz(self.aeroforces.L_y, self.aeroforces.b_array)*2*self.aeroforces.lift_arm*0.6
        self.main_moment = np.trapz(self.aeroforces.M_y, self.aeroforces.b_array)*2
        self.engine_moments = sum(self.engine_thrust * np.array(self.vertical_engine_arms))*2

    def control_surface_effectiveness(self,r):
        return -6.624*r**4 + 12.07*r**3 - 8.292*r**2 + 3.295*r + 0.004942
    
    def chord_v(self, y):

        return (self.chord_tip_v - self.chord_root_v) * (y / self.b_v) + self.chord_root_v
    
    def chord_h(self, y):

        return (self.chord_tip_h - self.chord_root_h) * (y / self.b_h) + self.chord_root_h
    
    def calculate_engine_OEI_yaw(self):
        right_yaw_moment = 0

        for i in self.engine_positions[:2]:
            i += 1/4*self.prop_diameter
            right_yaw_moment += self.engine_thrust * i
        
        left_yaw_moment = 0
        for i in self.engine_positions[:1]:
            i -= 1/4*self.prop_diameter
            left_yaw_moment += self.engine_thrust * i
        
        self.CN_OEI = (right_yaw_moment - left_yaw_moment)/2 / (self.S*self.b*0.5*self.rho*self.V**2)*1.5
        return self.CN_OEI
    
    def calculate_required_rudder_surface(self):

        CNe = self.calculate_engine_OEI_yaw()
        CNe_dr = -CNe/np.deg2rad(self.rudder_deflection)
        self.b_test = np.arange(self.rudder_start, self.b_v+0.001, 0.001)
        tolerance = 0.000001
        for b in self.b_test:
            integral_test, _ = quad(self.chord_v,self.rudder_start, b)
            cndr_test = integral_test * -(self.airfoil_cl_alpha * self.control_surface_effectiveness(self.rudder_chord_ratio)*self.l_v) / (self.S * self.b)
            # print(cndr_test, CNe_dr)
            if 0 < abs(cndr_test - CNe_dr) <= tolerance:
                self.rudder_end = b
                self.cndr = cndr_test
                break
        if not hasattr(self, 'rudder_end'):
            raise ValueError("Aint gonna work cuh")
        
        integral, _ = quad(self.chord_v, self.rudder_start, self.rudder_end)
        self.Sr = integral
        self.rudder_area = self.rudder_chord_ratio * self.Sr
        self.rudder_normal_force = self.cndr*np.deg2rad(self.rudder_deflection)*0.5*self.rho*self.V**2*self.S*self.b/self.l_v

    def calculate_pitch_rate(self):

        pitch_rate = (self.nmax-1)*9.81/self.V*1.5
        return pitch_rate
    
    def calculate_Cmde_Cmq(self,b):
        self.pitch_rate = self.calculate_pitch_rate()
        integral, _ = quad(self.chord_h, self.elevator_start, b)
        elevator_effectiveness = self.control_surface_effectiveness(self.elevator_chord_ratio)
        ratio = 2*(self.airfoil_cl_alpha * elevator_effectiveness)/(self.Sh*self.l_h*self.tail_lift_slope)*integral
        return ratio
    
    def calculate_elevator_surface(self):
        self.pitch_rate = self.calculate_pitch_rate()
        self.required_Cmde_Cmq = -self.pitch_rate/np.deg2rad(-self.elevator_deflection)*(self.MAC/self.V)
        self.calculate_elevator_position()

    def calculate_elevator_position(self):
        self.b_test = np.arange(0, self.b_h/2+0.001, 0.001)
        tolerance = 0.001
        #TODO account for double vertical tail
        for b in self.b_test:
            ratio = self.calculate_Cmde_Cmq(b)
            # print(ratio, self.required_Cmde_Cmq)
            if abs(ratio - self.required_Cmde_Cmq) < tolerance:
                self.elevator_end = b
                if self.elevator_end + self.vertical_tail_thickness < self.b_h/2:
                    break
        
        if not hasattr(self, 'elevator_end'):
            raise ValueError("Aint gonna work cuh")
        
        integral, _ = quad(self.chord_h, self.elevator_start, self.elevator_end)
        self.elevator_area = self.elevator_chord_ratio * integral

    def calculate_elevator_normal_force(self):
        area_elevator, _ = quad(self.chord_h, self.elevator_start, self.elevator_end)
        self.Se = area_elevator

        elevator_effectiveness = self.control_surface_effectiveness(self.elevator_chord_ratio)
        self.CMde = -self.airfoil_cl_alpha * elevator_effectiveness * self.l_h/(self.S * self.MAC)*self.Se

        Cmq = -self.Sh*self.l_h**2/self.S/self.MAC * self.tail_lift_slope/2
        print(f'cmq: {Cmq}')

        N = self.CMde * np.deg2rad(self.elevator_deflection) * 0.5 * self.rho * self.V**2 * self.S * self.MAC/self.l_h
        return N
    
    def calculate_pitch_up_performance(self):
        self.horizontal_tail_lift = (self.main_lift_moment -self.engine_moments + self.main_moment - self.take_off_drag * self.highest_cg) /self.l_h * self.Sh/self.S/2
        zero_elevator_lift = self.i_h*self.tail_lift_slope * self.Sh * 0.5 * self.rho * self.V**2
        self.required_elevator_lift = self.horizontal_tail_lift - zero_elevator_lift
        self.trim_deflection_TO = np.ceil(-self.required_elevator_lift * self.l_h*1.5/(0.5*self.rho*self.V**2*self.MAC*self.S) / self.CMde * 180/np.pi)
        print(f'trim deflection TO: {self.trim_deflection_TO} deg')

    def main(self):
        self.calculate_required_rudder_surface()
        if self.plot:
            self.plot_vertical_tail()
        self.calculate_elevator_surface()

        if self.plot:
            self.plot_horizontal_tail()
        
        self.elevator_lift = self.calculate_elevator_normal_force()
        print(f"Elevator lift: {self.elevator_lift}")
        print(f"Rudder lift: {self.rudder_normal_force}")
        self.calculate_pitch_up_performance()
        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)

    def update_attributes(self):

        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b2'] = self.rudder_end
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['b1'] = self.rudder_start
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['chord_ratio'] = self.rudder_chord_ratio
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['deflection'] = self.rudder_deflection
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['area'] = self.rudder_area
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['Sr'] = self.Sr
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2'] = self.elevator_end
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['Se'] = self.Se
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['area'] = self.elevator_area
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1'] = self.elevator_start
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['chord_ratio'] = self.elevator_chord_ratio
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['deflection'] = self.elevator_deflection
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['pitch_rate'] = np.rad2deg(self.pitch_rate)
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1_s1'] = self.elevator_start
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2_s1'] = self.vertical_tail_first_x
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b1_s2'] = self.vertical_tail_second_x
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['b2_s2'] = self.elevator_end + self.vertical_tail_thickness
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['CN_OEI'] = self.CN_OEI
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['cndr'] = self.cndr
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['TO_deflection'] = self.trim_deflection_TO
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['CMde'] = 2*self.CMde
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['elevator_lift'] = self.elevator_lift
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_z_delta_e'] = 2*self.elevator_lift / (np.deg2rad(self.elevator_deflection) * 0.5 * self.rho * self.V**2 * self.S * self.MAC)
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_sym']['C_m_delta_e'] = 2*self.CMde
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['rudder_lift'] = self.rudder_normal_force
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['C_y_delta_r'] = self.rudder_normal_force / (-np.deg2rad(self.rudder_deflection) * 0.5 * self.rho * self.V**2 * self.S * self.b)
        self.rudder_height = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'] / 2 + self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage']
        # TODO: UPDATE THIS AFTER CG HEIGHT IS DETERMINED
        self.high_cg = self.aircraft_data.data['outputs']['cg_range']['highest_cg']
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['C_l_delta_r'] = (self.rudder_height-self.high_cg) * self.rudder_normal_force / (np.deg2rad(self.rudder_deflection) * 0.5 * self.rho * self.V**2 * self.S * self.b)
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['C_n_delta_r'] = self.cndr


    def plot_horizontal_tail(self):
        b_half = np.arange(0, self.b_h/2 + 0.001, 0.001)

        # Main tail
        y_root_LE = self.chord_root_h / 2
        leading_edge_h = y_root_LE - np.tan(np.deg2rad(self.sweep_h)) * b_half
        trailing_edge_h = leading_edge_h - self.chord_h(b_half)

        # Elevator
        b_elevator = np.arange(self.elevator_start, self.elevator_end + 0.01, 0.01)
        le_elevator = y_root_LE - np.tan(np.deg2rad(self.sweep_h)) * b_elevator
        te_elevator = le_elevator - self.chord_h(b_elevator)

        le_elevator_actual = te_elevator + self.chord_h(b_elevator) * self.elevator_chord_ratio

        # Mirror for left side
        b_half_mirror = -b_half
        leading_edge_h_mirror = leading_edge_h
        trailing_edge_h_mirror = trailing_edge_h

        b_elevator_mirror = -b_elevator
        le_elevator_mirror = le_elevator
        te_elevator_mirror = te_elevator
        le_elevator_actual_mirror = le_elevator_actual

        vertical_tail_first_x = self.vertical_tail_first_x
        first_idx = np.where(b_elevator >= vertical_tail_first_x)[0][0]
        
        vertical_tail_first_LE = le_elevator_actual[first_idx]
        vertical_tail_first_TE = te_elevator[first_idx]

        vertical_tail_second_x = self.vertical_tail_second_x
        second_idx = np.where(b_elevator >= vertical_tail_second_x)[0][0]
        vertical_tail_second_LE = le_elevator_actual[second_idx]
        vertical_tail_second_TE = te_elevator[second_idx]

        plt.figure(figsize=(10, 5))
        # Horizontal tail (right and left)
        plt.plot(b_half, leading_edge_h, color='green')
        plt.plot(b_half, trailing_edge_h, color='green')
        plt.plot(b_half_mirror, leading_edge_h_mirror, color='green')
        plt.plot(b_half_mirror, trailing_edge_h_mirror, color='green')
        plt.plot([0, 0], [y_root_LE, trailing_edge_h[0]+(le_elevator_actual[0]-te_elevator[0])], color='green')  # Root vertical line

        # Tip vertical lines
        plt.plot([self.b_h/2, self.b_h/2], [leading_edge_h[-1], trailing_edge_h[-1]], color='green')
        plt.plot([-self.b_h/2, -self.b_h/2], [leading_edge_h[-1], trailing_edge_h[-1]], color='green')

        # Elevator (right and left)
        # Right side
        elevator_le1, = plt.plot(b_elevator[:first_idx], le_elevator_actual[:first_idx], color='red', label='Elevator')
        plt.plot(b_elevator[:first_idx], te_elevator[:first_idx], color='red')
        plt.plot(b_elevator[second_idx:], le_elevator_actual[second_idx:], color='red')
        plt.plot(b_elevator[second_idx:], te_elevator[second_idx:], color='red')

        # Vertical tail (blue dotted lines)
        vt1, = plt.plot([b_elevator[first_idx+10], b_elevator[first_idx+10]], [leading_edge_h[first_idx], trailing_edge_h[first_idx]], linestyle = '--', color='blue', label='Vertical Tail')
        plt.plot([b_elevator[second_idx-10], b_elevator[second_idx-10]], [leading_edge_h[second_idx], trailing_edge_h[second_idx]], linestyle = '--', color='blue')
        plt.plot([-b_elevator[first_idx+10], -b_elevator[first_idx+10]], [leading_edge_h[first_idx], trailing_edge_h[first_idx]], linestyle = '--', color='blue')
        plt.plot([-b_elevator[second_idx-10], -b_elevator[second_idx-10]], [leading_edge_h[second_idx], trailing_edge_h[second_idx]], linestyle = '--', color='blue')

        plt.plot([vertical_tail_first_x, vertical_tail_first_x], [vertical_tail_first_LE, vertical_tail_first_TE], color='red')
        plt.plot([vertical_tail_second_x, vertical_tail_second_x], [vertical_tail_second_LE, vertical_tail_second_TE], color='red')
        plt.plot([b_elevator[-1], b_elevator[-1]], [le_elevator_actual[-1], te_elevator[-1]], color='red')

        # Mirror elevator for left side
        plt.plot(b_elevator_mirror[:first_idx], le_elevator_actual_mirror[:first_idx], color='red')
        plt.plot(b_elevator_mirror[:first_idx], te_elevator_mirror[:first_idx], color='red')
        plt.plot(b_elevator_mirror[second_idx:], le_elevator_actual_mirror[second_idx:], color='red')
        plt.plot(b_elevator_mirror[second_idx:], te_elevator_mirror[second_idx:], color='red')

        plt.plot([ -vertical_tail_first_x, -vertical_tail_first_x], [vertical_tail_first_LE, vertical_tail_first_TE], color='red')
        plt.plot([ -vertical_tail_second_x, -vertical_tail_second_x], [vertical_tail_second_LE, vertical_tail_second_TE], color='red')
        plt.plot([b_elevator_mirror[-1], b_elevator_mirror[-1]], [le_elevator_actual_mirror[-1], te_elevator_mirror[-1]], color='red')

        # Final touches
        plt.title('Horizontal Tail and Elevator Planform')
        plt.xlabel('Spanwise Position (m)')
        plt.ylabel('Chordwise Position (m)')
        plt.ylim(-5, 5)
        plt.xlim(-self.b_h/2 - 1, self.b_h/2 + 1)
        plt.gca().set_aspect('equal')
        plt.grid(True)
        # Only show elevator and vertical tail in legend
        handles = [elevator_le1, vt1]
        labels = [h.get_label() for h in handles]
        plt.legend(handles, labels)
        plt.show()



    def plot_vertical_tail(self):
        b = np.arange(0, self.b_v + 0.001, 0.001)
        x_root_LE = 0
        leading_edge_v = x_root_LE + np.tan(np.deg2rad(self.sweep_v)) * b
        x_root_TE = x_root_LE + self.chord_root_v
        x_tip_LE = leading_edge_v[-1]
        x_tip_TE = x_tip_LE + self.chord_tip_v

        y_rudder_root = self.rudder_start
        y_rudder_tip = self.rudder_end
        c_rudder_root = self.chord_v(y_rudder_root)
        c_rudder_tip = self.chord_v(y_rudder_tip)

        trailing_edge_v = leading_edge_v + self.chord_v(b)

        x_tail_TE_root = x_root_LE + np.tan(np.deg2rad(self.sweep_v)) * y_rudder_root + c_rudder_root
        x_tail_TE_tip = x_root_LE + np.tan(np.deg2rad(self.sweep_v)) * y_rudder_tip + c_rudder_tip

        x_rudder_LE_root = x_tail_TE_root - self.rudder_chord_ratio * c_rudder_root
        x_rudder_LE_tip = x_tail_TE_tip - self.rudder_chord_ratio * c_rudder_tip

        x_rudder_TE_root = x_tail_TE_root
        x_rudder_TE_tip = x_tail_TE_tip

        plt.figure(figsize=(6, 10))
        plt.plot(trailing_edge_v, b, color='blue')
        plt.plot(leading_edge_v, b, color='green')
        plt.plot([x_root_LE, x_root_TE], [0, 0], color='green')
        plt.plot([x_tip_LE, x_tip_TE], [self.b_v, self.b_v], color='green')
        plt.plot([x_root_TE, x_tip_TE], [0, self.b_v], color='green')

        plt.plot([x_rudder_LE_root, x_rudder_TE_root], [y_rudder_root, y_rudder_root], color='red')
        plt.plot([x_rudder_LE_tip, x_rudder_TE_tip], [y_rudder_tip, y_rudder_tip], color='red')
        plt.plot([x_rudder_LE_root, x_rudder_LE_tip], [y_rudder_root, y_rudder_tip], color='red')
        plt.plot([x_rudder_TE_root, x_rudder_TE_tip], [y_rudder_root, y_rudder_tip], color='red')
        plt.gca().set_aspect('equal')
        plt.title('Vertical Tail Chord Distribution with Rudder')
        plt.xlabel('Chordwise Position (m)')
        plt.ylabel('Spanwise Position (m)')
        plt.grid()
        plt.show()




if __name__ == "__main__":
    data = Data('design3.json')
    elevator_rudder = ElevatorRudder(data,plot=True)
    # elevator_rudder.calculate_required_rudder_surface()

    # elevator_rudder.plot_vertical_tail()
    elevator_rudder.main()
    print(f"OEI Yaw CN: {elevator_rudder.CN_OEI}")
    print(f"Rudder end: {elevator_rudder.rudder_end}")
    print(f"Rudder area: {elevator_rudder.rudder_area}")
    print(f"pitch rate: {np.rad2deg(elevator_rudder.pitch_rate)}")
    print(f"Elevator end: {elevator_rudder.elevator_end}")
    print(f"Elevator area: {elevator_rudder.elevator_area}")
