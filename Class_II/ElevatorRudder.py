import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import EmpType, Data
from Class_I.empennage import Empennage
import numpy as np
from scipy.integrate import quad
from utils import ISA
import matplotlib.pyplot as plt

class ElevatorRudder:

    def __init__(self, aircraft_data: Data, plot=False):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.engine_positions = [7, 12, 17] #TODO link to json

        self.plot = plot

        self.airfoil_cl_alpha = 0.12 # TODO link to json
        self.tail_lift_slope = 0.09 # TODO link to json


        self.rudder_chord_ratio = self.aircraft_data.data['inputs']['control_surfaces']['rudder_chord']
        self.elevator_chord_ratio = self.aircraft_data.data['inputs']['control_surfaces']['elevator_chord']

        self.rudder_deflection = self.aircraft_data.data['inputs']['control_surfaces']['rudder_deflection']
        self.elevator_deflection = self.aircraft_data.data['inputs']['control_surfaces']['elevator_deflection']

        self.engine_power = self.aircraft_data.data['inputs']['engine_power']
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

        self.vertical_tail_thickness = self.aircraft_data.data['inputs']['airfoils']['vertical_tail']*self.chord_root_v/2
        print(f"Vertical tail thickness: {self.vertical_tail_thickness}")
        self.elevator_start = self.vertical_tail_thickness + self.elevator_start

        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']

        self.nmax = self.aircraft_data.data['outputs']['general']['nmax']
        self.climb_rate = self.aircraft_data.data['requirements']['climb_rate']

        self.engine_thrust = 0.75*self.engine_power * self.prop_efficiency / self.V

        self.prop_diameter = 5.2 # TODO link to json
        high_altitude = self.aircraft_data.data['requirements']['high_altitude']
        self.rho = self.aircraft_data.data['rho_air'] #TODO adapt to mission profiles
        self.isa = ISA(altitude=high_altitude)
        self.rho_high = self.isa.rho

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
        
        self.CN_OEI = (right_yaw_moment - left_yaw_moment) / (self.S*self.b*0.5*self.rho_high*self.V**2)
        return self.CN_OEI
    
    def calculate_required_rudder_surface(self):

        CNe = self.calculate_engine_OEI_yaw()
        CNe_dr = -CNe/np.deg2rad(self.rudder_deflection)
        self.b_test = np.arange(self.rudder_start, self.b_v+0.001, 0.001)
        tolerance = 0.0001
        for b in self.b_test:
            integral_test, _ = quad(self.chord_v,self.rudder_start, b)
            cndr_test = integral_test * -(self.airfoil_cl_alpha * self.control_surface_effectiveness(self.rudder_chord_ratio)*self.l_v) / (self.S * self.b)
            #print(cndr_test, CNe_dr)
            if abs(CNe_dr - cndr_test) < tolerance:
                self.rudder_end = b
                break
        if not hasattr(self, 'rudder_end'):
            raise ValueError("Aint gonna work cuh")
        
        self.rudder_area = self.rudder_chord_ratio * self.chord_v((self.rudder_end-self.rudder_start)/2) * (self.rudder_end - self.rudder_start)

    def calculate_pitch_rate(self):
        pitch_rate = (self.nmax-1)*9.81/(self.V*np.sqrt(1-(self.climb_rate/self.V)**2))*0.8
        return pitch_rate
    
    def calculate_Cmde_Cmq(self,b):
        self.pitch_rate = self.calculate_pitch_rate()
        integral, _ = quad(self.chord_h, self.elevator_start, b)
        integral*=2
        elevator_effectiveness = self.control_surface_effectiveness(self.elevator_chord_ratio)
        ratio = (self.airfoil_cl_alpha * elevator_effectiveness)/(self.Sh*self.l_h*self.tail_lift_slope)*integral
        return ratio
    
    def calculate_elevator_surface(self):
        self.pitch_rate = self.calculate_pitch_rate()
        self.required_Cmde_Cmq = -self.pitch_rate/np.deg2rad(-self.elevator_deflection)*(self.MAC/self.V)
        self.calculate_elevator_position()

    def calculate_elevator_position(self):
        self.b_test = np.arange(0, self.b_h/2+0.001, 0.001)
        tolerance = 0.001
        for b in self.b_test:
            ratio = self.calculate_Cmde_Cmq(b)
            #print(ratio, self.required_Cmde_Cmq)
            if abs(ratio - self.required_Cmde_Cmq) < tolerance:
                self.elevator_end = b
                break

        self.elevator_area = self.elevator_chord_ratio * self.chord_h((self.elevator_end-self.elevator_start)/2) * (self.elevator_end - self.elevator_start)
        
        if not hasattr(self, 'elevator_end'):
            raise ValueError("Aint gonna work cuh")
        
    def main(self):
        self.calculate_required_rudder_surface()
        if self.plot:
            self.plot_vertical_tail()
        self.calculate_elevator_surface()

        if self.plot:
            self.plot_horizontal_tail()
        
        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)

    def update_attributes(self):

        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['end'] = self.rudder_end
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['start'] = self.rudder_start
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['chord_ratio'] = self.rudder_chord_ratio
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['deflection'] = self.rudder_deflection
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['area'] = self.rudder_area
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['end'] = self.elevator_end
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['area'] = self.elevator_area
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['start'] = self.elevator_start
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['chord_ratio'] = self.elevator_chord_ratio
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['deflection'] = self.elevator_deflection
        self.aircraft_data.data['outputs']['control_surfaces']['elevator']['pitch_rate'] = np.rad2deg(self.pitch_rate)
        self.aircraft_data.data['outputs']['control_surfaces']['rudder']['CN_OEI'] = self.CN_OEI


    def plot_horizontal_tail(self):
        b_half = np.arange(0, self.b_h/2 + 0.001, 0.001)

        # Main tail
        y_root_LE = self.chord_root_h / 2
        leading_edge_h = y_root_LE - np.tan(np.deg2rad(self.sweep_h)) * b_half
        trailing_edge_h = leading_edge_h - self.chord_h(b_half)

        # Elevator
        b_elevator = np.array([self.elevator_start, self.elevator_end])
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

        # Plotting
        plt.figure(figsize=(10, 5))

        # Horizontal tail (right and left)
        plt.plot(b_half, leading_edge_h, color='green', label='Horizontal Tail LE')
        plt.plot(b_half, trailing_edge_h, color='green', label='Horizontal Tail TE')
        plt.plot(b_half_mirror, leading_edge_h_mirror, color='green')
        plt.plot(b_half_mirror, trailing_edge_h_mirror, color='green')
        plt.plot([0, 0], [y_root_LE, trailing_edge_h[0]], color='green')  # Root vertical line

        # Tip vertical lines
        plt.plot([self.b_h/2, self.b_h/2], [leading_edge_h[-1], trailing_edge_h[-1]], color='green')
        plt.plot([-self.b_h/2, -self.b_h/2], [leading_edge_h[-1], trailing_edge_h[-1]], color='green')

        # Elevator (right and left)
        # Right
        plt.plot(b_elevator, le_elevator_actual, color='red', label='Elevator LE')
        plt.plot(b_elevator, te_elevator, color='red', label='Elevator TE')
        plt.plot([b_elevator[0], b_elevator[0]], [le_elevator_actual[0], te_elevator[0]], color='red')
        plt.plot([b_elevator[1], b_elevator[1]], [le_elevator_actual[1], te_elevator[1]], color='red')
        # Left (mirror)
        plt.plot(b_elevator_mirror, le_elevator_actual_mirror, color='red')
        plt.plot(b_elevator_mirror, te_elevator_mirror, color='red')
        plt.plot([b_elevator_mirror[0], b_elevator_mirror[0]], [le_elevator_actual_mirror[0], te_elevator_mirror[0]], color='red')
        plt.plot([b_elevator_mirror[1], b_elevator_mirror[1]], [le_elevator_actual_mirror[1], te_elevator_mirror[1]], color='red')

        # Final touches
        plt.title('Horizontal Tail and Elevator Planform')
        plt.xlabel('Spanwise Position (m)')
        plt.ylabel('Chordwise Position (m)')
        plt.ylim(-5, 5)
        plt.xlim(-self.b_h/2 - 1, self.b_h/2 + 1)
        plt.gca().set_aspect('equal')
        plt.grid(True)
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
    