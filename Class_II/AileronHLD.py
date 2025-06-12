import matplotlib.pyplot as plt
import numpy as np
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, FlapType
from scipy.integrate import quad
from aero.lift_curve import lift_curve



class AileronHLD:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"

        self.flaptype = FlapType[self.aircraft_data.data['inputs']['control_surfaces']['flap_type']]
        print(self.flaptype)
        self.S = self.aircraft_data.data['outputs']['wing_design']['S']
        self.b = self.aircraft_data.data['outputs']['wing_design']['b']

        self.taper_ratio = self.aircraft_data.data['outputs']['wing_design']['taper_ratio']
        self.sweep = self.aircraft_data.data['outputs']['wing_design']['sweep_x_c']
        self.root_chord = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.tip_chord = self.aircraft_data.data['outputs']['wing_design']['chord_tip']

        self.w_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']

        self.cruise_altitude = self.aircraft_data.data['inputs']['cruise_altitude']

        self.max_aileron_deflection = np.deg2rad(self.aircraft_data.data['inputs']['control_surfaces']['aileron_deflection'])

        self.turn_radius = self.aircraft_data.data['outputs']['general']['min_turn_radius']
        self.bank_angle = np.deg2rad(self.aircraft_data.data['outputs']['general']['max_bank_angle'])
        self.V = np.sqrt(self.turn_radius*9.81*np.tan(self.bank_angle))
        self.object_distance = self.turn_radius*1.4

        self.aileron_end = self.aircraft_data.data['inputs']['control_surfaces']['aileron_end']*self.b/2

        self.flap_start = 1 + self.w_fuselage/2

        self.rel_flap_chord = self.aircraft_data.data['inputs']['control_surfaces']['flap_chord']

        self.rel_LE_flap_chord = self.aircraft_data.data['inputs']['control_surfaces']['LE_flap_chord']
        self.rho = self.aircraft_data.data['rho_air']

        self.avg_chord = (self.root_chord + self.tip_chord)/2

        self.aileron_chord_ratio = self.aircraft_data.data['inputs']['control_surfaces']['aileron_chord']

        self.dihedral = self.aircraft_data.data['outputs']['wing_design']['dihedral']

        self.airfoil_Cd0 = self.aircraft_data.data['inputs']['airfoils']['cd0_wing']
        self.lift_curve = lift_curve()
        self.airfoil_Cl_alpha = self.lift_curve.dcl_dalpha()*180/np.pi

        self.CLMax_landing = self.aircraft_data.data['inputs']['CLmax_landing']
        self.CLMax_clean = self.aircraft_data.data['inputs']['CLmax_clean']
        self.required_CLmax_increase = (self.CLMax_landing - self.CLMax_clean)*1.5
        self.Clp = self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['Clp']

        self.LE_flap = False
        self.tau = self.aileron_effectiveness()

    def leading_edge(self,y):
        return self.root_chord/2 - np.deg2rad(self.sweep)*y
    
    def chord_span_function(self,y):

        return self.root_chord + (self.tip_chord - self.root_chord)/(self.b/2) * y
    
    def Swa(self, y):
        return self.chord_span_function(y)
    def c(self, y):
        return self.chord_span_function(y)*y
    
    def c_Clp(self, y):
        return self.chord_span_function(y)*y**2

    def aileron_effectiveness(self):
        r = self.aileron_chord_ratio
        return -6.624*r**4 + 12.07*r**3 - 8.292*r**2 + 3.295*r + 0.004942

    def calculate_roll_rate(self):
        self.roll_rate = self.bank_angle/((self.object_distance-self.turn_radius)/self.V)*1.5
        return self.roll_rate
    
    def calculate_Clp_integral(self):
        integral,_ = quad(self.c_Clp, 0, self.b/2)
        return integral
    
    # def Clda_Clp_ratio(self,b):
    #     integral,_ = quad(self.c, b, self.aileron_end)
    #     return -1/2*self.airfoil_Cl_alpha*self.tau*integral*self.b/(self.airfoil_Cl_alpha+self.airfoil_Cd0)/self.calculate_Clp_integral()
    
    def Clda_Clp_ratio(self,b):
        integral,_ = quad(self.c, b, self.aileron_end)
        ratio = 2*self.airfoil_Cl_alpha*self.tau/(self.S*self.b)*integral/self.Clp
        return ratio

    def calculate_aileron_position(self):
        self.b_test = np.arange(0, self.b/2+0.001, 0.001)
        tolerance = 0.0001
        for b in self.b_test:
            ratio = self.Clda_Clp_ratio(b)
            # print(ratio, self.required_Cla_Clp)
            if abs(ratio - self.required_Cla_Clp) < tolerance:
                self.aileron_start = b
                break
        
        aileron_integral = quad(self.c, self.aileron_start, self.aileron_end)[0]
        self.Clda = -2*np.rad2deg(self.airfoil_Cl_alpha)*self.tau/self.S/self.b*aileron_integral

        L = self.Clda*self.max_aileron_deflection*self.S*self.b
        mid_point = (self.aileron_start + self.aileron_end)/2

        self.aileron_lift = L/mid_point/2
        self.aileroned_area = quad(self.Swa, self.aileron_start, self.aileron_end)[0]
        print(f"Aileron lift: {self.aileron_lift}")

    def calculate_aileron_size(self):
        self.required_roll_rate = self.calculate_roll_rate()
        print(self.required_roll_rate)
        self.required_Cla_Clp = -self.required_roll_rate/(self.max_aileron_deflection*(2*self.V/self.b))
        self.calculate_aileron_position()
        self.aileron_area = self.chord_span_function((self.aileron_end - self.aileron_start)/2)*self.aileron_chord_ratio*(self.aileron_end - self.aileron_start)

    def get_clmax_increase(self, LE=False):
        #@self.flap_deflection = 40
        
        self.flap_chord = self.rel_flap_chord*self.avg_chord
        self.flap_LE_chord = self.rel_LE_flap_chord*self.avg_chord

        if LE:
            self.clmax_increase = (1+0.5*self.flap_LE_chord)/self.flap_LE_chord*0.4
            return

        if self.flaptype == FlapType.FOWLER:
            c_c = (1+0.63*self.flap_chord)/self.flap_chord
            self.clmax_increase = 1.3*c_c

        elif self.flaptype == FlapType.PLAIN_SPLIT:
            self.clmax_increase = 0.9

        elif self.flaptype == FlapType.SLOT:
            self.clmax_increase = 1.3

        elif self.flaptype == FlapType.DOUBLE_SLOT:
            c_c = (1+0.63*self.flap_chord)/self.flap_chord
            self.clmax_increase = 1.6*c_c

        elif self.flaptype == FlapType.TRIPLE_SLOT:
            c_c = (1+0.78*self.flap_chord)/self.flap_chord
            self.clmax_increase = 1.9*c_c

    def calculate_flapped_area(self,LE=False):
        if LE:
            Clmax_increase = self.required_CLmax_increase_LE
        else:
            Clmax_increase = self.required_CLmax_increase
        return Clmax_increase/0.9/self.clmax_increase*self.S

    def calculate_flap_endpoint(self,LE=False):
        a = self.root_chord
        m = (self.tip_chord - self.root_chord) / (self.b / 2)

        if LE:
            flap_area = self.LE_flap_area
        else:
            flap_area = self.flap_area

        K = flap_area + a * self.flap_start + 0.5 * m * self.flap_start**2

        # Solve quadratic: (m/2)*b2^2 + a*b2 - K = 0
        discriminant = a**2 + 2 * m * K

        if discriminant < 0:
            raise ValueError("No real solution for b2.")

        b2 = (-a + np.sqrt(discriminant)) / m

        return b2
    
    def calculate_flapped_by_span(self):
        m = (self.tip_chord - self.root_chord) / (self.b / 2)
        return self.root_chord * (self.flap_end-self.flap_start) + 0.5 * m * (self.flap_end**2 - self.flap_start**2)
    
    def calculate_CLmax_increase(self,area,clmax_increase):
        return 0.9*clmax_increase*area/self.S
    
    def add_LE_flap(self):
        self.flap_end = self.aileron_start - 1

        self.tot_TE_flap_area = self.calculate_flapped_by_span()*2
        self.CL_increase = self.calculate_CLmax_increase(self.tot_TE_flap_area,self.clmax_increase)
        self.CL_increase_TO_TE = self.calculate_CLmax_increase(self.tot_TE_flap_area,self.clmax_increase*0.6)
        self.get_clmax_increase(LE=True)
        self.required_CLmax_increase_LE = self.required_CLmax_increase - self.CL_increase
        self.LE_flap_area = self.calculate_flapped_area(LE=True)/2
        self.tot_LE_flap_area = self.LE_flap_area*2

        self.LE_flap_end = self.calculate_flap_endpoint(LE=True)
        self.CL_increase_TO = self.calculate_CLmax_increase(self.tot_LE_flap_area,self.clmax_increase*0.6)
        self.CL_max_TO = self.CLMax_clean + self.CL_increase_TO + self.CL_increase_TO_TE

        self.actual_LE_flap_area = self.chord_span_function((self.LE_flap_end - self.flap_start)/2)*self.rel_LE_flap_chord*(self.LE_flap_end - self.flap_start)
        self.actual_flap_area = self.chord_span_function((self.flap_end - self.flap_start)/2)*self.rel_flap_chord*(self.flap_end - self.flap_start)

        # print(f"LE flap area: {self.LE_flap_area}")
        # print(f"LE flap chord: {self.flap_LE_chord}")
        # print(f"LE flap end: {self.LE_flap_end}")
        # print(f"TE flap area: {self.tot_TE_flap_area}")
        # print(f"TE flap chord: {self.flap_chord}")
        # print(f"TE flap end: {self.flap_end}")
        # print(f"TO CLmax: {self.CL_max_TO}")
    
    def calculate_flap_size(self):
        self.get_clmax_increase()
        self.TO_clmax_increase = self.clmax_increase*0.6
        self.flap_area = self.calculate_flapped_area()/2
        self.tot_flap_area = self.flap_area*2
        self.flap_end = self.calculate_flap_endpoint()
        if self.flap_end > self.aileron_start-1:
            self.add_LE_flap()
            self.LE_flap = True
            return

        self.CL_increase_TO = self.calculate_CLmax_increase(self.tot_flap_area,self.clmax_increase*0.6)
        self.CL_max_TO = self.CLMax_clean + self.CL_increase_TO
        print(self.CL_max_TO)

        self.CL_increase = self.calculate_CLmax_increase(self.tot_flap_area,self.clmax_increase)

        self.actual_flap_area = self.chord_span_function((self.flap_end - self.flap_start)/2)*self.rel_flap_chord*(self.flap_end - self.flap_start)
        # print(f"TO CLmax: {self.CL_max_TO}")
        # print(f"Flap area: {self.flap_area}")
        # print(f"Flap chord: {self.flap_chord}")
        # print(f"Flap end: {self.flap_end}")
        # print(f"CL increase: {self.CL_increase}")

    def main(self):
        self.calculate_roll_rate()
        self.calculate_aileron_size()
        self.calculate_flap_size()
        self.update_attributes()
        self.aircraft_data.save_design(design_file=self.design_file)

    def update_attributes(self):

        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b1'] = self.aileron_start
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['b2'] = self.aileron_end
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['Swa'] = self.aileroned_area
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['aileron_lift'] = self.aileron_lift
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['area_single'] = self.aileron_area
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['Swa'] = self.aileroned_area
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['Clda'] = self.Clda
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['roll_rate'] = np.rad2deg(self.roll_rate)
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['bank_angle'] = np.rad2deg(self.bank_angle)
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['turn_radius'] = self.turn_radius
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['object_distance'] = self.object_distance
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['Clda'] = self.Clda
        self.aircraft_data.data['outputs']['aerodynamic_stability_coefficients_asym']['C_l_delta_a'] = -self.Clda
        self.aircraft_data.data['outputs']['HLD']['b1'] = self.flap_start
        self.aircraft_data.data['outputs']['HLD']['b2'] = self.flap_end
        self.aircraft_data.data['outputs']['HLD']['Swf_single'] = self.flap_area
        self.aircraft_data.data['outputs']['HLD']['flap_area'] = self.actual_flap_area
        self.aircraft_data.data['outputs']['control_surfaces']['aileron']['aileron_lift'] = abs(self.aileron_lift)
        self.aircraft_data.data['inputs']['CLmax_takeoff'] = self.CL_max_TO
        self.aircraft_data.data['inputs']['CLmax_landing'] = self.CLMax_landing


        if self.LE_flap:
            self.aircraft_data.data['outputs']['HLD']['b2'] = self.LE_flap_end
            self.aircraft_data.data['outputs']['HLD']['b1'] = self.flap_start
            self.aircraft_data.data['outputs']['HLD']['LE_flap_area'] = self.actual_LE_flap_area
            self.aircraft_data.data['outputs']['HLD']['Swf_LE_single'] = self.LE_flap_area

    def plot_wing(self):

        self.b_array = np.arange(0, self.b/2, 0.1)
        leading_edge = self.root_chord/2 - np.tan(np.deg2rad(self.sweep)) * self.b_array
        y_tip_LE = leading_edge[-1]
        y_tip_TE = y_tip_LE - self.tip_chord

        y_root_LE = self.root_chord/2 
        y_root_TE = y_root_LE - self.root_chord

        y_root_LE_aileron = self.leading_edge(self.aileron_start)-(1-self.aileron_chord_ratio)*self.chord_span_function(self.aileron_start)
        y_root_TE_aileron = y_root_LE_aileron - self.chord_span_function(self.aileron_start)*self.aileron_chord_ratio

        y_tip_LE_aileron = self.leading_edge(self.aileron_end)-(1-self.aileron_chord_ratio)*self.chord_span_function(self.aileron_end)
        y_tip_TE_aileron = y_tip_LE_aileron - self.chord_span_function(self.aileron_end)*self.aileron_chord_ratio

        y_root_LE_flap = self.leading_edge(self.flap_start)-(1-self.rel_flap_chord)*self.chord_span_function(self.flap_start)
        y_root_TE_flap = y_root_LE_flap - self.chord_span_function(self.flap_start)*self.rel_flap_chord

        y_tip_LE_flap = self.leading_edge(self.flap_end)-(1-self.rel_flap_chord)*self.chord_span_function(self.flap_end)
        y_tip_TE_flap = y_tip_LE_flap - self.chord_span_function(self.flap_end)*self.rel_flap_chord

        fig, ax = plt.subplots()
        # Fuselage centerline
        ax.plot([self.w_fuselage/2, self.w_fuselage/2], [-10, 10], color='green', linestyle='--')
        ax.plot([-self.w_fuselage/2, -self.w_fuselage/2], [-10, 10], color='green', linestyle='--')
        ax.plot(self.b_array, leading_edge, color='blue')
        ax.plot(-self.b_array, leading_edge, color='blue')  # Mirror

        ax.plot([0,0],[y_root_LE, y_root_TE], color='blue')  # Centerline, no mirror needed

        ax.plot([self.b/2, self.b/2], [y_tip_LE,y_tip_TE], color='blue')
        ax.plot([-self.b/2, -self.b/2], [y_tip_LE,y_tip_TE], color='blue')  # Mirror

        ax.plot([0, self.b/2], [y_root_TE, y_tip_TE], color='blue')
        ax.plot([0, -self.b/2], [y_root_TE, y_tip_TE], color='blue')  # Mirror

        # Ailerons
        ax.plot([self.aileron_start, self.aileron_start], [y_root_LE_aileron, y_root_TE_aileron], color='red')
        ax.plot([-self.aileron_start, -self.aileron_start], [y_root_LE_aileron, y_root_TE_aileron], color='red')  # Mirror

        ax.plot([self.aileron_end, self.aileron_end], [y_tip_LE_aileron, y_tip_TE_aileron], color='red')
        ax.plot([-self.aileron_end, -self.aileron_end], [y_tip_LE_aileron, y_tip_TE_aileron], color='red')  # Mirror

        ax.plot([self.aileron_start, self.aileron_end], [y_root_LE_aileron, y_tip_LE_aileron], color='red')
        ax.plot([-self.aileron_start, -self.aileron_end], [y_root_LE_aileron, y_tip_LE_aileron], color='red')  # Mirror

        # Flaps
        ax.plot([self.flap_start, self.flap_start], [y_root_LE_flap, y_root_TE_flap], color='orange')
        ax.plot([-self.flap_start, -self.flap_start], [y_root_LE_flap, y_root_TE_flap], color='orange')

        ax.plot([self.flap_end, self.flap_end], [y_tip_LE_flap, y_tip_TE_flap], color='orange')
        ax.plot([-self.flap_end, -self.flap_end], [y_tip_LE_flap, y_tip_TE_flap], color='orange')

        ax.plot([self.flap_start, self.flap_end], [y_root_LE_flap, y_tip_LE_flap], color='orange')
        ax.plot([-self.flap_start, -self.flap_end], [y_root_LE_flap, y_tip_LE_flap], color='orange')


        if self.LE_flap:
            y_root_LE_LE_flap = self.leading_edge(self.flap_start)
            y_root_TE_LE_flap = y_root_LE_LE_flap - self.chord_span_function(self.flap_start)*self.rel_LE_flap_chord

            y_tip_LE_LE_flap = self.leading_edge(self.LE_flap_end)
            y_tip_TE_LE_flap = y_tip_LE_LE_flap - self.chord_span_function(self.LE_flap_end)*self.rel_LE_flap_chord
            ax.plot([self.flap_start, self.LE_flap_end], [y_root_TE_LE_flap, y_tip_TE_LE_flap], color='purple')
            ax.plot([-self.flap_start, -self.LE_flap_end], [y_root_TE_LE_flap, y_tip_TE_LE_flap], color='purple')

            ax.plot([self.flap_start, self.flap_start], [y_root_LE_LE_flap, y_root_TE_LE_flap], color='purple')
            ax.plot([-self.flap_start, -self.flap_start], [y_root_LE_LE_flap, y_root_TE_LE_flap], color='purple')

            ax.plot([self.LE_flap_end, self.LE_flap_end], [y_tip_LE_LE_flap, y_tip_TE_LE_flap], color='purple')
            ax.plot([-self.LE_flap_end, -self.LE_flap_end], [y_tip_LE_LE_flap, y_tip_TE_LE_flap], color='purple')


        ax.set_aspect('equal', adjustable='box')
        ax.set_title("Wing Planform with Sweep")
        ax.set_xlabel("Spanwise Direction (b)")
        ax.set_ylabel("Chord (m)")
        ax.set_ylim(-10,10)
        ax.grid(True)

        plt.show()

if __name__ == "__main__":
    aircraft_data = Data("design3.json")
    control_surface = AileronHLD(aircraft_data=aircraft_data)
    control_surface.main()
    control_surface.plot_wing()
    # print(f"Bank angle: {np.rad2deg(control_surface.bank_angle)}")
    # print(f"Yaw rate: {np.rad2deg(yaw_rate)}")
    # print(f"Roll rate: {np.rad2deg(roll_rate)}")
    # print(f"turn time: {control_surface.turn_time/2}")
    # print(f"turn radius: {control_surface.turn_radius}")

    #print(control_surface.aileron_start)

