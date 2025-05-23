import numpy as np
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, WingType, ISA
from matplotlib import pyplot as plt

class WingPlanform:
    def __init__(self, 
                 aircraft_data: Data,
                 count: int = 0
                 ) -> None:
        
        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.wing_type = WingType[aircraft_data.data['inputs']['wing_type']]
        self.S = aircraft_data.data['outputs']['max']['S']
        self.aspect_ratio = aircraft_data.data['inputs']['aspect_ratio']
        self.count = count

        self.fuselage_length = self.aircraft_data.data['outputs']['general']['l_fuselage']
        self.x_c = 0 #Where along the wing we want to look, so in this case 0 is the leading edge of the wing
        self.x_c_OEW_cg = self.aircraft_data.data['inputs']['xc_OEW'] # x/c OEW CG position
        self.x_c_wing_cg = self.aircraft_data.data['inputs']['xc_wing'] # x/c wing CG position
        self.fuse_x_cg = self.aircraft_data.data['inputs']['xc_fuselage']  # normalized CG position
        self.mass_fraction_wing = self.aircraft_data.data['inputs']['mf_wing'] # mass fraction of the wing
        self.mass_fraction_fuse = self.aircraft_data.data['inputs']['mf_fuselage'] # mass fraction of the fuselage
        self.d_fuselage = self.aircraft_data.data['outputs']['general']['d_fuselage']

    def calculate(self):
        self.mach = ISA(self.aircraft_data.data['inputs']['cruise_altitude']).Mach(self.aircraft_data.data['requirements']['cruise_speed'])
        if self.mach <= 0.7:
            self.sweep_c_4 = 0  # degrees
        else: 
            self.sweep_c_4 = 0.75 * 0.935/(self.mach+0.03)

        self.taper_ratio = 0.2*(2-self.sweep_c_4*np.pi/180)

        if self.wing_type == WingType.HIGH:
            self.dihedral = 1  # degrees
        elif self.wing_type == WingType.LOW:
            self.dihedral = 5  # degrees
        else:
            raise ValueError("Invalid wing type")
        
        self.b = self.aircraft_data.data['outputs']['max']['b']
        self.sweep_x_c = np.rad2deg(np.arctan(
            np.tan(np.deg2rad(self.sweep_c_4)) -
            ((4 / self.aspect_ratio) * (self.x_c - 0.25) *
             ((1 - self.taper_ratio) / (1 + self.taper_ratio)))
        ))

        self.chord_root = 2 * self.S / (self.b * (1 + self.taper_ratio))
        self.chord_tip = self.taper_ratio * self.chord_root

        # Correct MAC formula
        self.MAC = (2 / 3) * self.chord_root * ((1 + self.taper_ratio + self.taper_ratio**2) / (1 + self.taper_ratio))
        self.y_MAC = (self.b / 6) * (1 + 2 * self.taper_ratio) / (1 + self.taper_ratio)

        self.X_LEMAC = (self.fuse_x_cg * self.fuselage_length) + \
                  self.MAC * ((self.x_c_wing_cg * self.mass_fraction_wing / self.mass_fraction_fuse) -
                         self.x_c_OEW_cg * (1 + self.mass_fraction_wing / self.mass_fraction_fuse))

        self.X_LE = self.X_LEMAC - self.y_MAC * np.tan(np.deg2rad(self.sweep_x_c))
        #self.oswald = 4.61*(1-0.045*self.aspect_ratio**0.68)*(np.cos(np.deg2rad(self.sweep_x_c)))**0.15 -3.1
        self.update_design_data()
        self.aircraft_data.save_design(self.design_file)


    def update_design_data(self):
        self.aircraft_data.data['outputs']['wing_design']['taper_ratio'] = self.taper_ratio
        self.aircraft_data.data['outputs']['wing_design']['sweep_c_4'] = self.sweep_c_4
        self.aircraft_data.data['outputs']['wing_design']['dihedral'] = self.dihedral
        self.aircraft_data.data['outputs']['wing_design']['sweep_x_c'] = self.sweep_x_c
        self.aircraft_data.data['outputs']['wing_design']['chord_root'] = self.chord_root
        self.aircraft_data.data['outputs']['wing_design']['chord_tip'] = self.chord_tip
        self.aircraft_data.data['outputs']['wing_design']['y_MAC'] = self.y_MAC
        self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] = self.X_LEMAC
        self.aircraft_data.data['outputs']['wing_design']['X_LE'] = self.X_LE
        self.aircraft_data.data['outputs']['wing_design']['MAC'] = self.MAC
        self.aircraft_data.data['outputs']['wing_design']['S'] = self.S
        self.aircraft_data.data['outputs']['wing_design']['b'] = self.b
        self.aircraft_data.data['outputs']['wing_design']['aspect_ratio'] = self.aspect_ratio
        #self.aircraft_data.data['inputs']['oswald_factor'] = self.oswald

    def get_half_span_points(self):
        """Return array of points for half span (positive side only)"""
        return np.arange(0, self.b/2 + 0.05, 0.05)  # include endpoint

    def plot_wing(self, ax):
        b_array = self.get_half_span_points()
        # Calculate actual number of points (2 * half-span points - 1 to avoid double counting center)
        total_points = len(b_array) 
        print(f"\nWing planform array length (full span): {total_points} points")
        print(f"First x-coordinate: {b_array[0]} m")
        print(f"Last x-coordinate: {b_array[-1]} m")
        leading_edge = self.chord_root/2 - np.tan(np.deg2rad(self.sweep_x_c)) * b_array
        y_tip_LE = leading_edge[-1]
        y_tip_TE = y_tip_LE - self.chord_tip

        y_root_LE = self.chord_root/2 
        y_root_TE = y_root_LE - self.chord_root

        # Calculate chord at each spanwise position
        chord_distribution = np.zeros_like(b_array)
        for i, x in enumerate(b_array):
            # Linear interpolation between root and tip chord
            chord_distribution[i] = self.chord_root + (self.chord_tip - self.chord_root) * (x / (self.b/2))

        colors = ['blue', 'orange', 'green']
        designs = ['2', '7', '10']

        # Main planform
        ax.plot(b_array, leading_edge, label=f'Design {designs[self.count-1]}', color = colors[self.count-1])
        # ax.plot(-b_array, leading_edge, color = colors[self.count-1])  # Mirror

        ax.plot([0, 0], [y_root_LE, y_root_TE], color = colors[self.count-1])
        ax.plot([self.b/2, self.b/2], [y_tip_LE, y_tip_TE], color = colors[self.count-1])
        # ax.plot([-self.b/2, -self.b/2], [y_tip_LE, y_tip_TE], color = colors[self.count-1])

        ax.plot([0, self.b/2], [y_root_TE, y_tip_TE], color = colors[self.count-1])
        # ax.plot([0, -self.b/2], [y_root_TE, y_tip_TE], color = colors[self.count-1])

        return b_array, chord_distribution


if __name__ == "__main__":
    # data = Data('design1.json')
    # wing = WingPlanform(data)
    # wing.calculate()
    # print(f"x_LE: {wing.X_LE}")
    # print(f"x_LEMAC: {wing.X_LEMAC}")
    # print(f"y_MAC: {wing.y_MAC}")
    # print(f"chord_root: {wing.chord_root}")
    # print(f"chord_tip: {wing.chord_tip}")
    # print(f"sweep_x_c: {wing.sweep_x_c}")
    fig, ax = plt.subplots()

    for i in range(1, 4):
        data = Data(f'design{i}.json')
        wing = WingPlanform(data, count=i)
        wing.calculate()
        wing.plot_wing(ax=ax)

    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Wing Planform Comparison")
    ax.set_xlabel("Spanwise Direction (m)")
    ax.set_ylabel("Chordwise Direction (m)")
    ax.set_ylim(-20, 20)
    ax.legend(loc='upper right', bbox_to_anchor=(1.2, 1))
    ax.legend()
    plt.show()
