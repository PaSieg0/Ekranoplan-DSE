import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import matplotlib.pyplot as plt
from utils import Data, MissionType



class PlanformPlot:
    def __init__(self, aircraft_data):
        self.aircraft_data = aircraft_data
        
        self.wing_S = aircraft_data.data['outputs']['wing_design']['S']
        self.wing_b = aircraft_data.data['outputs']['wing_design']['b']
        self.wing_c_root = aircraft_data.data['outputs']['wing_design']['chord_root']
        self.wing_c_tip = aircraft_data.data['outputs']['wing_design']['chord_tip']
        self.wing_sweep_LE = aircraft_data.data['outputs']['wing_design']['sweep_x_c']
        self.wing_sweep_c_4 = aircraft_data.data['outputs']['wing_design']['sweep_c_4']
        self.wing_X_LEMAC = aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.wing_y_mac = aircraft_data.data['outputs']['wing_design']['y_MAC']
        self.wing_MAC = aircraft_data.data['outputs']['wing_design']['MAC']
        self.wing_X_LE = self.wing_X_LEMAC - self.wing_y_mac * np.tan(np.deg2rad(self.wing_sweep_LE))

        self.S_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']
        self.b_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.h_aspect_ratio = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['aspect_ratio']
        self.h_LE = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos']
        self.h_sweep = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['sweep_c_4']
        self.h_taper = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper']
        self.h_c_root = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
        self.h_c_tip = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip']

        self.v_LE = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']
        self.b_v = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        self.v_sweep_LE = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep']
        self.v_quarter_tip = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['quarter_tip']
        self.v_cr = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
        self.v_ct = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']


        self.l_fuselage = aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        self.w_fuselage = aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']
        self.l_forebody = aircraft_data.data['outputs']['fuselage_dimensions']['l_forebody']
        self.l_nose = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose']
        self.l_tailcone = aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone']
        self.l_afterbody = aircraft_data.data['outputs']['fuselage_dimensions']['l_afterbody']

        self.most_aft_cg = aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        self.most_forward_cg = aircraft_data.data['outputs']['cg_range']['most_forward_cg']

        self.cargo_start = aircraft_data.data['outputs']['fuselage_dimensions']['l_nose'] + aircraft_data.data['outputs']['fuselage_dimensions']['cargo_distance_from_nose']
        self.cargo_length = aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']
        self.cargo_end = self.cargo_start + self.cargo_length
        self.cargo_height = aircraft_data.data['outputs']['fuselage_dimensions']['cargo_height']
        self.cargo_width = aircraft_data.data['outputs']['fuselage_dimensions']['cargo_width']


    def plot_init(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        # self.ax.set_xlim(-self.l_fuselage - 10, self.l_fuselage + 10)
        # self.ax.set_ylim(-self.h_fuselage - 5, self.h_fuselage + 5)
        self.ax.set_xlabel('Lateral [m]')
        self.ax.set_ylabel('Longitudinal [m]')
        self.ax.set_title('Aircraft Planform')

    def plot_wing(self) -> None:
        sweep_offset = (self.wing_b/2) * np.tan(np.deg2rad(self.wing_sweep_LE))

        self.ax.plot([0, self.wing_b/2], [self.wing_X_LE, self.wing_X_LE + sweep_offset], 'k-', lw=2)
        self.ax.plot([self.wing_b/2, self.wing_b/2], [self.wing_X_LE + sweep_offset, self.wing_X_LE + sweep_offset + self.wing_c_tip], 'k-', lw=2)
        self.ax.plot([self.wing_b/2, 0], [self.wing_X_LE + sweep_offset + self.wing_c_tip, self.wing_X_LE + self.wing_c_root], 'k-', lw=2)
        self.ax.plot([0, -self.wing_b/2], [self.wing_X_LE, self.wing_X_LE + sweep_offset], 'k-', lw=2)
        self.ax.plot([-self.wing_b/2, -self.wing_b/2], [self.wing_X_LE + sweep_offset, self.wing_X_LE + sweep_offset + self.wing_c_tip], 'k-', lw=2)
        self.ax.plot([-self.wing_b/2, 0], [self.wing_X_LE + sweep_offset + self.wing_c_tip, self.wing_X_LE + self.wing_c_root], 'k-', lw=2)

    def plot_empennage(self) -> None:
        self.h_sweep_LE = np.rad2deg(np.arctan(
            np.tan(np.deg2rad(self.h_sweep)) -
            ((4 / self.h_aspect_ratio) * (0 - 0.25) *
             ((1 - self.h_taper) / (1 + self.h_taper)))
        ))
        sweep_offset_h = (self.b_h/2) * np.tan(np.deg2rad(self.h_sweep_LE))
        sweep_offset_v = (self.b_v) * np.tan(np.deg2rad(self.v_sweep_LE))

        self.ax.plot([0, self.b_h/2], [self.h_LE, self.h_LE + sweep_offset_h], 'k-', lw=2)
        self.ax.plot([self.b_h/2, self.b_h/2], [self.h_LE + sweep_offset_h, self.h_LE + sweep_offset_h + self.h_c_tip], 'k-', lw=2)
        self.ax.plot([self.b_h/2, 0], [self.h_LE + sweep_offset_h + self.h_c_tip, self.h_LE + self.h_c_root], 'k-', lw=2)

        self.ax.plot([0, -self.b_h/2], [self.h_LE, self.h_LE + sweep_offset_h], 'k-', lw=2)
        self.ax.plot([-self.b_h/2, -self.b_h/2], [self.h_LE + sweep_offset_h, self.h_LE + sweep_offset_h + self.h_c_tip], 'k-', lw=2)
        self.ax.plot([-self.b_h/2, 0], [self.h_LE + sweep_offset_h + self.h_c_tip, self.h_LE + self.h_c_root], 'k-', lw=2)

        self.ax.plot([self.w_fuselage/2, -self.w_fuselage/2], [self.v_LE, self.v_LE], 'k-', lw=2, label='Vertical Tail Root LE')
        self.ax.plot([self.w_fuselage/2, -self.w_fuselage/2], [self.v_LE + self.v_cr, self.v_LE + self.v_cr], 'k-', lw=2, label='Vertical Tail Root TE')

        self.ax.plot([self.w_fuselage/2, -self.w_fuselage/2], [self.v_LE + sweep_offset_v, self.v_LE + sweep_offset_v], 'r-', lw=2, label='Vertical Tail Root LE')
        self.ax.plot([self.w_fuselage/2, -self.w_fuselage/2], [self.v_LE + sweep_offset_v + self.v_ct, self.v_LE + sweep_offset_v + self.v_ct], 'r-', lw=2, label='Vertical Tail Tip TE')

    def plot_cg(self) -> None:
        self.ax.scatter([0, 0], [self.most_aft_cg, self.most_forward_cg], color='red', label='CG Range', zorder=5)

    def plot_fuselage(self) -> None:
        theta = np.linspace(0, -np.pi, 100)
        x_nose = (self.w_fuselage / 2) * np.cos(theta)
        y_nose = (self.w_fuselage / 2) * np.sin(theta) + self.w_fuselage/2
        self.ax.plot(x_nose, y_nose, 'k-', lw=2)

        # Connect nose to start of forebody (vertical lines from nose endpoints to y = l_nose)
        self.ax.plot([self.w_fuselage/2, self.w_fuselage/2], [self.w_fuselage/2, self.l_nose], 'k-', lw=2)
        self.ax.plot([-self.w_fuselage/2, -self.w_fuselage/2], [self.w_fuselage/2, self.l_nose], 'k-', lw=2)

        # Forebody
        self.ax.plot([self.w_fuselage/2, self.w_fuselage/2], [self.l_nose, self.l_nose + self.l_forebody], 'k-', lw=2)
        self.ax.plot([-self.w_fuselage/2, -self.w_fuselage/2], [self.l_nose, self.l_nose + self.l_forebody], 'k-', lw=2)
        # Step
        self.ax.plot([self.w_fuselage/2, -self.w_fuselage/2], [self.l_nose + self.l_forebody, self.l_nose + self.l_forebody], 'k--', lw=2, label='Step')
        # Tailcone
        self.ax.plot([self.w_fuselage/2, self.w_fuselage/2], [self.l_nose + self.l_forebody, self.l_nose + self.l_forebody + self.l_tailcone], 'k-', lw=2)
        self.ax.plot([-self.w_fuselage/2, -self.w_fuselage/2], [self.l_nose + self.l_forebody, self.l_nose + self.l_forebody + self.l_tailcone], 'k-', lw=2)
        # Afterbody
        self.ax.plot([self.w_fuselage/2, 0], [self.l_nose + self.l_forebody + self.l_tailcone, self.l_nose + self.l_forebody + self.l_tailcone + self.l_afterbody], 'k-', lw=2)
        self.ax.plot([-self.w_fuselage/2, 0], [self.l_nose + self.l_forebody + self.l_tailcone, self.l_nose + self.l_forebody + self.l_tailcone + self.l_afterbody], 'k-', lw=2)

    def plot_MAC(self) -> None:
        self.ax.plot([self.wing_y_mac, self.wing_y_mac], [self.wing_X_LEMAC, self.wing_X_LEMAC + self.wing_MAC], 'b--', lw=2, label='MAC')
        self.ax.scatter([self.wing_y_mac], [self.wing_X_LEMAC + self.wing_MAC*0.277], color='b', label='MAC AC Center', zorder=5)

    def plot_cargo(self) -> None:
        self.ax.plot([self.cargo_width/2, self.cargo_width/2], [self.cargo_start, self.cargo_end], 'g-', lw=2)
        self.ax.plot([-self.cargo_width/2, -self.cargo_width/2], [self.cargo_start, self.cargo_end], 'g-', lw=2)
        self.ax.plot([self.cargo_width/2, -self.cargo_width/2], [self.cargo_start, self.cargo_start], 'g-', lw=2)
        self.ax.plot([self.cargo_width/2, -self.cargo_width/2], [self.cargo_end, self.cargo_end], 'g-', lw=2)
        self.ax.fill_betweenx(
            [self.cargo_start, self.cargo_end],
            -self.cargo_width/2,
            self.cargo_width/2,
            color='lightgreen',
            alpha=0.5,
            label='Cargo Area Fill'
        )
        # self.ax.plot


    def show_plot(self) -> None:
        self.ax.grid()
        self.ax.legend()
        self.ax.set_aspect('equal', adjustable='box')
        plt.show()


if __name__ == "__main__":
    # Assuming aircraft_data is already defined and contains the necessary data
    aircraft_data = Data('design3.json')  # Load your aircraft data here
    planform_plot = PlanformPlot(aircraft_data)
    planform_plot.plot_init()
    planform_plot.plot_wing()
    planform_plot.plot_cg()
    planform_plot.plot_empennage()
    planform_plot.plot_fuselage()
    planform_plot.plot_MAC()
    planform_plot.plot_cargo()
    planform_plot.show_plot()