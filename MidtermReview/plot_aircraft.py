import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, WingType
import numpy as np
import matplotlib.pyplot as plt


class AircraftPlotter:
    def __init__(self, aircraft_data: Data, count: int = 0) -> None:
        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.wing_type = WingType[aircraft_data.data['inputs']['wing_type']]
        self.S = aircraft_data.data['outputs']['max']['S']
        self.aspect_ratio = aircraft_data.data['inputs']['aspect_ratio']
        self.count = count

        self.fuselage_length = self.aircraft_data.data['outputs']['general']['l_fuselage']
        self.x_c = 0 #Where along the wing we want to look, so in this case 0 is the leading edge of the wing

        self.d_fuselage = self.aircraft_data.data['outputs']['general']['d_fuselage']
        self.chord_root = self.aircraft_data.data['outputs']['wing_design']['chord_root']
        self.chord_tip = self.aircraft_data.data['outputs']['wing_design']['chord_tip']
        self.b = self.aircraft_data.data['outputs']['wing_design']['b']

        self.sweep_x_c = self.aircraft_data.data['outputs']['wing_design']['sweep_x_c']

        self.chord_root_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root']
        self.chord_tip_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip']
        self.b_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.sweep_x_c_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['sweep']
        
        self.chord_root_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
        self.chord_tip_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
        self.b_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        self.sweep_x_c_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep']
        self.LE_pos_wing = self.aircraft_data.data['outputs']['wing_design']['X_LE']
        self.LE_pos_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['LE_pos']
        self.LE_pos_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']

    def plot_wing(self, ax):
        b_array = np.arange(0, self.b/2, 0.1)
        b_h_array = np.arange(0, self.b_h/2, 0.1)
        b_v_array = np.arange(0, self.b_v, 0.1)
        leading_edge = self.chord_root/2 - np.tan(np.deg2rad(self.sweep_x_c)) * b_array

        y_tip_LE = leading_edge[-1]
        y_tip_TE = y_tip_LE - self.chord_tip

        y_root_LE = self.chord_root/2 
        y_root_TE = y_root_LE - self.chord_root

        leading_edge_h = y_root_LE -(self.LE_pos_h -self.LE_pos_wing) - np.tan(np.deg2rad(self.sweep_x_c_h)) * b_h_array
        y_h_tip_LE = leading_edge_h[-1]
        y_h_tip_TE = y_h_tip_LE - self.chord_tip_h
        
        y_h_root_LE = y_root_LE - (self.LE_pos_h - self.LE_pos_wing)
        y_h_root_TE = y_h_root_LE - self.chord_root_h

        colors = ['blue', 'orange', 'green']
        designs = ['2', '7', '10']

        # Main planform
        ax.plot(b_array, leading_edge, label=f'Design {designs[self.count-1]}', color = colors[self.count-1])
        ax.plot(-b_array, leading_edge, color = colors[self.count-1])  # Mirror

        ax.plot([0, 0], [y_root_LE, y_root_TE], color = colors[self.count-1])
        ax.plot([self.b/2, self.b/2], [y_tip_LE, y_tip_TE], color = colors[self.count-1])
        ax.plot([-self.b/2, -self.b/2], [y_tip_LE, y_tip_TE], color = colors[self.count-1])

        ax.plot([0, self.b/2], [y_root_TE, y_tip_TE], color = colors[self.count-1])
        ax.plot([0, -self.b/2], [y_root_TE, y_tip_TE], color = colors[self.count-1])

        # Horizontal tail
        ax.plot(b_h_array, leading_edge_h, color = colors[self.count-1])
        ax.plot(-b_h_array, leading_edge_h, color = colors[self.count-1])  # Mirror
        ax.plot([0, 0], [y_h_root_LE, y_h_root_TE], color = colors[self.count-1])
        ax.plot([self.b_h/2, self.b_h/2], [y_h_tip_LE, y_h_tip_TE], color = colors[self.count-1])
        ax.plot([-self.b_h/2, -self.b_h/2], [y_h_tip_LE, y_h_tip_TE], color = colors[self.count-1])

        ax.plot([0, self.b_h/2], [y_h_root_TE, y_h_tip_TE], color = colors[self.count-1])
        ax.plot([0, -self.b_h/2], [y_h_root_TE, y_h_tip_TE], color = colors[self.count-1])

    def plot_vertical_tail(self, ax):
        b_v_array = np.arange(0, self.b_v + 0.01, 0.1)  # Spanwise direction (y-axis)

        # Compute leading edge position, sweeping backward (left)
        leading_edge_v = -0.5 * self.chord_root_v + np.tan(np.deg2rad(self.sweep_x_c_v)) * b_v_array

        # Root leading/trailing edges (centered around 0)
        y_root_LE = -0.5 * self.chord_root_v
        y_root_TE = 0.5 * self.chord_root_v

        # Tip leading/trailing edges
        y_tip_LE = leading_edge_v[-1]
        y_tip_TE = y_tip_LE + self.chord_tip_v

        colors = ['blue', 'orange', 'green']
        designs = ['2', '7', '10']

        # Draw outline of vertical tail (rotated: x = chord, y = span)
        ax.plot(leading_edge_v, b_v_array, color=colors[self.count - 1], label=f'Design {designs[self.count - 1]}')  # Leading edge
        ax.plot([y_root_LE, y_root_TE], [0, 0], color=colors[self.count - 1])  # Root chord line
        ax.plot([y_tip_LE, y_tip_TE], [self.b_v, self.b_v], color=colors[self.count - 1])  # Tip chord line
        ax.plot([y_root_TE, y_tip_TE], [0, self.b_v], color=colors[self.count - 1])  # Trailing edge




if __name__ == "__main__":
    fig, ax = plt.subplots()

    for i in range(1, 4):
        data = Data(f'design{i}.json')
        aircraft = AircraftPlotter(data, count=i)
        aircraft.plot_wing(ax)

    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Wing Planform Including Empennage")
    ax.set_xlabel("Spanwise Direction (m)")
    ax.set_ylabel("Chordwise Direction (m)")
    ax.set_ylim(-50, 20)
    ax.legend(loc='upper right', bbox_to_anchor=(1.4, 1))
    plt.show()

    fig, ax = plt.subplots()
    for i in range(1, 4):
        data = Data(f'design{i}.json')
        aircraft = AircraftPlotter(data, count=i)
        aircraft.plot_vertical_tail(ax)
    
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Vertical Tail Planform")
    ax.set_ylabel("Spanwise Direction (m)")
    ax.set_xlabel("Chordwise Direction (m)")
    ax.set_ylim(0, 15)
    ax.set_xlim(-10, 10)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1))
    plt.show()
