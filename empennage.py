import os
import pandas as pd
import numpy as np
from utils import Data
from enum import Enum, auto

class EmpType(Enum):
    CRUCIFORM = auto()
    T_TAIL = auto()
    CONVENTIONAL = auto()
    H_TAIL = auto()
    NONE = auto()
class Empennage:
    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.tail_type = EmpType[aircraft_data.data['inputs']['tail_type']]

        self.d_fuselage = self.aircraft_data.data['outputs']['general']['d_fuselage']
        self.sweep_h = self.aircraft_data.data['inputs']['sweep_h']
        self.sweep_v = self.aircraft_data.data['inputs']['sweep_v']
        self.aspect_h = self.aircraft_data.data['inputs']['aspect_h']
        self.aspect_v = self.aircraft_data.data['inputs']['aspect_v']
        self.sweep_hc4 = self.aircraft_data.data['inputs']['sweep_hc4']
        self.taper_h = self.aircraft_data.data['inputs']['taper_h']
        self.taper_v = self.aircraft_data.data['inputs']['taper_v']
        self.b = self.aircraft_data.data['outputs']['max']['b']
        self.n_fuselages = self.aircraft_data.data['inputs']['n_fuselages']

        self.v_position = self.aircraft_data.data['inputs']['v_position']
        self.h_position = self.aircraft_data.data['inputs']['h_position']
        self.l_fuselage = self.aircraft_data.data['outputs']['general']['l_fuselage']
        self.aft_cg = self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']

        self.S = self.aircraft_data.data['outputs']['max']['S']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']

        self.V_h = self.aircraft_data.data['inputs']['V_h']
        self.V_v = self.aircraft_data.data['inputs']['V_v']

        self.n_v_tails = 1
        self.aft_clearance = 0.3

    def fus_separation(self):
        return max(self.b/(self.n_fuselages+1),2*self.d_fuselage)

    def chord_h(self, y):
        # y is spanwise position from centerline (0 to b_h/2)
        c_r = self.chord_root_h
        taper = self.taper_h
        b = self.b_h
        return c_r * (1 - (1 - taper) * 2 * y / b)
    
    def chord_v(self, y):
        c_r = self.chord_root_v
        taper = self.taper_v
        b = self.b_v
        return c_r * (1 - (1 - taper) * 2 * y / b)

    def calculate_tail_areas(self):
        if self.tail_type == EmpType.NONE:
            return
        
        self.l_h = (self.h_position*self.l_fuselage - self.aft_cg)
        self.l_v = (self.v_position*self.l_fuselage - self.aft_cg)
        self.S_h = self.V_h * self.S * self.MAC / self.l_h
        self.S_v = self.V_v * self.S * self.b / self.l_v

        self.b_h = np.sqrt(self.S_h * self.aspect_h)
        self.b_v = np.sqrt(self.S_v * self.aspect_v)

        self.chord_root_h = 2 * self.S_h / (self.b_h * (1 + self.taper_h))
        self.chord_tip_h = self.taper_h * self.chord_root_h

        self.chord_root_v = 2 * self.S_v / (self.b_v * (1 + self.taper_v))
        self.chord_tip_v = self.taper_v * self.chord_root_v

        self.mac_h = (2 / 3) * self.chord_root_h * ((1 + self.taper_h + self.taper_h**2) / (1 + self.taper_h))
        self.mac_v = (2 / 3) * self.chord_root_v * ((1 + self.taper_v + self.taper_v**2) / (1 + self.taper_v))

        if self.tail_type == EmpType.CRUCIFORM:
            self.h_tail_pos = self.b_v/2 + self.d_fuselage
            self.v_position = self.l_fuselage - self.aft_clearance - 3/4*self.mac_v
            self.h_position = (self.v_position + np.tan(self.sweep_v*np.pi/180)*self.b_v/2 + 1/4*self.chord_root_h)/self.l_fuselage
            self.v_position = self.v_position/self.l_fuselage
        
        elif self.tail_type == EmpType.T_TAIL:
            self.h_tail_pos = self.b_v + self.d_fuselage
            self.taper_v = min(self.chord_root_h/self.chord_root_v, 0.7)
            self.chord_tip_v = self.taper_v * self.chord_root_v

            self.v_position = self.l_fuselage - self.aft_clearance - 3/4*self.mac_v
            self.h_position = (self.v_position + np.tan(self.sweep_v*np.pi/180)*self.b_v + 1/4*self.chord_root_h)/self.l_fuselage
            self.v_position = self.v_position/self.l_fuselage


        elif self.tail_type == EmpType.CONVENTIONAL:
            self.h_tail_pos = self.d_fuselage

            self.v_position = (self.l_fuselage - self.aft_clearance - 3/4*self.mac_v)/self.l_fuselage
            self.h_position = (self.l_fuselage - self.aft_clearance - 3/4*self.mac_h)/self.l_fuselage

        elif self.tail_type == EmpType.H_TAIL:
            self.n_v_tails = 2
            self.S_v = self.S_v / self.n_v_tails
            self.b_v = self.b_v / self.n_v_tails

            self.fus_sep = self.fus_separation()
            span_wise_v_pos = self.fus_sep / 2 + self.d_fuselage/2

            self.chord_tip_v = self.chord_h(span_wise_v_pos)
            self.chord_root_v = 2 * self.S_v / (self.b_v * (1 + self.taper_v))
            self.taper_v = min(self.chord_root_h/self.chord_root_v, 0.7)
            self.mac_v = (2 / 3) * self.chord_root_v * ((1 + self.taper_v + self.taper_v**2) / (1 + self.taper_v))
            self.h_tail_pos = self.d_fuselage + self.b_v

            self.v_position = self.l_fuselage - self.aft_clearance - 3/4*self.mac_v
            self.h_position = (self.v_position + np.tan(self.sweep_v*np.pi/180)*self.b_v + 1/4*self.chord_root_h)/self.l_fuselage
            self.v_position = self.v_position/self.l_fuselage


    def run_iteration(self):
        if self.tail_type == EmpType.NONE:
            return
        self.iteration_number = 0
        self.max_iterations = 100
        self.tolerance = 0.001
        self.calculate_tail_areas()

        self.prev_lv = self.l_v
        self.prev_lh = self.l_h

        while True:
            self.iteration_number += 1
            self.calculate_tail_areas()

            self.curr_lv = self.l_v
            self.curr_lh = self.l_h

            stop_condition = (abs((self.curr_lv - self.prev_lv) / self.prev_lv) < self.tolerance and abs((self.curr_lh - self.prev_lh) / self.prev_lh)) or self.iteration_number >= self.max_iterations

            if stop_condition:
                self.update_attributes()
                self.aircraft_data.save_design(design_file=self.design_file)
                break

    def update_attributes(self):
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] = self.S_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S'] = self.S_v
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h'] = self.l_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v'] = self.l_v
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['mac'] = self.mac_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['mac'] = self.mac_v
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b'] = self.b_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'] = self.b_v
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_root'] = self.chord_root_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root'] = self.chord_root_v
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['chord_tip'] = self.chord_tip_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip'] = self.chord_tip_v
        self.aircraft_data.data['inputs']['h_position'] = self.h_position
        self.aircraft_data.data['inputs']['v_position'] = self.v_position
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['relative_pos_fus'] = self.h_position
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['relative_pos_fus'] = self.v_position
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['sweep'] = self.sweep_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep'] = self.sweep_v
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['sweep_hc4'] = self.sweep_hc4
        self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['taper'] = self.taper_h
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper'] = self.taper_v

if __name__ == "__main__":
    data = Data("design1.json")

    empennage = Empennage(data)
    empennage.run_iteration()
