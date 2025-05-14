import os
import pandas as pd
import numpy as np
from utils import Data
from enum import Enum, auto

class EmpType(Enum):
    CRUCIFORM = auto()
    T_TAIL = auto()
    V_TAIL = auto()
    CONVENTIONAL = auto()
    H_TAIL = auto()
    NONE = auto()
class Empennage:
    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data
        self.tail_type = EmpType[aircraft_data.data['inputs']['wing_type']]

    def calculate_tail_areas(self):
        if self.tail_type == EmpType.NONE:
            return
        
        self.l_h = (0.9*self.aircraft_data.data['outputs']['general']['l_fuselage'] - self.aircraft_data.data['outputs']['cg_range']['most_aft_cg'])
        self.l_v = (0.9*self.aircraft_data.data['outputs']['general']['l_fuselage'] - self.aircraft_data.data['outputs']['cg_range']['most_aft_cg'])
        self.S_h = self.aircraft_data.data['inputs']['V_h'] * self.aircraft_data.data['outputs']['max']['S'] * self.aircraft_data.data['outputs']['max']['MAC'] / self.l_h
        self.S_v = self.aircraft_data.data['inputs']['V_v'] * self.aircraft_data.data['outputs']['max']['S'] * self.aircraft_data.data['outputs']['max']['b'] / self.l_v

        self.b_h = self.S_h / self.aircraft_data.data['inputs']['aspect_h']
        self.b_v = self.S_v / self.aircraft_data.data['inputs']['aspect_v']

        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)

    def update_attributes(self):
        self.aircraft_data.data['outputs']['empennage_design']['S_h'] = self.S_h
        self.aircraft_data.data['outputs']['empennage_design']['S_v'] = self.S_v
        self.aircraft_data.data['outputs']['empennage_design']['l_h'] = self.l_h
        self.aircraft_data.data['outputs']['empennage_design']['l_v'] = self.l_v

if __name__ == "__main__":
    data = Data("design4.json")
    
