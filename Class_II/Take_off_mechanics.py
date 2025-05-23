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

class Take_off_mechanics:
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

        self.CLmax_takeoff = self.aircraft_data.data['outputs']['wing_design']['CLmax_takeoff']
        self.CL_hydro = self.aircraft_data.data['outputs']['wing_design']['CL_hydro']
        self.MTOW = self.aircraft_data.data['outputs']['wing_design']['MTOW']
        self.Cd = self.aircraft_data.data['outputs']['wing_design']['Cd']
        self.Cd_water = self.aircraft_data.data['outputs']['wing_design']['Cd_water']

    def Take_off(self):
