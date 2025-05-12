import os
import pandas as pd
import numpy as np
from utils import Data

class Empennage:
    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data

    def calculate_tail_areas(self, Xcg_aft):
        self.S_h = self.aircraft_data.data['inputs']['V_h'] * self.aircraft_data.data['outputs']['max']['S'] * self.aircraft_data.data['outputs']['max']['MAC'] / (self.aircraft_data.data['outputs']['general']['l_fuselage'] - (self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] + Xcg_aft*self.aircraft_data.data['outputs']['max']['MAC']))
        self.S_v = self.aircraft_data.data['inputs']['V_v'] * self.aircraft_data.data['outputs']['max']['S'] * self.aircraft_data.data['outputs']['max']['b'] / (self.aircraft_data.data['outputs']['general']['l_fuselage'] - (self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] + Xcg_aft*self.aircraft_data.data['outputs']['max']['MAC']))

        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)

    def update_attributes(self):
        self.aircraft_data.data['outputs']['empennage_design']['S_h'] = self.S_h
        self.aircraft_data.data['outputs']['empennage_design']['S_v'] = self.S_v

if __name__ == "__main__":
    data = Data("design4.json")
    emp = Empennage(data)
    Xcg_aft = 0.30440835377790804
    emp.calculate_tail_areas(Xcg_aft=Xcg_aft)
