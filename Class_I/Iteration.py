import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
from scipy.optimize import fsolve
from WingLoading import main, WingLoading
from ClassIWeightEstimation import ClassI
import matplotlib.pyplot as plt
from utils import Data, ISA, MissionType, AircraftType, WingType

def solve_hb(target_A_A):
    h_b = np.arange(0, 2, 0.00001)
    y = 1 - np.exp(-4.74*h_b**0.814) - h_b**2*np.exp(-3.88*h_b**0.758)

    for i, y_val in enumerate(y):
        diff = abs((1/target_A_A - y_val))
        if diff <= 0.01:
            return h_b[i]
        
    raise ValueError

def Ainf_Ah(h_b):
    return 1 - np.exp(-4.74*h_b**0.814) - h_b**2*np.exp(-3.88*h_b**0.758)

class AircraftIteration:
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f'design{self.design_number}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type

        self.tolerance = 0.00015
        self.max_iterations = 20
        self.iteration = 0

        self.V_lof = 1.05 * self.aircraft_data.data['requirements']['stall_speed_takeoff']

        self.class_i = ClassI(
            aircraft_data=self.aircraft_data,
            mission_type=self.mission_type,
        )

    def get_initial_conditions(self):
        self.class_i.main()
        self.prev_MTOM = self.class_i.MTOM
        self.MTOM_history = [self.prev_MTOM]
        self.WP, self.TW, self.WS = main(aircraft_data=self.aircraft_data,
                                         mission_type=self.mission_type,
                                         PLOT_OUTPUT=False)
        self.S = self.class_i.MTOW / self.WS
        
        if self.design_number == 4:
            self.b = np.sqrt(self.S/2 * self.class_i.A)
        else:
            self.b = np.sqrt(self.S * self.class_i.A)
        self.h_b = self.aircraft_data.data['inputs']['cruise_altitude'] / self.b
        self.A_ratio = Ainf_Ah(self.h_b)
        self.new_k = np.sqrt(1 / self.A_ratio)
        self.new_Cd0 = self.aircraft_data.data['inputs']['Cd0']
        self.d_fuselage = self.aircraft_data.data['outputs']['general']['d_fuselage']
        self.l_fuselage = self.aircraft_data.data['outputs']['general']['l_fuselage']
        self.n_fuselages = self.aircraft_data.data['inputs']['n_fuselages']
        self.wing_type = WingType[self.aircraft_data.data['inputs']['wing_type']]

        if self.wing_type == WingType.HIGH:
            self.h_D = (self.aircraft_data.data['inputs']['cruise_altitude'] - self.d_fuselage) / self.d_fuselage / self.n_fuselages
        elif self.wing_type == WingType.LOW:
            self.h_D = (self.aircraft_data.data['inputs']['cruise_altitude']) / self.d_fuselage / self.n_fuselages
        self.A_ratio_fus = Ainf_Ah(self.h_D)
        self.k_fus = np.sqrt(1 / self.A_ratio_fus)
        self.new_k = self.new_k * self.k_fus
        self.k_tail = 1

    def run_iteration(self) -> list[float]:
        self.get_initial_conditions()

        while True:
            self.iteration += 1
            self.class_i.k = self.new_k
            self.class_i.Cd0 = self.new_Cd0

            # print(f"k: {self.new_k}")
            # print(f"Iteration: {self.iteration}")
            # print(f"MTOM: {self.class_i.MTOM}")
            # print(f"MTOW: {self.class_i.MTOW}")
            # print(f"S: {self.S}")

            self.class_i.main()
            self.curr_MTOM = self.class_i.MTOM
            self.WP, self.TW, self.WS = main(aircraft_data=self.aircraft_data,
                                            mission_type=self.mission_type,
                                             PLOT_OUTPUT=False)
            stop_condition = abs((self.curr_MTOM - self.prev_MTOM) / self.prev_MTOM) < self.tolerance or self.iteration >= self.max_iterations
            if stop_condition:
                self.update_attributes()
                self.aircraft_data.save_design(self.design_file)
                break

            self.prev_MTOM = self.curr_MTOM
            self.MTOM_history.append(self.curr_MTOM)
            self.S = self.class_i.MTOW / self.WS
            self.b = np.sqrt(self.S/self.aircraft_data.data['inputs']['n_wings'] * self.class_i.A)
            self.h_b = self.aircraft_data.data['inputs']['cruise_altitude'] / self.b
            self.A_ratio = Ainf_Ah(self.h_b)
            if self.design_number != 4:
                self.h_b_tail = (self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height'] + self.aircraft_data.data['inputs']['cruise_altitude']) / self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
                self.A_ratio_tail = Ainf_Ah(self.h_b_tail)
                self.k_tail = np.sqrt(1 / self.A_ratio_tail)
            else:
                self.k_tail = 1
            self.new_k = np.sqrt(1 / self.A_ratio)*self.k_fus*self.k_tail
            self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['k'] = self.new_k
            self.new_Cd0 = self.aircraft_data.data['inputs']['Cd0']


            self.max_power = max(self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['P'], self.aircraft_data.data['outputs']['general']['take_off_power'])

    def update_attributes(self):
        mission_type = self.mission_type.name.lower()
        self.aircraft_data.data['outputs'][mission_type]['MTOM'] = self.class_i.MTOM
        self.aircraft_data.data['outputs'][mission_type]['MTOW'] = self.class_i.MTOW
        self.aircraft_data.data['outputs'][mission_type]['OEW'] = self.class_i.OEW
        self.aircraft_data.data['outputs'][mission_type]['ZFW'] = self.class_i.ZFW
        self.aircraft_data.data['outputs'][mission_type]['EW'] = self.class_i.EW

        self.aircraft_data.data['outputs'][mission_type]['total_fuel'] = self.class_i.total_fuel
        self.aircraft_data.data['outputs'][mission_type]['S'] = self.S
        self.aircraft_data.data['outputs'][mission_type]['b'] = self.b
        self.aircraft_data.data['outputs'][mission_type]['MAC'] = self.S / self.b
        self.aircraft_data.data['outputs'][mission_type]['h_b'] = self.h_b
        self.aircraft_data.data['outputs'][mission_type]['k'] = self.new_k
        self.aircraft_data.data['outputs'][mission_type]['WP'] = self.WP
        self.aircraft_data.data['outputs'][mission_type]['TW'] = self.TW
        self.aircraft_data.data['outputs'][mission_type]['WS'] = self.WS
        self.aircraft_data.data['outputs'][mission_type]['WS'] = self.WS
        self.aircraft_data.data['outputs'][mission_type]['Mff'] = self.class_i.Mff
        self.aircraft_data.data['outputs']['general']['V_lof'] = self.aircraft_data.data['requirements']['stall_speed_takeoff'] * 1.05
        if self.WP:
            self.aircraft_data.data['outputs'][mission_type]['P'] = self.class_i.MTOW / self.WP
        else: 
            self.aircraft_data.data['outputs'][mission_type]['P'] = None
        if self.TW:
            self.aircraft_data.data['outputs'][mission_type]['T'] = self.class_i.MTOW * self.TW
        else:
            self.aircraft_data.data['outputs'][mission_type]['T'] = None

        n_engines_flight = np.ceil(self.aircraft_data.data['outputs'][mission_type]['P']/ self.aircraft_data.data['inputs']['engine_power']) + 1
        n_engines_takeoff = np.ceil(self.aircraft_data.data['outputs']['general']['take_off_power']/ self.aircraft_data.data['inputs']['engine_power'])
        self.aircraft_data.data['inputs']['n_engines'] = max(n_engines_flight, n_engines_takeoff)
        if self.mission_type == MissionType.DESIGN:
            self.aircraft_data.data['outputs'][mission_type]['fuel_economy'] = self.class_i.mission_fuel / 9.81 / 0.82 / (self.aircraft_data.data['requirements']['design_payload']/1000) / (2*self.class_i.design_range / 1000)
        elif self.mission_type == MissionType.ALTITUDE:
            self.aircraft_data.data['outputs'][mission_type]['fuel_economy'] = self.class_i.mission_fuel / 9.81 / 0.82 / (self.aircraft_data.data['requirements']['altitude_payload']/1000) / (2*(self.class_i.altitude_range_WIG+self.class_i.altitude_range_WOG) / 1000)
        

        self.aircraft_data.data['outputs']['max']['MTOM'] = max(self.aircraft_data.data['outputs']['design']['MTOM'], self.aircraft_data.data['outputs']['ferry']['MTOM'], self.aircraft_data.data['outputs']['altitude']['MTOM'])
        self.aircraft_data.data['outputs']['max']['MTOW'] = self.aircraft_data.data['outputs']['max']['MTOM']*9.81
        self.aircraft_data.data['outputs']['max']['S'] = max(self.aircraft_data.data['outputs']['design']['S'], self.aircraft_data.data['outputs']['ferry']['S'], self.aircraft_data.data['outputs']['altitude']['S'])
        self.aircraft_data.data['outputs']['max']['b'] = max(self.aircraft_data.data['outputs']['design']['b'], self.aircraft_data.data['outputs']['ferry']['b'], self.aircraft_data.data['outputs']['altitude']['b'])
        self.aircraft_data.data['outputs']['max']['MAC'] = max(self.aircraft_data.data['outputs']['design']['MAC'], self.aircraft_data.data['outputs']['ferry']['MAC'], self.aircraft_data.data['outputs']['altitude']['MAC'])
        self.aircraft_data.data['outputs']['max']['fuel_economy'] = min(self.aircraft_data.data['outputs']['design']['fuel_economy'], self.aircraft_data.data['outputs']['altitude']['fuel_economy'])
        self.aircraft_data.data['outputs']['max']['Mff'] = min(self.aircraft_data.data['outputs']['design']['Mff'], self.aircraft_data.data['outputs']['ferry']['Mff'], self.aircraft_data.data['outputs']['altitude']['Mff'])
        self.aircraft_data.data['outputs']['max']['OEW'] = max(self.aircraft_data.data['outputs']['design']['OEW'], self.aircraft_data.data['outputs']['ferry']['OEW'], self.aircraft_data.data['outputs']['altitude']['OEW'])
        self.aircraft_data.data['outputs']['max']['total_fuel'] = max(self.aircraft_data.data['outputs']['design']['total_fuel'], self.aircraft_data.data['outputs']['ferry']['total_fuel'], self.aircraft_data.data['outputs']['altitude']['total_fuel'])
        self.aircraft_data.data['outputs']['max']['mission_fuel'] = max(self.aircraft_data.data['outputs']['design']['mission_fuel'], self.aircraft_data.data['outputs']['ferry']['mission_fuel'], self.aircraft_data.data['outputs']['altitude']['mission_fuel'])
        self.aircraft_data.data['outputs']['max']['reserve_fuel'] = max(self.aircraft_data.data['outputs']['design']['reserve_fuel'], self.aircraft_data.data['outputs']['ferry']['reserve_fuel'], self.aircraft_data.data['outputs']['altitude']['reserve_fuel'])
        self.aircraft_data.data['outputs']['max']['max_fuel'] = 1.1 * self.aircraft_data.data['outputs']['max']['total_fuel']
        self.aircraft_data.data['outputs']['max']['LD'] = max(self.aircraft_data.data['outputs']['design']['LD'], self.aircraft_data.data['outputs']['ferry']['LD'], self.aircraft_data.data['outputs']['altitude']['LD'])

if __name__=='__main__':
    iteration = AircraftIteration(
        aircraft_data=Data('design1.json'),
        mission_type=MissionType.DESIGN
    )

    iteration.run_iteration()