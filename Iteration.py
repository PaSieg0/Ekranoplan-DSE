import numpy as np
from scipy.optimize import fsolve
from WingLoading import main
from ClassIWeightEstimation import ClassI, MissionType, AircraftType
import matplotlib.pyplot as plt
from ISA_Class import ISA
from utils import Data


def solve_hb(target_A_A):
    h_b = np.arange(0, 1, 0.00001)
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
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type

        self.tolerance = 0.0001
        self.max_iterations = 10
        self.iteration = 0

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
        self.b = np.sqrt(self.S * self.class_i.A)
        self.h_b = self.aircraft_data.data['cruise_altitude'] / self.b
        self.A_ratio = Ainf_Ah(self.h_b)
        self.new_k = np.sqrt(1 / self.A_ratio)


        
    def run_iteration(self) -> list[float]:
        self.get_initial_conditions()
        while True:
            self.iteration += 1
            self.class_i.k = self.new_k

            self.class_i.main()
            self.curr_MTOM = self.class_i.MTOM
            self.WP, self.TW, self.WS = main(aircraft_data=self.aircraft_data,
                                            mission_type=self.mission_type,
                                             PLOT_OUTPUT=False)
            stop_condition = abs((self.curr_MTOM - self.prev_MTOM) / self.prev_MTOM) < self.tolerance or self.iteration >= self.max_iterations
            if stop_condition:
                self.update_attributes()
                self.aircraft_data.save_design('design1.json')
                break

            self.prev_MTOM = self.curr_MTOM
            self.MTOM_history.append(self.curr_MTOM)
            self.S = self.class_i.MTOW / self.WS
            self.b = np.sqrt(self.S * self.class_i.A)
            self.h_b = self.aircraft_data.data['cruise_altitude'] / self.b
            self.A_ratio = Ainf_Ah(self.h_b)
            self.new_k = np.sqrt(1 / self.A_ratio)
            self.aircraft_data.data['k'] = self.new_k
            

        
            
    def update_attributes(self):
        mission_type = self.mission_type.name.lower()
        self.aircraft_data.data[mission_type]['MTOM'] = self.class_i.MTOM
        self.aircraft_data.data[mission_type]['MTOW'] = self.class_i.MTOW
        self.aircraft_data.data[mission_type]['OEW'] = self.class_i.OEW
        self.aircraft_data.data[mission_type]['ZFW'] = self.class_i.ZFW
        self.aircraft_data.data[mission_type]['EW'] = self.class_i.EW
        self.aircraft_data.data[mission_type]['Fuel'] = self.class_i.fuel
        self.aircraft_data.data[mission_type]['Fuel_used'] = self.class_i.fuel_used
        self.aircraft_data.data[mission_type]['Fuel_reserve'] = self.class_i.fuel_res
        self.aircraft_data.data[mission_type]['S'] = self.S
        self.aircraft_data.data[mission_type]['aspect_ratio'] = self.class_i.A
        self.aircraft_data.data[mission_type]['b'] = self.b
        self.aircraft_data.data[mission_type]['MAC'] = self.S / self.b
        self.aircraft_data.data[mission_type]['cruise_altitude'] = self.aircraft_data.data['cruise_altitude']
        self.aircraft_data.data[mission_type]['h_b'] = self.h_b
        self.aircraft_data.data[mission_type]['k'] = self.new_k
        self.aircraft_data.data[mission_type]['WP'] = self.WP
        self.aircraft_data.data[mission_type]['TW'] = self.TW
        self.aircraft_data.data[mission_type]['WS'] = self.WS

        if self.mission_type == MissionType.DESIGN:
            self.aircraft_data.data[mission_type]['fuel_economy'] = self.class_i.fuel_used / 9.81 * 0.82 / 90 / (self.class_i.design_range / 1000)
            self.aircraft_data.data[mission_type]['design_payload'] = self.class_i.design_payload
        elif self.mission_type == MissionType.FERRY:
            self.aircraft_data.data[mission_type]['fuel_economy'] = self.class_i.fuel_used / 9.81 * 0.82 / 90 / (self.class_i.ferry_range / 1000)
            self.aircraft_data.data[mission_type]['ferry_payload'] = self.class_i.ferry_payload
        elif self.mission_type == MissionType.ALTITUDE:
            self.aircraft_data.data[mission_type]['fuel_economy'] = self.class_i.fuel_used / 9.81 * 0.82 / 90 / ((self.class_i.altitude_range_WIG+self.class_i.altitude_range_WOG) / 1000)
            self.aircraft_data.data[mission_type]['altitude_payload'] = self.class_i.altitude_payload
        

if __name__=='__main__':
    iteration = AircraftIteration(
        aircraft_data=Data('design1.json'),
        mission_type=MissionType.FERRY
    )

    iteration.run_iteration()