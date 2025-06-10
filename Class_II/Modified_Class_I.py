import sys
import os
import pandas as pd
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, AircraftType, MissionType, ISA


class ModifiedClassI:
    def __init__(self,
                 aircraft_data: Data,
                 mission_type: MissionType,
                 class_ii_OEW: float
                 ) -> None:
        self.design_file = f'design{aircraft_data.data["design_id"]}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type

        self.aircraft_type = AircraftType[self.aircraft_data.data['inputs']['aircraft_type']]

        self.design_range = self.aircraft_data.data['requirements']['design_range'] + self.aircraft_data.data['requirements']['reserve_range']/2
        self.design_payload = self.aircraft_data.data['requirements']['design_payload']
        self.design_crew = self.aircraft_data.data['requirements']['design_crew']
        self.ferry_range = self.aircraft_data.data['requirements']['ferry_range'] + self.aircraft_data.data['requirements']['reserve_range']
        self.ferry_payload = self.aircraft_data.data['requirements']['ferry_payload']
        self.ferry_crew = self.aircraft_data.data['requirements']['ferry_crew']
        self.altitude_range_WIG = self.aircraft_data.data['requirements']['altitude_range_WIG'] + self.aircraft_data.data['requirements']['reserve_range']/2
        self.altitude_range_WOG = self.aircraft_data.data['requirements']['altitude_range_WOG']
        self.altitude_payload = self.aircraft_data.data['requirements']['altitude_payload']
        self.altitude_crew = self.aircraft_data.data['requirements']['altitude_crew']

        self.cruise_speed = self.aircraft_data.data['requirements']['cruise_speed']
        self.jet_consumption = self.aircraft_data.data['inputs']['jet_consumption']
        self.prop_consumption = self.aircraft_data.data['inputs']['prop_consumption']
        self.prop_efficiency = self.aircraft_data.data['inputs']['prop_efficiency']
        self.Cd0 = self.aircraft_data.data['inputs']['Cd0']
        self.e = self.aircraft_data.data['inputs']['oswald_factor']
        self.A = self.aircraft_data.data['inputs']['aspect_ratio']
        self.tfo = self.aircraft_data.data['inputs']['tfo']
        self.k = self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['k']

        self.class_ii_OEW = class_ii_OEW

        self.fuel_fractions = {
            1: 0.970,
            2: 0.985,
            3: 0.995
        } # Raymers

    def oswald_efficiency(self, A_eff) -> float: # Howe 2000
        def f_Lambda(Z):
            return 0.005 * (1 + 1.5 * (Z - 0.6)**2)

        isa = ISA()
        M = isa.Mach(self.cruise_speed)
        taper = self.aircraft_data.data['outputs']['wing_design']['taper_ratio']
        t_c = self.aircraft_data.data['inputs']['airfoils']['wing']
        sweepc4 = self.aircraft_data.data['outputs']['wing_design']['sweep_c_4']
        N_e = self.aircraft_data.data['inputs']['n_engines']

        term1 = 1 + 0.12 * M**6
        term2 = (0.142 + f_Lambda(taper) * A_eff * (10 / t_c)**0.33) / np.cos(np.radians(sweepc4))**2
        term3 = 0.1 * (3 * N_e + 1) / (4 + A_eff)**0.8
        efficiency =  1 / (term1*(1 + term2 + term3))

        return efficiency

    def calculate_LD(self) -> float:
        self.e = self.oswald_efficiency(self.A*self.k**2)
        LD = np.sqrt(np.pi*self.A*self.e/(4*self.Cd0))
        
        self.aircraft_data.data['outputs']['general']['LD_g'] = LD
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['LD'] = LD*self.k
        return LD

    def update_fuel_fractions_prop(self) -> None:
        self.LD = self.calculate_LD()
        if self.mission_type == MissionType.DESIGN:
            range_fraction = np.exp(-self.design_range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        if self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-self.ferry_range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-self.altitude_range_WIG*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-self.altitude_range_WOG*self.prop_consumption*9.81/self.prop_efficiency * (self.LD)**-1)
            range_fraction = range_fraction_1*range_fraction_2

        self.fuel_fractions[5] = range_fraction
        
    def calculate_Mff(self) -> None:
        self.update_fuel_fractions_prop()

        self.Mff_1way = 1
        for fraction in self.fuel_fractions.values():
            self.Mff_1way *= fraction

        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.ALTITUDE:
            mfuel1 = (1-self.Mff_1way)*self.MTOW
            mass_at_destination = self.Mff_1way*self.MTOW
            mfuel2 = (1-self.Mff_1way)*(mass_at_destination-(self.aircraft_data.data['requirements']['design_payload']-10_000)*9.81)
            self.Mff = 1-(mfuel1+mfuel2)/self.MTOW
        else:
            self.Mff = self.Mff_1way

    def calculate_fuel_mission(self) -> float:
        if self.mission_type == MissionType.DESIGN:
            mission_range = self.aircraft_data.data['requirements']['design_range']
        if self.mission_type == MissionType.FERRY:
            mission_range = self.aircraft_data.data['requirements']['ferry_range']
        if self.mission_type == MissionType.ALTITUDE:
            mission_range_WOG = self.aircraft_data.data['requirements']['altitude_range_WOG']
            mission_range_WIG = self.aircraft_data.data['requirements']['altitude_range_WIG']

        self.LD = self.calculate_LD()

        if self.mission_type == MissionType.DESIGN:
            range_fraction = np.exp(-mission_range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        if self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-mission_range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-mission_range_WIG*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-mission_range_WOG*self.prop_consumption*9.81/self.prop_efficiency * (self.LD)**-1)
            range_fraction = range_fraction_1*range_fraction_2

        for fraction in [0.97, 0.985, 0.995]:
            range_fraction *= fraction

        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.ALTITUDE:
            mfuel1 = (1-range_fraction)*self.MTOW
            mass_at_destination = range_fraction*self.MTOW
            mfuel2 = (1-range_fraction)*(mass_at_destination-self.aircraft_data.data['requirements']['design_payload']*9.81)
            fuel_mission = mfuel1 + mfuel2
        else:
            fuel_mission = (1-range_fraction) * self.MTOW

        return fuel_mission


    def main(self):
        if self.mission_type == MissionType.DESIGN:
            crew_weight = self.design_crew*9.81
            payload = self.design_payload
        elif self.mission_type == MissionType.FERRY:
            crew_weight = self.ferry_crew*9.81
            payload = self.ferry_payload
        elif self.mission_type == MissionType.ALTITUDE:
            crew_weight = self.altitude_crew*9.81
            payload = self.altitude_payload
        
        # Iteratively solve for MTOW and Mff since they depend on each other
        self.Mff = 1 # stating value for iteration
        tol = 1e-6
        max_iter = 100
        self.MTOW = (payload*9.81 + crew_weight + self.class_ii_OEW) / (1 - (1-self.Mff) - self.tfo)
        for _ in range(max_iter):
            self.calculate_Mff()
            mtow_new = (payload*9.81 + crew_weight + self.class_ii_OEW) / (1 - (1-self.Mff) - self.tfo)
            if abs(mtow_new - self.MTOW) < tol:
                self.MTOW = mtow_new
                break
            self.MTOW = mtow_new

        self.OEW = self.class_ii_OEW
        self.EW = self.OEW - crew_weight
        self.total_fuel = self.MTOW * (1-self.Mff)
        self.mission_fuel = self.calculate_fuel_mission()
        self.reserve_fuel = self.total_fuel - self.mission_fuel
        self.ZFW = self.MTOW - self.total_fuel
        self.MTOM = self.MTOW/9.81
        self.fuel_max = 1.1 * self.total_fuel

        self.update_attributes()
        # self.aircraft_data.save_design(self.design_file)

    def update_attributes(self):
        self.aircraft_data.data['inputs']['oswald_factor'] = self.e
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['MTOW'] = self.MTOW
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['total_fuel'] = self.total_fuel
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['mission_fuel'] = self.mission_fuel
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['reserve_fuel'] = self.reserve_fuel
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['ZFW'] = self.ZFW
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['MTOM'] = self.MTOM
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['max_fuel'] = self.fuel_max
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['max_fuel_L'] = self.fuel_max/9.81/0.82
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['total_fuel_L'] = self.total_fuel/9.81/0.82
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['mission_fuel_L'] = self.mission_fuel/9.81/0.82
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['reserve_fuel_L'] = self.reserve_fuel/9.81/0.82



if __name__=="__main__":
    data = Data("design3.json")
    
    classI = ModifiedClassI(
        aircraft_data=data,
        mission_type=MissionType.DESIGN,
        class_ii_OEW=158814
    )
    classI.main()
    print(classI.MTOM)
