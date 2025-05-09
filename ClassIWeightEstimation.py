import os
from enum import Enum, auto
import pandas as pd
import numpy as np
from utils import Data

class AircraftType(Enum):
    JET = auto()
    PROP = auto()
    MIXED = auto()


class MissionType(Enum):
    DESIGN = auto()
    FERRY = auto()
    ALTITUDE = auto()


class ClassI:
    def __init__(self,
                 aircraft_data: Data,
                 mission_type: MissionType,
                 reference_aircraft_path: str='ReferenceAircraft.xlsx'
                 ) -> None:
        self.design_file = f'design{aircraft_data.data["design_id"]}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type

        self.aircraft_type = AircraftType[self.aircraft_data.data['inputs']['aircraft_type']]
        self.mission_type = mission_type

        self.design_range = self.aircraft_data.data['requirements']['design_range'] + self.aircraft_data.data['requirements']['reserve_range']/2
        self.design_payload = self.aircraft_data.data['requirements']['design_payload']
        self.design_crew = self.aircraft_data.data['requirements']['design_crew']
        self.ferry_range = self.aircraft_data.data['requirements']['ferry_range'] + self.aircraft_data.data['requirements']['reserve_range']
        self.ferry_payload = self.aircraft_data.data['requirements']['ferry_payload']
        self.ferry_crew = self.aircraft_data.data['requirements']['ferry_crew']
        self.altitude_range_WIG = self.aircraft_data.data['requirements']['altitude_range_WIG'] + self.aircraft_data.data['requirements']['reserve_range']
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
        self.reserve_fuel = self.aircraft_data.data['inputs']['reserve_fuel']
        self.k = self.aircraft_data.data['inputs']['k']

        self.reference_aircraft_path = reference_aircraft_path
        self.reference_aircraft = self.load_reference_aircraft()
        self.slope, self.intersection = self.linear_relation()
        self.fuel_fractions = {
            1: 0.992,
            2: 0.990,
            3: 0.996,
            4: 0.985,
            6: 0.990,
            7: 0.990
        }


    def calculate_LD(self) -> float:
        if self.aircraft_type == AircraftType.JET:
            LD = 3/4*np.sqrt(np.pi*self.A*self.e/(3*self.Cd0))
        elif self.aircraft_type == AircraftType.PROP:
            LD = np.sqrt(np.pi*self.A*self.e/(4*self.Cd0))
        elif self.aircraft_type == AircraftType.MIXED:
            LD = (3/4*np.sqrt(np.pi*self.A*self.e/(3*self.Cd0)) + np.sqrt(np.pi*self.A*self.e/(4*self.Cd0))) / 2
        else:
            raise ValueError(f"Unsupported aircraft type: {self.aircraft_type}")
        self.aircraft_data.data['outputs']['general']['LD'] = LD
        return LD

    def update_fuel_fractions_jet(self) -> None:
        self.LD = self.calculate_LD()
        if self.mission_type == MissionType.DESIGN:
            range_fraction = np.exp(-self.design_range*self.jet_consumption*9.81/self.cruise_speed * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-self.ferry_range*self.jet_consumption*9.81/self.cruise_speed * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-self.altitude_range_WIG*self.jet_consumption*9.81/self.cruise_speed * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-self.altitude_range_WOG*self.jet_consumption*9.81/self.cruise_speed * (self.LD)**-1)
            range_fraction = range_fraction_1*range_fraction_2

        self.fuel_fractions[5] = range_fraction

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

    def update_fuel_fractions_mixed(self) -> None:
        self.LD = self.calculate_LD()
        if self.mission_type == MissionType.DESIGN:
            range_fraction = np.exp(-self.design_range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-self.ferry_range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-self.altitude_range_WIG*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-self.altitude_range_WOG*self.jet_consumption*9.81/self.cruise_speed * (self.LD)**-1)
            range_fraction = range_fraction_1*range_fraction_2

        self.fuel_fractions[5] = range_fraction
        
    def calculate_Mff(self) -> None:

        if self.aircraft_type == AircraftType.JET:
            self.update_fuel_fractions_jet()
        elif self.aircraft_type == AircraftType.PROP:
            self.update_fuel_fractions_prop()
        elif self.aircraft_type == AircraftType.MIXED:
            self.update_fuel_fractions_mixed()

        self.Mff = 1
        for fraction in self.fuel_fractions.values():
            self.Mff *= fraction

        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.ALTITUDE:
            self.Mff **= 2


    def linear_relation(self):
        x, y = self.reference_aircraft['MTOW'], self.reference_aircraft['EW']
        x *= 9.81
        y *= 9.81
        self.slope, self.intersection = np.polyfit(x, y, 1)
        return self.slope, self.intersection
    
    def load_reference_aircraft(self) -> pd.DataFrame:
        data = pd.read_excel(self.reference_aircraft_path)
        data.dropna(inplace=True)
        return data

        
    def main(self):
        self.calculate_Mff()
        if self.mission_type == MissionType.DESIGN:
            self.MTOW = (self.design_payload*9.81 + self.design_crew*9.81 + self.intersection) / (1 - self.slope - (1-self.Mff) - (1-self.Mff)*self.reserve_fuel - self.tfo)
            self.fuel_used = self.MTOW * (1-self.Mff)
            self.fuel_res = self.MTOW * (1-self.Mff) * self.reserve_fuel
            self.fuel = self.fuel_used + self.fuel_res
            self.OEW = self.slope * self.MTOW + self.intersection
            self.EW = self.OEW - self.design_crew
            self.ZFW = self.MTOW - self.fuel
            self.MTOM = self.MTOW/9.81
        elif self.mission_type == MissionType.FERRY:
            self.MTOW = (self.ferry_payload*9.81 + self.design_crew*9.81 + self.intersection) / (1 - self.slope - (1-self.Mff) - (1-self.Mff)*self.reserve_fuel - self.tfo)
            self.fuel_used = self.MTOW * (1-self.Mff)
            self.fuel_res = self.MTOW * (1-self.Mff) * self.reserve_fuel
            self.fuel = self.fuel_used + self.fuel_res
            self.OEW = self.slope * self.MTOW + self.intersection
            self.EW = self.OEW - self.ferry_crew
            self.ZFW = self.MTOW - self.fuel
            self.MTOM = self.MTOW/9.81
        elif self.mission_type == MissionType.ALTITUDE:
            self.MTOW = (self.altitude_payload*9.81 + self.design_crew*9.81 + self.intersection) / (1 - self.slope - (1-self.Mff) - (1-self.Mff)*self.reserve_fuel - self.tfo)
            self.fuel_used = self.MTOW * (1-self.Mff)
            self.fuel_res = self.MTOW * (1-self.Mff) * self.reserve_fuel
            self.fuel = self.fuel_used + self.fuel_res
            self.OEW = self.slope * self.MTOW + self.intersection
            self.EW = self.OEW - self.altitude_crew
            self.ZFW = self.MTOW - self.fuel
            self.MTOM = self.MTOW/9.81
            
        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)

    def update_attributes(self):
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['MTOW'] = self.MTOW
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['fuel_used'] = self.fuel_used
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['fuel_res'] = self.fuel_res
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['fuel'] = self.fuel
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['OEW'] = self.OEW
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['EW'] = self.EW
        self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['ZFW'] = self.ZFW



if __name__=="__main__":
    data = Data("design1.json")
    
    classI = ClassI(
        aircraft_data=data,
        mission_type=MissionType.DESIGN
    )
    classI.main()
    print(classI.MTOM)
