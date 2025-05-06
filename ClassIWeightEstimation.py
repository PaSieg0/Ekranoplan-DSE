from enum import Enum, auto
import pandas as pd
import numpy as np

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
                 aircraft_type: AircraftType,
                 mission_type: MissionType,
                 cruise_speed: float,
                 jet_consumption: float,
                 prop_consumption: float,
                 prop_efficiency: float,
                 Cd0: float,
                 e: float,
                 A: float,
                 tfo: float,
                 reserve_fuel: float,
                 k: float,
                 reference_aircraft_path: str='ReferenceAircraft.xlsx'
                 ) -> None:
        self.aircraft_type = aircraft_type
        self.mission_type = mission_type

        if mission_type == MissionType.DESIGN:
            self.range = (2800+50)*1.852*1000
            self.payload = 90000
            self.crew = 5*85
        elif mission_type == MissionType.FERRY:
            self.range = (6500+100)*1.852*1000
            self.payload = 0
            self.crew = 5*85
        elif mission_type == MissionType.ALTITUDE:
            self.range_WIG = (550+100)*1.852*1000
            self.range_WOG = 250*1.852*1000
            self.payload = 90000
            self.crew = 5*85

        self.reference_aircraft_path = reference_aircraft_path
        self.reference_aircraft = self.load_reference_aircraft()
        self.slope, self.intersection = self.linear_relation()
        self.cruise_speed = cruise_speed
        self.jet_consumption = jet_consumption
        self.prop_consumption = prop_consumption
        self.prop_efficiency = prop_efficiency
        self.Cd0 = Cd0
        self.e = e
        self.A = A
        self.tfo = tfo
        self.reserve_fuel = reserve_fuel
        self.k = k
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
        return LD

    def update_fuel_fractions_jet(self) -> None:
        self.LD = self.calculate_LD()
        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-self.range*self.jet_consumption*9.81/self.cruise_speed * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-self.range_WIG*self.jet_consumption*9.81/self.cruise_speed * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-self.range_WOG*self.jet_consumption*9.81/self.cruise_speed * (self.LD)**-1)
            range_fraction = range_fraction_1*range_fraction_2

        self.fuel_fractions[5] = range_fraction

    def update_fuel_fractions_prop(self) -> None:
        self.LD = self.calculate_LD()
        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-self.range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-self.range_WIG*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-self.range_WOG*self.prop_consumption*9.81/self.prop_efficiency * (self.LD)**-1)
            range_fraction = range_fraction_1*range_fraction_2

        self.fuel_fractions[5] = range_fraction

    def update_fuel_fractions_mixed(self) -> None:
        self.LD = self.calculate_LD()
        if self.mission_type == MissionType.DESIGN or self.mission_type == MissionType.FERRY:
            range_fraction = np.exp(-self.range*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
        elif self.mission_type == MissionType.ALTITUDE:
            range_fraction_1 = np.exp(-self.range_WIG*self.prop_consumption*9.81/self.prop_efficiency * (self.k*self.LD)**-1)
            range_fraction_2 = np.exp(-self.range_WOG*self.jet_consumption*9.81/self.cruise_speed * (self.LD)**-1)
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
        print(f"Fuel fractions: {self.fuel_fractions}")
        self.MTOW = (self.payload*9.81 + self.crew*9.81 + self.intersection) / (1 - self.slope - (1-self.Mff) - (1-self.Mff)*self.reserve_fuel - self.tfo)
        self.fuel_used = self.MTOW * (1-self.Mff)
        self.fuel_res = self.MTOW * (1-self.Mff) * self.reserve_fuel
        self.fuel = self.fuel_used + self.fuel_res
        self.OEW = self.slope * self.MTOW + self.intersection
        self.ZFW = self.MTOW - self.fuel
        self.EW = self.OEW - self.crew
        self.MTOM = self.MTOW/9.81

if __name__=="__main__":


    # CHANGE THESE VALUES
    # MIGHT WANT TO MAKE A .json FILE TO SAVE ALL PARAMETERS AND LOAD THEM
    aircraft_type = AircraftType.MIXED
    mission_type = MissionType.FERRY
    cruise_speed = 225*0.51444
    jet_consumption = 19e-6
    prop_consumption = 90e-9
    prop_efficiency = 0.82
    Cd0 = 0.02
    e = 0.85
    A = 6
    tfo = 0.001
    reserve_fuel = 0
    k = 1

    class_i = ClassI(
        aircraft_type=aircraft_type,
        mission_type=mission_type,
        cruise_speed=cruise_speed,
        jet_consumption=jet_consumption,
        prop_consumption=prop_consumption,
        prop_efficiency=prop_efficiency,
        Cd0=Cd0,
        e=e,
        A=A,
        tfo=tfo,
        reserve_fuel=reserve_fuel,
        k=k
    )
    class_i.main()
    print(f"{class_i.aircraft_type.name}: {class_i.mission_type.name}")
    print(f"MTOM: {class_i.MTOM:=,.2f} kg.")
 
    

    for type_i in AircraftType:
        max_MTOM = float('inf')
        for mission_i in MissionType:
            class_i = ClassI(
                aircraft_type=type_i,
                mission_type=mission_i,
                cruise_speed=cruise_speed,
                jet_consumption=jet_consumption,
                prop_consumption=prop_consumption,
                prop_efficiency=prop_efficiency,
                Cd0=Cd0,
                e=e,
                A=A,
                tfo=tfo,
                reserve_fuel=reserve_fuel,
                k=k
            )
            class_i.main()
            print(f"{class_i.aircraft_type.name}: {class_i.mission_type.name}")
            print(f"MOTM: {class_i.MTOM:=,.2f} kg.")
        else:
            print("\n")
