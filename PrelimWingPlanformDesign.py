import numpy as np
from enum import Enum, auto
from utils import Data
from ISA_Class import ISA

class WingType(Enum):
    HIGH = auto()
    LOW = auto()

class WingPlanform:
    def __init__(self, 
                 aircraft_data: Data
                 ) -> None:
        
        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.wing_type = WingType[aircraft_data.data['wing_type']]
        self.wing_area = aircraft_data.data['max']['S']
        self.aspect_ratio = aircraft_data.data['aspect_ratio']

        self.fuselage_length = 52.6411 # meters
        self.x_c = 0 #Where along the wing we want to look, so in this case 0 is the leading edge of the wing
        self.taper_ratio = 0.4
        self.x_c_OEW_cg = 0.2 # x/c OEW CG position
        self.x_c_wing_cg = 0.4 # x/c wing CG position
        self.fuse_x_cg = 0.4  # normalized CG position
        self.mass_fraction_wing = 0.08 # mass fraction of the wing
        self.mass_fraction_fuse = 0.08 # mass fraction of the fuselage


    def calculate(self):
        self.mach = ISA(self.aircraft_data.data['cruise_altitude']).Mach(self.aircraft_data.data['cruise_speed'])
        if self.mach <= 0.7:
            self.sweep_c_4 = 0  # degrees
        else: 
            self.sweep_c_4 = 0.75 * 0.935/(self.mach+0.03)

        self.taper = 0.2*(2-self.sweep_c_4*np.pi/180)

        if self.wing_type == WingType.HIGH:
            self.dihedral = 1  # degrees
        elif self.wing_type == WingType.LOW:
            self.dihedral = 5  # degrees
        else:
            raise ValueError("Invalid wing type")
        
        self.sweep_c_4
        
        b = self.aircraft_data.data['max']['b']
        self.sweep_x_c = np.rad2deg(np.arctan(
            np.tan(np.deg2rad(self.sweep_c_4)) -
            ((4 / self.aspect_ratio) * (self.x_c - 0.25) *
             ((1 - self.taper_ratio) / (1 + self.taper_ratio)))
        ))

        self.chord_root = 2 * self.wing_area / (b * (1 + self.taper_ratio))
        self.chord_tip = self.taper_ratio * self.chord_root

        # Correct MAC formula
        MAC = (2 / 3) * self.chord_root * ((1 + self.taper_ratio + self.taper_ratio**2) / (1 + self.taper_ratio))
        self.y_MAC = (b / 6) * (1 + 2 * self.taper_ratio) / (1 + self.taper_ratio)

        self.X_LEMAC = (self.fuse_x_cg * self.fuselage_length) + \
                  MAC * ((self.x_c_wing_cg * self.mass_fraction_wing / self.mass_fraction_fuse) -
                         self.x_c_OEW_cg * (1 + self.mass_fraction_wing / self.mass_fraction_fuse))

        self.X_LE = self.X_LEMAC - self.y_MAC * np.tan(np.deg2rad(self.sweep_x_c))
        self.update_design_data()
        self.aircraft_data.save_design(self.design_file)


    def update_design_data(self):
        self.aircraft_data.data['wing_design']['taper_ratio'] = self.taper_ratio
        self.aircraft_data.data['wing_design']['sweep_c_4'] = self.sweep_c_4
        self.aircraft_data.data['wing_design']['dihedral'] = self.dihedral
        self.aircraft_data.data['wing_design']['sweep_x_c'] = self.sweep_x_c
        self.aircraft_data.data['wing_design']['chord_root'] = self.chord_root
        self.aircraft_data.data['wing_design']['chord_tip'] = self.chord_tip
        self.aircraft_data.data['wing_design']['y_MAC'] = self.y_MAC
        self.aircraft_data.data['wing_design']['X_LEMAC'] = self.X_LEMAC
        self.aircraft_data.data['wing_design']['X_LE'] = self.X_LE
        


if __name__ == "__main__":
    data = Data('design1.json')
    wing = WingPlanform(data)
    wing.calculate()
    print(f"x_LE: {wing.X_LE}")
    print(f"x_LEMAC: {wing.X_LEMAC}")
    print(f"y_MAC: {wing.y_MAC}")
    print(f"chord_root: {wing.chord_root}")
    print(f"chord_tip: {wing.chord_tip}")
    print(f"sweep_x_c: {wing.sweep_x_c}")
