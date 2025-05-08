import numpy as np
from enum import Enum

class WingType(Enum):
    HIGHWING = "highwing"
    LOWWING = "lowwing"

class WingPlanform:
    def __init__(self, 
                 wing_area: float,
                 aspect_ratio: float,
                 wing_type: WingType) -> None:

        self.fuselage_length = 52.6411 # meters
        self.x_c = 0 #Where along the wing we want to look, so in this case 0 is the leading edge of the wing
        self.taper_ratio = 0.4
        self.x_c_OEW_cg = 0.2 # x/c OEW CG position
        self.x_c_wing_cg = 0.4 # x/c wing CG position
        self.fuse_x_cg = 0.4  # normalized CG position
        self.mass_fraction_wing = 0.08 # mass fraction of the wing
        self.mass_fraction_fuse = 0.08 # mass fraction of the fuselage
        self.sweep_c_4 = 0  # degrees

        if wing_type == WingType.HIGHWING:
            self.dihedral = 1  # degrees
        elif wing_type == WingType.LOWWING:
            self.dihedral = 5  # degrees
        else:
            raise ValueError("Invalid wing type")

        self.wing_area = wing_area
        self.aspect_ratio = aspect_ratio

    def x_LE(self):
        b = np.sqrt(self.aspect_ratio * self.wing_area)
        sweep_x_c = np.rad2deg(np.arctan(
            np.tan(np.deg2rad(self.sweep_c_4)) -
            ((4 / self.aspect_ratio) * (self.x_c - 0.25) *
             ((1 - self.taper_ratio) / (1 + self.taper_ratio)))
        ))

        chord_root = 2 * self.wing_area / (b * (1 + self.taper_ratio))
        chord_tip = self.taper_ratio * chord_root

        # Correct MAC formula
        MAC = (2 / 3) * chord_root * ((1 + self.taper_ratio + self.taper_ratio**2) / (1 + self.taper_ratio))
        y_MAC = (b / 6) * (1 + 2 * self.taper_ratio) / (1 + self.taper_ratio)

        X_LEMAC = (self.fuse_x_cg * self.fuselage_length) + \
                  MAC * ((self.x_c_wing_cg * self.mass_fraction_wing / self.mass_fraction_fuse) -
                         self.x_c_OEW_cg * (1 + self.mass_fraction_wing / self.mass_fraction_fuse))

        X_LE = X_LEMAC - y_MAC * np.tan(np.deg2rad(sweep_x_c))
        return X_LE, X_LEMAC, y_MAC, chord_root, chord_tip, sweep_x_c

#TEST
# if __name__ == "__main__":
#     wing = WingPlanform(wing_area=900, aspect_ratio=10, wing_type=WingType.HIGHWING)
#     x_le_result = wing.x_LE()
#     print(f"x_LE: {x_le_result[0]}")
#     print(f"x_LEMAC: {x_le_result[1]}")
#     print(f"y_MAC: {x_le_result[2]}")
#     print(f"chord_root: {x_le_result[3]}")
#     print(f"chord_tip: {x_le_result[4]}")
#     print(f"sweep_x_c: {x_le_result[5]}")
