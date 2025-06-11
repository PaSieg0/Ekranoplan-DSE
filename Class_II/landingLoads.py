import sys
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, EvaluateType
from AerodynamicForces import AerodynamicForces

class LandingLoads:
    def __init__(self, data: Data, evaluate_type: EvaluateType):
        self.data = data
        self.evaluate_type = evaluate_type
        self.aero_forces = AerodynamicForces(data, evaluate_type)
        self.center_of_pressure_from_AC = 0.25 # 25% of the MAC from the leading edge
        self.x_LEMAC = self.data.data["outputs"]["wing_design"]["x_LEMAC"]
        self.x_aft_cg = self.data.data["outputs"]["cg_range"]["most_aft_cg"]
        self.x_fore_cg = self.data.data["outputs"]["cg_range"]["most_foreward_cg"]
        self.wing_MAC = self.data.data["outputs"]["wing_design"]["MAC"]

        self.x_center_of_pressure = self.x_LEMAC + self.center_of_pressure_from_AC * self.wing_MAC
        
    def calculate_radius_of_gyration(self):
        # Calculate the radius of gyration based on the CG range
        
    def calculate_landing_load(self):
