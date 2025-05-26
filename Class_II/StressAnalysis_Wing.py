import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import matplotlib.pyplot as plt
from utils import Data
from AerodynamicForces import AerodynamicForces
from WingStructure import WingStructure


class StressAnalysisWing(AerodynamicForces, WingStructure):

    def __init__(self, aircraft_data: Data, airfoil_aerodynamics: Data, airfoil_data: Data):
        AerodynamicForces.__init__(self, aircraft_data, airfoil_aerodynamics)
        WingStructure.__init__(self, aircraft_data, airfoil_data)

    
        self.lift_function = self.get_lift_function()
        self.drag_function = self.get_drag_function()
        self.moment_function = self.get_moment_function()
        self.weight_distribution = self.wing_weight_dist()


    def calculate_shear_force(self,y):
        return 

    def calculate_shear_stress(self):
        return 
    
    def calculate_bending_moment(self):
        return 
    
    def calculate_bending_moment_stress(self):
        return 

    def calculate_torque(self):
        return 
    
    def calculate_torsion(self):
        return 
    
    def calculate_wingtip_deflection(self):
        return 
    
    def calculate_wingtip_twist(self):
        return 
    


if __name__ == "__main__":

    stress_analysis = StressAnalysisWing(
        aircraft_data=Data("design3.json"),
        airfoil_aerodynamics=Data("AeroForces.txt", 'aerodynamics'),
        airfoil_data=Data("Airfoil_data.dat", 'airfoil_geometry')

    )
    

    


        