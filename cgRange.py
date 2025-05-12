from utils import Data
import numpy as np
from ClassIWeightEstimation import MissionType

class CGRange:
    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data

        self.most_aft_cg = float('-inf')

    def calculate_cg_range(self):
        # Get the data from the aircraft_data object
        g = 9.81
        for mission in MissionType:
            mission_name = mission.name.lower()
            self.xcg_components = {
                "OEW": self.aircraft_data.data["inputs"]["xcg_OEW"],
                "Payload": self.aircraft_data.data["inputs"]["xcg_payload"],
                "Fuel": (self.aircraft_data.data["outputs"]["wing_design"]["X_LEMAC"] + 0.5*self.aircraft_data.data["outputs"]["max"]["MAC"]) / self.aircraft_data.data["outputs"]["general"]["l_fuselage"]
            }

            self.mass_components = {
                "OEW": self.aircraft_data.data["outputs"][mission_name]["OEW"]/g,
                "Payload": self.aircraft_data.data["requirements"][f"{mission_name}_payload"],
                "Fuel": self.aircraft_data.data["outputs"][mission_name]["max_fuel"]/g
            }

            total_weight = sum(self.mass_components.values())
            self.x_cg = sum(self.xcg_components[comp] * self.mass_components[comp] / total_weight for comp in self.xcg_components) * self.aircraft_data.data["outputs"]["general"]["l_fuselage"]
            if self.x_cg > self.most_aft_cg:
                self.most_aft_cg = self.x_cg
                self.most_aft_mission = mission_name.upper()

        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)


    def update_attributes(self):
        self.aircraft_data.data["outputs"]["cg_range"]["most_aft_cg"] = self.most_aft_cg
        self.aircraft_data.data["outputs"]["cg_range"]["most_aft_mission"] = self.most_aft_mission

if __name__ == "__main__":
    data = Data("design1.json")
    cg_range = CGRange(data)
    cg_range.calculate_cg_range()