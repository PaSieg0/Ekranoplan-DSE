from utils import Data
import numpy as np
from ClassIWeightEstimation import MissionType
from enum import Enum, auto

class LoadCase(Enum):
    OEW = auto()
    OEW_PAYLOAD = auto()
    OEW_PAYLOAD_FUEL = auto()
    OEW_FUEL = auto()


class CGRange:
    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data

        self.most_aft_cg = float('-inf')
        self.most_aft_mission = None

        self.most_forward_cg = float('inf')
        self.most_forward_mission = None

    def calculate_cg_range(self):
        # Get the data from the aircraft_data object
        g = 9.81
        for load_case in LoadCase:
            for mission in MissionType:
                mission_name = mission.name.lower()
                self.xcg_components = {
                    "OEW": self.aircraft_data.data["inputs"]["xcg_OEW"],
                    "Payload": self.aircraft_data.data['outputs']['general']["xcg_payload"],
                    "Fuel": (self.aircraft_data.data["outputs"]["wing_design"]["X_LEMAC"] + 0.5*self.aircraft_data.data["outputs"]["wing_design"]["MAC"]) / self.aircraft_data.data["outputs"]["general"]["l_fuselage"]
                }

                self.mass_components = {}
                
                if load_case == LoadCase.OEW:
                    self.mass_components["Payload"] = 0
                    self.mass_components["Fuel"] = 0
                    self.mass_components["OEW"] = self.aircraft_data.data["outputs"][mission_name]["OEW"] / g
                elif load_case == LoadCase.OEW_PAYLOAD:
                    self.mass_components["Payload"] = self.aircraft_data.data["requirements"][f"{mission_name}_payload"]
                    self.mass_components["Fuel"] = 0
                    self.mass_components["OEW"] = self.aircraft_data.data["outputs"][mission_name]["OEW"] / g
                elif load_case == LoadCase.OEW_PAYLOAD_FUEL:
                    self.mass_components["Payload"] = self.aircraft_data.data["requirements"][f"{mission_name}_payload"]
                    self.mass_components["Fuel"] = self.aircraft_data.data["outputs"][mission_name]["max_fuel"] / g
                    self.mass_components["OEW"] = self.aircraft_data.data["outputs"][mission_name]["OEW"] / g
                elif load_case == LoadCase.OEW_FUEL:
                    self.mass_components["Payload"] = 0
                    self.mass_components["Fuel"] = self.aircraft_data.data["outputs"][mission_name]["max_fuel"] / g
                    self.mass_components["OEW"] = self.aircraft_data.data["outputs"][mission_name]["OEW"] / g
                
                total_weight = sum(self.mass_components.values())
                self.x_cg = sum(self.xcg_components[comp] * self.mass_components[comp] / total_weight for comp in self.xcg_components) * self.aircraft_data.data["outputs"]["general"]["l_fuselage"]
                if self.x_cg > self.most_aft_cg:
                    self.most_aft_cg = self.x_cg
                    self.most_aft_mission = f"{mission_name.upper()}_{load_case.name.upper()}"
                if self.x_cg < self.most_forward_cg:
                    self.most_forward_cg = self.x_cg
                    self.most_forward_mission = f"{mission_name.upper()}_{load_case.name.upper()}"

        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)


    def update_attributes(self):
        self.aircraft_data.data["outputs"]["cg_range"]["most_aft_cg"] = self.most_aft_cg
        self.aircraft_data.data["outputs"]["cg_range"]["most_aft_mission"] = self.most_aft_mission
        self.aircraft_data.data["outputs"]["cg_range"]["most_forward_cg"] = self.most_forward_cg
        self.aircraft_data.data["outputs"]["cg_range"]["most_forward_mission"] = self.most_forward_mission

if __name__ == "__main__":
    data = Data("design1.json")
    cg_range = CGRange(data)
    cg_range.calculate_cg_range()