from utils import Data
import numpy as np


class CGRange:
    def __init__(self, aircraft_data: Data):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data

    def calculate_cg_range(self):
        # Get the data from the aircraft_data object
        g = 9.81
        self.xcg_components = {
            "OEW": self.aircraft_data.data["inputs"]["xcg_OEW"] / g,
            "Payload": self.aircraft_data.data["inputs"]["xcg_design_payload"],
            "Fuel": self.aircraft_data.data["inputs"]["xcg_total_fuel"] / g
        }

        self.mass_components = {
            "OEW": self.aircraft_data.data["outputs"]["max"]["OEW"]/g,
            "Payload": self.aircraft_data.data["requirements"]["design_payload"],
            "Fuel": self.aircraft_data.data["outputs"]["max"]["total_fuel"]/g
        }

        total_weight = sum(self.mass_components.values())  # Calculate total weight as the sum of all component weights
        x_cg = sum(self.xcg_components[comp] * self.mass_components[comp] / total_weight for comp in self.xcg_components)
        print(f"Total weight: {total_weight} kg")
        print(f"CG position: {x_cg} m")

if __name__ == "__main__":
    data = Data("design1.json")
    cg_range = CGRange(data)
    cg_range.calculate_cg_range()