import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, MissionType, LoadCase
import numpy as np
from matplotlib import pyplot as plt

class CGRange:
    def __init__(self, aircraft_data: Data, plot=False):
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"
        self.aircraft_data = aircraft_data

        self.most_aft_cg = float('-inf')
        self.most_aft_mission = None

        self.most_forward_cg = float('inf')
        self.most_forward_mission = None

        self.l_fuselage = aircraft_data.data["outputs"]["general"]["l_fuselage"]

        self.plot = plot

    def calculate_cg_range(self):
        g = 9.81
        cg_points = []  # Store (label, x_cg) tuples

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
                self.x_cg = sum(
                    self.xcg_components[comp] * self.mass_components[comp] / total_weight
                    for comp in self.xcg_components
                ) * self.aircraft_data.data["outputs"]["general"]["l_fuselage"]

                label = f"{mission.name}_{load_case.name}"
                cg_points.append((label, self.x_cg, total_weight * g)) 

                if self.x_cg > self.most_aft_cg:
                    self.most_aft_cg = self.x_cg
                    self.most_aft_mission = label
                if self.x_cg < self.most_forward_cg:
                    self.most_forward_cg = self.x_cg
                    self.most_forward_mission = label

        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)

   
        labels, cgs, weights_newton = zip(*cg_points)
        masses_kg = [w / g for w in weights_newton]
        masses_tonnes = np.array(masses_kg) / 1000  # Convert once

        if self.plot:
            fig, ax = plt.subplots()

            # Normalize CGs
            cgs_norm = [cg / self.l_fuselage for cg in cgs]
            aft_index = min(range(len(cgs)), key=lambda i: abs(cgs_norm[i] - self.most_aft_cg / self.l_fuselage))
            forward_index = min(range(len(cgs)), key=lambda i: abs(cgs_norm[i] - self.most_forward_cg / self.l_fuselage))

            # Plot
            ax.scatter(cgs_norm, masses_tonnes, label="CG vs Mass", color='blue', marker='o')

            # Vertical lines at most aft/forward normalized CG
            ax.axvline(self.most_aft_cg / self.l_fuselage, color='r', linestyle='--', label="Most Aft CG")
            ax.axvline(self.most_forward_cg / self.l_fuselage, color='g', linestyle='--', label="Most Forward CG")

            # Annotate at normalized CGs
            ax.annotate(labels[aft_index], (cgs_norm[aft_index], masses_tonnes[aft_index]),
                        textcoords="offset points", xytext=(0, 0), ha='center', fontsize=8, rotation=-20)

            ax.annotate(labels[forward_index], (cgs_norm[forward_index], masses_tonnes[forward_index]),
                        textcoords="offset points", xytext=(10, -5), ha='center', fontsize=8, rotation=-20)

            designs = ['2', '7', '10']
            # Labels
            ax.set_title(f"Total Mass vs CG Position - Design {designs[self.design_number-1]}")
            ax.set_xlabel("x_cg / l_fuselage")
            ax.set_ylabel("Total Mass [tonnes]")
            ax.grid(True)
            plt.tight_layout()
            plt.show()




    def update_attributes(self):
        self.aircraft_data.data["outputs"]["cg_range"]["most_aft_cg"] = self.most_aft_cg
        self.aircraft_data.data["outputs"]["cg_range"]["most_aft_mission"] = self.most_aft_mission
        self.aircraft_data.data["outputs"]["cg_range"]["most_forward_cg"] = self.most_forward_cg
        self.aircraft_data.data["outputs"]["cg_range"]["most_forward_mission"] = self.most_forward_mission

if __name__ == "__main__":
    data = Data("design3.json")
    cg_range = CGRange(data,plot=True)
    cg_range.calculate_cg_range()