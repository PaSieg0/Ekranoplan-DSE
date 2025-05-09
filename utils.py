import json
import os
from enum import Enum, auto
from typing import Any

class AircraftType(Enum):
    JET = auto()
    PROP = auto()
    MIXED = auto()


class MissionType(Enum):
    DESIGN = auto()
    FERRY = auto()
    ALTITUDE = auto()


class EnumEncoder(json.JSONEncoder):
    """
    Makes sure that enums are saved as strings correctly in the json file
    """
    def default(self, o):
        if isinstance(o, Enum):
            return o.name
        return json.JSONEncoder.default(self, o)


class Data:
    def __init__(self, design_file):
        self.data = self.load_design(design_file)

    def load_design(self, design_file) -> dict[str, Any]:
        file_path = os.path.join("Data", design_file)
        try:
            with open(file_path, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            print(f"Error: {file_path} not found.")
        except json.JSONDecodeError:
            print(f"Error: Failed to decode JSON from {file_path}.")
        return {}

    def save_design(self, design_file):
        file_path = os.path.join("Data", design_file)
        try:
            with open(file_path, 'w') as file:
                json.dump(self.data, file, cls=EnumEncoder, indent=4)
        except IOError as e:
            print(f"Error: Failed to write to {file_path}. {e}")


# Example usage
if __name__ == "__main__":
    aircraft = Data("design1.json")
    print(aircraft.data)
    aircraft.data["aircraft_type"] = AircraftType.JET.name
    aircraft.save_design("design1.json")
    print(aircraft.data)

    print("Aircraft Type:", AircraftType[aircraft.data["aircraft_type"]])