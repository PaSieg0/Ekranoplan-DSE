import json
import os
from enum import Enum, auto

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
        self.load_design(design_file)

    def load_design(self, design_file):
        file_path = os.path.join("Data", design_file)
        try:
            with open(file_path, 'r') as file:
                design_data = json.load(file)
                for key, value in design_data.items():
                    setattr(self, key, value)
        except FileNotFoundError:
            print(f"Error: {file_path} not found.")
        except json.JSONDecodeError:
            print(f"Error: Failed to decode JSON from {file_path}.")


# Example usage
if __name__ == "__main__":
    aircraft = Data("design1.json")
    print(aircraft.__dict__)