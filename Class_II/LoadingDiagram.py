import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def common_entries(*dcts):
    if not dcts:
        return
    for i in set(dcts[0]).intersection(*dcts[1:]):
        yield (i,) + tuple(d[i] for d in dcts)

class LoadingDiagram:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"


    def determine_OEW_cg(self):
        weight_pos_product = 1
        for key, val1, val2 in common_entries(
            self.aircraft_data.data['outputs']['component_weights'],
            self.aircraft_data.data['outputs']['component_positions'],
        ):
            if key != 'total_OEW':
                weight_pos_product *= val1 * val2
        oew_cg = weight_pos_product / self.aircraft_data.data['outputs']['component_weights']['total_OEW']
        return oew_cg
