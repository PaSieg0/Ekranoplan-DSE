import numpy as np
class Cg:
    def __init__(self, aircraft_type: int):
        self.aircraft_type = aircraft_type
        self.equation_map = {
            2: self.Concept_2,      # Ekrano
            7: self.Concept_7,      # Double fuselage
            10: self.Concept_10,     # Sea plane
            11: self.Concept_11,     # Double wing
            13: self.Concept_13      # Flying airfoil
        }

    def Cg_calculation(self):
        match self.aircraft_type:
            case 2:
                pass  # To-do: Implement Concept_2 Cg calculation
            case 7:
                pass  # To-do: Implement Concept_7 Cg calculation
            case 10:
                pass  # To-do: Implement Concept_10 Cg calculation
            case 11:
                pass  # To-do: Implement Concept_11 Cg calculation
            case 13:
                pass  # To-do: Implement Concept_13 Cg calculation
            case _:
                raise ValueError("Invalid aircraft type")



