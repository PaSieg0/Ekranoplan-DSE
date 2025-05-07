import numpy as np
class Cg:
    def __init__(self, aircraft_type: int):
        self.aircraft_type = aircraft_type
        self.equation_map = {
            1: self.Concept_2,      # Ekrano
            2: self.Concept_7,      # Double fuselage
            3: self.Concept_10,     # Sea plane
            4: self.Concept_11,     # Double wing
            5: self.Concept_13      # Flying airfoil
        }

    def Cg_calculation(self):
        match self.aircraft_type:
            case 1:
                pass  # To-do: Implement Concept_2 Cg calculation
            case 2:
                pass  # To-do: Implement Concept_7 Cg calculation
            case 3:
                pass  # To-do: Implement Concept_10 Cg calculation
            case 4:
                pass  # To-do: Implement Concept_11 Cg calculation
            case 5:
                pass  # To-do: Implement Concept_13 Cg calculation
            case _:
                raise ValueError("Invalid aircraft type")



