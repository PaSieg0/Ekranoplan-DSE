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
                m_main_wing = 16.3
                m_payload = 20.6 + 7.8
                m_fuselage = 14.6 #hull
                m_legs = 2.9 # all that touches water outside fo hull
                m_tail = 2.3 + 3.6
                m_engine = 10.4 + 1.4
                m_fuel = 15.8
                m_misc = 4.3 # all that is not in the previous categories

                l_main_wing= 0.504
                l_payload=0.471
                l_fusel=0.471
                l_legs=0.455
                l_tail=0.958
                l_engine=0.178
                l_fuel=0.477
                l_misc=0.471
                xcg = (m_main_wing*l_main_wing + m_payload*l_payload + m_fuselage*l_fusel + m_legs*l_legs + m_tail*l_tail + m_engine*l_engine + m_fuel*l_fuel + m_misc*l_misc) / (m_main_wing + m_payload + m_fuselage + m_legs + m_tail + m_engine + m_fuel + m_misc)  
                return xcg


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


cg = Cg(2).Cg_calculation()
print(f"Center of Gravity (Cg) for aircraft type 2: {cg:.3f} m")
