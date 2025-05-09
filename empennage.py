import os
import pandas as pd
import numpy as np
from utils import Data

def hor_tail_area(Xcg_aft):
    """
    Calculate the horizontal tail area based on the aircraft data.
    
    Returns:
        float: The horizontal tail area.
    """
    S_h = data.data['inputs']['V_h'] * data.data['outputs']['max']['S'] * data.data['outputs']['max']['MAC'] / (data.data['outputs']['general']['l_fuselage'] - (data.data['outputs']['wing_design']['X_LEMAC'] + Xcg_aft*data.data['outputs']['max']['MAC']))

    return S_h

def ver_tail_area(Xcg_aft):
    """
    Calculate the vertical tail area based on the aircraft data.
    
    Returns:
        float: The vertical tail area.
    """
    S_v = data.data['inputs']['V_v'] * data.data['outputs']['max']['S'] * data.data['outputs']['max']['b'] / (data.data['outputs']['general']['l_fuselage'] - (data.data['outputs']['wing_design']['X_LEMAC'] + Xcg_aft*data.data['outputs']['max']['MAC']))

    return S_v

if __name__ == "__main__":
    data = Data("design1.json")

    Xcg_aft = 0.30440835377790804 # From CG calculation

    S_h = hor_tail_area( Xcg_aft)
    S_v = ver_tail_area( Xcg_aft)

    print(f"Horizontal Tail Area: {S_h} m^2")
    print(f"Vertical Tail Area: {S_v} m^2")