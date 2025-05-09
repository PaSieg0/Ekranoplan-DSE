import os
import pandas as pd
import numpy as np
from utils import Data

def hor_tail_area(V_h, S, MAC, X_h, Xcg_aft):
    """
    Calculate the horizontal tail area based on the aircraft data.
    
    Returns:
        float: The horizontal tail area.
    """
    S_h = V_h * S * MAC / (X_h - (data.data['X_LEMAC'] + Xcg_aft*data.data['MAC']))

    return S_h

def ver_tail_area(V_v, S, b, X_v, Xcg_aft):
    """
    Calculate the vertical tail area based on the aircraft data.
    
    Returns:
        float: The vertical tail area.
    """
    S_v = V_v * S * b / (X_v - (data.data['X_LEMAC'] + Xcg_aft*data.data['MAC']))

    return S_v

if __name__ == "__main__":
    data = Data("design1.json")

    V_h = 3.662649875 # From statistics, taken from Aircraft Data Excel
    V_v = 0.273372017 # From statistics, taken from Aircraft Data Excel

    S = data.data['design']['S']
    MAC = data.data['design']['MAC']
    b = data.data['design']['b']
    
    X_h = 75 #m From Excel
    X_v = 75 #m From Excel
    Xcg_aft = 0.30440835377790804 # From CG calculation

    S_h = hor_tail_area(V_h, S, MAC, X_h, Xcg_aft)
    S_v = ver_tail_area(V_v, S, b, X_v, Xcg_aft)

    print(f"Horizontal Tail Area: {S_h} m^2")
    print(f"Vertical Tail Area: {S_v} m^2")