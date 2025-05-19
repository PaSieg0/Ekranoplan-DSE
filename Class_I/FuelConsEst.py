import numpy as np
import math


def compute_cp(eta_p, L_D, W4_W5, R, g=9.80665):
    """
    Calculate the specific fuel consumption cp.
    
    Parameters:
    eta_p (float): Propeller efficiency
    L_D (float): Lift-to-drag ratio
    W4_W5 (float): Ratio Initial weight (cruise start) / Final weight (cruise end)
    R (float): Range (in meters)
    g (float): Gravitational acceleration (default is 9.80665 m/s²)
    
    Returns:
    float: Specific fuel consumption cp
    """
    return (eta_p * L_D * np.log(W4_W5)) / (R * g)



def compute_cj(V, L_D, W4_W5, R, g=9.80665):
    """
    Calculate the specific fuel consumption cj for a jet aircraft.
    
    Parameters:
    eta_p (float): Propeller efficiency
    L_D (float): Lift-to-drag ratio
    W4_W5 (float): Ratio Initial weight (cruise start) / Final weight (cruise end)
    R (float): Range (in meters)
    g (float): Gravitational acceleration (default is 9.80665 m/s²)
    
    Returns:
    float: Specific fuel consumption cj
    """
    return (V * L_D * np.log(W4_W5)) / (R * g)

print(compute_cp(0.85, 15, 1.047, 3704000))