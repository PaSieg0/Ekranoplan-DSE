import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from utils import Data, ISA

def Calculate_U_ref(altitude):
    U_ref = 17.07-((17.07-13.41)/15000)*altitude # m/s
    return U_ref


def Calculate_K_g(mu):
    K_g = 0.88*mu/(5.3+mu) 
    return K_g

def Calculate_mu(w, rho, c, CL_alpha, g):
    mu = 2*w/(rho*c*CL_alpha*g) 
    return mu

def Calculate_V_b(V_stall, K_g, U_ref, V_c, CL_alpha, w):
    V_b = V_stall * (1+ (K_g*U_ref*V_c*CL_alpha)/(498*w))**0.5 # m/s
    return V_b

import numpy as np
import matplotlib.pyplot as plt

def point_determination(aircraft_data, h):
    # Example usage
    g = aircraft_data.data["gravitational_acceleration"]
    W_final = aircraft_data.data["design"]["MTOM"] * g  # Weight in N
    U_ref = Calculate_U_ref(h/0.3048)  # Reference speed in m/s
    S = aircraft_data.data["design"]["MTOM"] * g / aircraft_data.data["design"]["WS"]  # Wing area in m^2   
    rho = ISA(h).rho
    CL_alpha = aircraft_data.data["Cl_alpha"] # rad/s, TBD aerodynamics   
    w = aircraft_data.data["design"]["WS"] # N/m^2
    w = w*2.20462262/(g*3.2808399**2) # lb/ft^2
    b = np.sqrt(aircraft_data.data["design"]["aspect_ratio"]*S)
    chord = S/b # m
    mu = Calculate_mu(w, rho*0.0019403203319541, chord, CL_alpha, g)
    K_g = Calculate_K_g(mu)
    V_b = Calculate_V_b(aircraft_data.data["stall_speed_clean"], K_g, U_ref, aircraft_data.data["cruise_speed"], CL_alpha, w)
    dive_speed = aircraft_data.data["cruise_speed"] / 0.8 # m/s, TBD
    V = [[float(V_b), U_ref], [aircraft_data.data["cruise_speed"], U_ref], [dive_speed, U_ref/2]]

    for i, (V_eas, U_gust) in enumerate(V):
        # Δn  = ρ · V · CLα · Uref · S / (2 W)
        delta_n = rho * V_eas * CL_alpha * U_gust * S / (2 * W_final)
        V[i].append(float(delta_n))      # cast to built-in float for a cleaner printout
    return V

def plot_gust_load_diagrams(V):
    """
    V  =  [[V_eas, U_ref, Δn], …]     (speeds in m/s, Δn dimensionless)
    """
    plt.figure(figsize=(10, 6))
    plt.title("Symmetric-gust load factors")
    plt.xlabel("Equivalent air-speed V  [m s⁻¹]")
    plt.ylabel("Load factor n")

    # --------------------------------------------------------------
    # 1. build positive and negative branches (include the origin)
    # --------------------------------------------------------------
    pos_V, pos_n = [0.0], [1.0]
    neg_V, neg_n = [0.0], [1.0]

    for V_eas, _, delta_n in V:
        pos_V.append(V_eas);  pos_n.append(1.0 + delta_n)
        neg_V.append(V_eas);  neg_n.append(1.0 - delta_n)

    # sort so the envelope is drawn in ascending-speed order
    order = np.argsort(pos_V)
    pos_V, pos_n = np.array(pos_V)[order], np.array(pos_n)[order]
    neg_V, neg_n = np.array(neg_V)[order], np.array(neg_n)[order]

    # --------------------------------------------------------------
    # 2. plot the points and lines
    # --------------------------------------------------------------
    plt.scatter(pos_V, pos_n, color="black")
    plt.scatter(neg_V, neg_n, color="black")

    plt.plot(pos_V, pos_n, color="black",  lw=1.5)
    plt.plot(neg_V, neg_n, color="black", lw=1.5)

    # --------------------------------------------------------------
    # 3. close the envelope at the right-hand end (highest V)
    # --------------------------------------------------------------
    max_V          = pos_V[-1]           # same in both lists after sorting
    pos_n_at_maxV  = pos_n[-1]
    neg_n_at_maxV  = neg_n[-1]

    plt.plot([max_V, max_V],
             [neg_n_at_maxV, pos_n_at_maxV],
             color="black", lw=1.5)

    plt.grid(True, which="both", ls=":")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":

    aircraft_data = Data("design1.json")
    
    points = point_determination(aircraft_data, h=0)
    plot_gust_load_diagrams(np.array(points))
