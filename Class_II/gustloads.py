import numpy as np
import matplotlib.pyplot as plt

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import Iteration
from ClassIWeightEstimation import MissionType, AircraftType
from ISA_Class import ISA

def Calculate_U_ref(altitude):
    U_ref = 17.07-((17.07-13.41)/15000)*altitude # m/s
    return U_ref


def Calculate_K_g(mu):
    K_g = 0.88*mu/(5.3+mu) 
    return K_g

def Calculate_mu(w, rho, c, CL_alpha, g):
    mu = 2*w/(rho*c*CL_alpha*g) 
    return mu

def Calculate_V_b(V_stall, K_g, U_ref, V_c, CL_alpha):
    V_b = V_stall * (1+ (K_g*U_ref*V_c*CL_alpha)/(498*w))**0.5 # m/s
    return V_b

import numpy as np
import matplotlib.pyplot as plt

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

    # centre-line n = 1 g
    plt.axhline(1.0, color="black", ls="--", lw=0.8)

    plt.grid(True, which="both", ls=":")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":

    aircraft_type = AircraftType.PROP
    mission_type = MissionType.DESIGN
    cruise_speed = 225*0.51444
    jet_consumption = 19e-6
    prop_consumption = 90e-9
    prop_efficiency = 0.82
    Cd0 = 0.02
    e = 0.85
    A = 10
    tfo = 0.001
    reserve_fuel = 0
    k = 1
    n_engines = [4, 6, 8, 10]

    CLmax_clean=[1.5]
    CLmax_takeoff=[1.6, 1.8, 2.0, 2.2]
    CLmax_landing=[1.8, 1.9, 2.2]
    aspect_ratios=[A]
    stall_speed_clean=150*0.5144
    stall_speed_takeoff=120*0.5144
    stall_speed_landing=100*0.5144
    cruise_altitude=5
    high_altitude=10000*0.3048
    L=40
    r=3
    hull_surface=2*np.pi*L*r / 3
    rho_water=1000.0
    kinematic_viscosity=1.002e-6
    final_MTOMS = []
    fuel_economy, MTOM_history, S_final = Iteration.iteration(
                        aircraft_type=aircraft_type,
                        mission_type=mission_type,
                        Range=2800*1.852*1000,
                        cruise_speed=cruise_speed,
                        jet_consumption=jet_consumption,
                        prop_consumption=prop_consumption,
                        prop_efficiency=prop_efficiency,
                        Cd0=Cd0,
                        e=e,
                        A=A,
                        tfo=tfo,
                        k=k,
                        n_engines=n_engines,
                        reserve_fuel=reserve_fuel,
                        CLmax_clean=CLmax_clean,
                        CLmax_takeoff=CLmax_takeoff,
                        CLmax_landing=CLmax_landing,
                        aspect_ratios=[A],
                        stall_speed_clean=stall_speed_clean,
                        stall_speed_takeoff=stall_speed_takeoff,
                        stall_speed_landing=stall_speed_landing,
                        cruise_altitude=cruise_altitude,
                        high_altitude=high_altitude,
                        hull_surface=hull_surface,
                        L=L,
                        rho_water=rho_water,
                        kinematic_viscosity=kinematic_viscosity,
                        )
    # Example usage
    W_final = MTOM_history[-1] * 9.81  # Weight in N
    altitude = 10000 # ft
    U_ref = Calculate_U_ref(altitude)  # Reference speed in m/s
    S = S_final  # Wing area in m^2    
    g = 9.81 # m/s^2
    atmosphere_alt = ISA(altitude*0.3048)
    CL_alpha = 5 # rad/s, TBD aerodynamics   
    w = W_final/S # N/m^2
    w = w*2.20462262/(g*3.2808399**2) # lb/ft^2
    chord = np.sqrt(A*S) 
    mu = Calculate_mu(w, atmosphere_alt.rho*0.0019403203319541, chord, CL_alpha, g)
    K_g = Calculate_K_g(mu)
    V_b = Calculate_V_b(stall_speed_clean, K_g, U_ref, cruise_speed, CL_alpha)
    dive_speed = 300*0.51444 # m/s, TBD
    V = [[float(V_b), U_ref], [cruise_speed, U_ref], [dive_speed, U_ref/2]]

    for i, (V_eas, U_gust) in enumerate(V):
        # Δn  = ρ · V · CLα · Uref · S / (2 W)
        delta_n = atmosphere_alt.rho * V_eas * CL_alpha * U_gust * S / (2 * W_final)
        V[i].append(float(delta_n))      # cast to built-in float for a cleaner printout
    
    plot_gust_load_diagrams(np.array(V))
