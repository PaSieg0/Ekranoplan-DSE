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
                        kinematic_viscosity=kinematic_viscosity
                        )
    # Example usage
    W_final = MTOM_history[-1] * 9.81  # Weight in N
    S = S_final  # Wing area in m^2    
    altitude = 10000 # ft
    g = 9.81 # m/s^2
    atmosphere_alt = ISA(altitude*0.3048)
    CL_alpha = 5 # rad/s, TBD aerodynamics   
    w = W_final/S # N/m^2
    w = w*2.20462262/(g*3.2808399^2) # lb/ft^2
    
    mu = Calculate_mu(w, atmosphere_alt.rho*0.0019403203319541, c, CL_alpha, g)
    K_g = Calculate_K_g(mu)
    V_b = Calculate_V_b(stall_speed_clean, K_g, Calculate_U_ref(altitude), cruise_speed, CL_alpha)