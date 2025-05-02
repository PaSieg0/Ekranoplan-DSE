import matplotlib.pyplot as plt
import numpy as np
from Iteration import iteration
from ClassIWeightEstimation import MissionType, AircraftType



def plot(
        aircraft_type,
        mission_type,
        cruise_speed,
        jet_consumption,
        prop_consumption,
        prop_efficiency,
        Cd0,
        e,
        A,
        tfo,
        k,
        reserve_fuel,
        CLmax_clean,
        CLmax_takeoff,
        CLmax_landing,
        aspect_ratios,
        stall_speed_clean,
        stall_speed_takeoff,
        stall_speed_landing,
        cruise_altitude,
        high_altitude,
        hull_surface,
        L,
        rho_water,
        kinematic_viscosity
):
    
    for A in range(5, 20, 1):
        fuel_economy, MTOM_history = iteration(
                            aircraft_type=aircraft_type,
                            mission_type=mission_type,
                            cruise_speed=cruise_speed,
                            jet_consumption=jet_consumption,
                            prop_consumption=prop_consumption,
                            prop_efficiency=prop_efficiency,
                            Cd0=Cd0,
                            e=e,
                            A=A,
                            tfo=tfo,
                            k=k,
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
    
    Range = [2800, 2700, 2600, 2500, 2400, 2300, 2200, 2100, 2000]

    plt.plot(Range, fuel_economy, marker='o', label='Fuel Economy')
    plt.plot([2800, 2000,], [0.24, 0.24], 'r--', label='Fuel Economy Limit')
    plt.xlabel('Range [nmi]')
    plt.ylabel('Fuel Economy [L/mt/km]')
    plt.title('Fuel Economy vs Range')
    plt.grid()
    plt.legend()
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

    CLmax_clean=[1.5, 1.6, 1.7]
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


