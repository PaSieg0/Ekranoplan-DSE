import numpy as np
from scipy.optimize import fsolve
from WingLoading import main
from ClassIWeightEstimation import ClassI, MissionType, AircraftType


def solve_hb(target_A_A):
    h_b = np.arange(0, 1, 0.00001)
    y = 1 - np.exp(-4.74*h_b**0.814) - h_b**2*np.exp(-3.88*h_b**0.758)

    for i, y_val in enumerate(y):
        diff = abs((1/target_A_A - y_val))
        if diff <= 0.01:
            return h_b[i]
        
    raise ValueError

def Ainf_Ah(h_b):
    return 1 - np.exp(-4.74*h_b**0.814) - h_b**2*np.exp(-3.88*h_b**0.758)
            




def iteration(aircraft_type,
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
    CLASS_I = ClassI(
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
        reserve_fuel=reserve_fuel
    )
    

    CLASS_I.main()
    prev_MTOM = CLASS_I.MTOM
    print(f"Initial MTOM: {CLASS_I.MTOM:=,.2f}")

    tolerance = 0.001
    max_iterations = 100
    iteration = 0

    
    WP, TW, WS = main(
        plot_type=AircraftType.MIXED,
        CLmax_clean=CLmax_clean,
        CLmax_takeoff=CLmax_takeoff,
        CLmax_landing=CLmax_landing,
        aspect_ratios=aspect_ratios,
        Cd0=Cd0,
        e=e,
        k=np.sqrt(1),
        stall_speed_clean=stall_speed_clean,
        stall_speed_takeoff=stall_speed_takeoff,
        stall_speed_landing=stall_speed_landing,
        cruise_altitude=cruise_altitude,
        high_altitude=high_altitude,
        cruise_speed=cruise_speed,
        prop_efficiency=prop_efficiency,
        hull_surface=hull_surface,
        L=L,
        rho_water=rho_water,
        kinematic_viscosity=kinematic_viscosity,
        PLOT_OUTPUT=False
    )




    while True:
        iteration += 1

        S = CLASS_I.MTOW/WS
        b = np.sqrt(S*A)
        h = 30*0.3048
        h_b = h/b
        Ah = A/Ainf_Ah(h_b)
        CLASS_I = ClassI(
            aircraft_type=aircraft_type,
            mission_type=mission_type,
            cruise_speed=cruise_speed,
            jet_consumption=jet_consumption,
            prop_consumption=prop_consumption,
            prop_efficiency=prop_efficiency,
            Cd0=Cd0,
            e=e,
            A=Ah,
            tfo=tfo,
            k=np.sqrt(1),
            reserve_fuel=reserve_fuel
        )
        CLASS_I.main()
        curr_MTOM = CLASS_I.MTOM

        WP, TW, WS = main(
            plot_type=AircraftType.MIXED,
            CLmax_clean=CLmax_clean,
            CLmax_takeoff=CLmax_takeoff,
            CLmax_landing=CLmax_landing,
            aspect_ratios=[Ah],
            Cd0=Cd0,
            e=e,
            k=np.sqrt(1),
            stall_speed_clean=stall_speed_clean,
            stall_speed_takeoff=stall_speed_takeoff,
            stall_speed_landing=stall_speed_landing,
            cruise_altitude=cruise_altitude,
            high_altitude=high_altitude,
            cruise_speed=cruise_speed,
            prop_efficiency=prop_efficiency,
            hull_surface=hull_surface,
            L=L,
            rho_water=rho_water,
            kinematic_viscosity=kinematic_viscosity,
            PLOT_OUTPUT=False
        )

        diff_ratio = abs((curr_MTOM - prev_MTOM) / prev_MTOM)
        print(f"Iteration {iteration}, MTOM= {curr_MTOM}: ΔMTOM ratio = {diff_ratio:.5f}")

        if diff_ratio < tolerance or iteration >= max_iterations:
            print(f"Final MTOM: {CLASS_I.MTOM:=,.2f} kg")
            print(f"Final S: {S:=,.2f} m^2")
            print(f"Final A: {A:=,.2f}")
            print(f"Final b: {b:=,.2f} m")
            print(f"Final h: {h:=,.2f} m")
            break

        prev_MTOM = curr_MTOM
        A = Ah




if __name__=='__main__':
    aircraft_type = AircraftType.PROP
    mission_type = MissionType.DESIGN
    cruise_speed = 180*0.51444
    jet_consumption = 19e-6
    prop_consumption = 90e-9
    prop_efficiency = 0.82
    Cd0 = 0.02
    e = 0.85
    A = 8
    tfo = 0.001
    reserve_fuel = 0
    k = np.sqrt(2)

    CLmax_clean=[1.5, 1.6, 1.7]
    CLmax_takeoff=[1.6, 1.8, 2.0, 2.2]
    CLmax_landing=[1.6, 1.9, 2.2]
    aspect_ratios=[A]
    stall_speed_clean=140*0.5144
    stall_speed_takeoff=140*0.5144
    stall_speed_landing=140*0.5144
    cruise_altitude=100*0*0.3148
    high_altitude=10000*0.3148
    prop_efficiency=0.8
    hull_surface=400
    L=30
    rho_water=1000.0
    kinematic_viscosity=1.002e-6

    iteration(
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