import numpy as np
from scipy.optimize import fsolve
from WingLoading import main
from ClassIWeightEstimation import ClassI, MissionType, AircraftType
import matplotlib.pyplot as plt
from ISA_Class import ISA


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
              Range,
              cruise_speed,
              jet_consumption,
              prop_consumption,
              prop_efficiency,
              Cd0,
              e,
              A,
              tfo,
              k,
              n_engines,
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
    CLASS_I.range = Range
    

    CLASS_I.main()
    
    prev_MTOM = CLASS_I.MTOM
    print(f"Initial MTOM: {prev_MTOM:=,.2f} kg")

    tolerance = 0.0001
    max_iterations = 10
    iteration = 0

    
    WP, TW, WS = main(
        plot_type=aircraft_type,
        CLmax_clean=CLmax_clean,
        CLmax_takeoff=CLmax_takeoff,
        CLmax_landing=CLmax_landing,
        aspect_ratios=aspect_ratios,
        Cd0=Cd0,
        e=e,
        k=k,
        n_engines=n_engines,
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

    MTOM_history = []
    MTOM_history.append(prev_MTOM)

    print(f"Initial MTOM: {CLASS_I.MTOM:=,.2f} kg")


    while True:
        iteration += 1

        S = CLASS_I.MTOW/WS
        b = np.sqrt(S*A)
        h_b = cruise_altitude/b
        A_ratio = Ainf_Ah(h_b)
        new_k = np.sqrt(1/A_ratio)
        fuel_economy = CLASS_I.fuel_used/9.81*0.82/90/(2800*1.852)
        print(f"S={S}")
        print(f"b={b}")
        print(f"h_b={h_b}")
        print(f"A_ratio={A_ratio}")
        print(f"new_k={new_k}")

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
            k=new_k,
            reserve_fuel=reserve_fuel
        )
        CLASS_I.range = Range
        CLASS_I.main()
        curr_MTOM = CLASS_I.MTOM
        MTOM_history.append(curr_MTOM)

        WP, TW, WS = main(
            plot_type=aircraft_type,
            CLmax_clean=CLmax_clean,
            CLmax_takeoff=CLmax_takeoff,
            CLmax_landing=CLmax_landing,
            aspect_ratios=[A],
            Cd0=Cd0,
            e=e,
            k=new_k,
            n_engines=n_engines,
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
        print(f"k: {new_k}")

        if diff_ratio < tolerance or iteration >= max_iterations:
            print(f"Final MTOM: {CLASS_I.MTOM:=,.2f} kg")
            print(f"Final fuel economy: {CLASS_I.fuel_used/9.81*0.82/90/(2800*1.852):.4f} L/ton/km")
            print()
            print()
            print(f"Final MTOW: {CLASS_I.MTOW:=,.2f} N")
            print(f"Final OEW: {CLASS_I.OEW/9.81:=,.2f} kg")
            print(f"Final ZFW: {CLASS_I.ZFW/9.81:=,.2f} kg")
            print(f"Final EW: {CLASS_I.EW/9.81:=,.2f} kg")
            print(f"Final Payload: {CLASS_I.payload:=,.2f} kg")
            print(f"Final Fuel: {CLASS_I.fuel/9.81:=,.2f} kg")
            print(f"Final Fuel used: {CLASS_I.fuel_used/9.81:=,.2f} kg")
            print(f"Final Fuel reserve: {CLASS_I.fuel_res/9.81:=,.2f} kg")


            print(f"Final S: {S:=,.2f} m^2")
            print(f"Final A: {A:=,.2f}")
            print(f"Final b: {b:=,.2f} m")
            print(f"Final cruise_altitude: {cruise_altitude:=,.2f} m")
            print(f"Final h_b: {h_b:=,.2f}")
            print(f"Final k: {new_k:=,.2f}")
            if aircraft_type == AircraftType.JET or aircraft_type == AircraftType.MIXED:
                print(f"Final TW: {TW:=,.2f} ")
                print(f"Final T: {TW*CLASS_I.MTOW:=,.2f} N")
            if aircraft_type == AircraftType.PROP or aircraft_type == AircraftType.MIXED:
                print(f"Final WP: {WP:=,.2f} ")
                print(f"Final P: {CLASS_I.MTOW/WP:=,.2f} ")
            break

        prev_MTOM = curr_MTOM


    def calculate_Cd():
        Re = 1.05*stall_speed_takeoff*L / kinematic_viscosity
        Cd = 0.075 / (np.log10(Re) - 2)**2
        return Cd
    
    def take_off_requirement():
        CL_takeoff = k * np.array(CLmax_takeoff)/1.21
        Cd = calculate_Cd()
        print(f"Cd: {Cd:.4f}")
        D = 0.5 * rho_water * (1.05*stall_speed_takeoff)**2 * Cd * hull_surface

        print(f"Thrust shall be at least {D:=,.0f} N.")
        print(f"Power shall be at least {D*1.05*stall_speed_takeoff:=,.0f} W.")
        

        x = [CL*0.5*ISA(cruise_altitude).rho * (1.05*stall_speed_takeoff)**2 for CL in CL_takeoff]
        return x
    
    take_off_requirement()

    return fuel_economy, MTOM_history


if __name__=='__main__':
    aircraft_type = AircraftType.PROP
    mission_type = MissionType.ALTITUDE
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
    final_MTOMS = []
    fuel_economy, MTOM_history = iteration(
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
    print(fuel_economy)
    # for A in range(10, 11, 1):
    #     fuel_economy, MTOM_history = iteration(
    #                         aircraft_type=aircraft_type,
    #                         mission_type=mission_type,
    #                         cruise_speed=cruise_speed,
    #                         jet_consumption=jet_consumption,
    #                         prop_consumption=prop_consumption,
    #                         prop_efficiency=prop_efficiency,
    #                         Cd0=Cd0,
    #                         e=e,
    #                         A=A,
    #                         tfo=tfo,
    #                         k=k,
    #                         reserve_fuel=reserve_fuel,
    #                         CLmax_clean=CLmax_clean,
    #                         CLmax_takeoff=CLmax_takeoff,
    #                         CLmax_landing=CLmax_landing,
    #                         aspect_ratios=[A],
    #                         stall_speed_clean=stall_speed_clean,
    #                         stall_speed_takeoff=stall_speed_takeoff,
    #                         stall_speed_landing=stall_speed_landing,
    #                         cruise_altitude=cruise_altitude,
    #                         high_altitude=high_altitude,
    #                         hull_surface=hull_surface,
    #                         L=L,
    #                         rho_water=rho_water,
    #                         kinematic_viscosity=kinematic_viscosity
    #                         )
    #     final_MTOMS.append(round(MTOM_history[-1]))
    # print(final_MTOMS)

