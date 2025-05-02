import matplotlib.pyplot as plt
import numpy as np
from Iteration import iteration
from ClassIWeightEstimation import MissionType, AircraftType


def plot_A(
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
    
    for A in range(5, 12, 1):
        fuel_economies = []
        for R in range(2000, 2900, 100):
            R = R * 1.852 * 1000
            print(R, A)
            fuel_economy, MTOM_history = iteration(
                            aircraft_type=aircraft_type,
                            mission_type=mission_type,
                            Range=R,
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
                            aspect_ratios=aspect_ratios,
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
            fuel_economies.append(fuel_economy)
        plt.plot(range(2000, 2900, 100), fuel_economies, label=f'Aspect Ratio {A}')

    plt.plot([2800, 2000,], [0.24, 0.24], 'r--', label='Fuel Economy Limit')
    plt.xlabel('Range [nmi]')
    plt.ylabel('Fuel Economy [L/mt/km]')
    plt.title('Fuel Economy vs Range')
    plt.grid()
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', title=f'Altitude = {cruise_altitude} m')
    plt.tight_layout()
    plt.show()

def plot_h(
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
    
    for h in np.arange(3, 8, 1):
        fuel_economies = []
        for R in range(2000, 2900, 100):
            R = R * 1.852 * 1000
            fuel_economy, MTOM_history = iteration(
                            aircraft_type=aircraft_type,
                            mission_type=mission_type,
                            Range=R,
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
                            aspect_ratios=aspect_ratios,
                            stall_speed_clean=stall_speed_clean,
                            stall_speed_takeoff=stall_speed_takeoff,
                            stall_speed_landing=stall_speed_landing,
                            cruise_altitude=h,
                            high_altitude=high_altitude,
                            hull_surface=hull_surface,
                            L=L,
                            rho_water=rho_water,
                            kinematic_viscosity=kinematic_viscosity
                            )
            fuel_economies.append(fuel_economy)
        plt.plot(range(2000, 2900, 100), fuel_economies, label=f'Altitude {h} m')

    plt.plot([2800, 2000,], [0.24, 0.24], 'r--', label='Fuel Economy Limit')
    plt.xlabel('Range [nmi]')
    plt.ylabel('Fuel Economy [L/mt/km]')
    plt.title('Fuel Economy vs Range')
    plt.grid()
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', title=f'Aspect Ratio = {A}')
    plt.tight_layout()
    plt.show()

def plot_A_h(
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
    ranges_nmi = list(range(2000, 2900, 100))
    ranges_m = [r * 1.852 * 1000 for r in ranges_nmi]

    markers = ['o', 's', '^']                    # Circle, square, triangle
    colors = ['tab:blue', 'tab:green', 'tab:orange']         # One for each h

    altitudes = list(np.arange(4, 6, 1))         # h = 3, 4, 5
    aspect_ratios = list(range(9, 11, 1))        # A = 8, 9, 10

    for h_index, h in enumerate(altitudes):
        color = colors[h_index % len(colors)]

        for A_index, A in enumerate(aspect_ratios):
            marker = markers[A_index % len(markers)]
            fuel_economies = []

            for R in ranges_m:
                fuel_economy, MTOM_history = iteration(
                    aircraft_type=aircraft_type,
                    mission_type=mission_type,
                    Range=R,
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
                    aspect_ratios=aspect_ratios,
                    stall_speed_clean=stall_speed_clean,
                    stall_speed_takeoff=stall_speed_takeoff,
                    stall_speed_landing=stall_speed_landing,
                    cruise_altitude=h,
                    high_altitude=high_altitude,
                    hull_surface=hull_surface,
                    L=L,
                    rho_water=rho_water,
                    kinematic_viscosity=kinematic_viscosity
                )
                fuel_economies.append(fuel_economy)

            plt.plot(
                ranges_nmi,
                fuel_economies,
                label=f'A={A}, h={h} m',
                marker=marker,
                color=color
            )

    plt.plot([2000, 2800], [0.24, 0.24], 'r--', label='Fuel Economy Limit')
    plt.xlabel('Range [nmi]')
    plt.ylabel('Fuel Economy [L/mt/km]')
    plt.title('Fuel Economy vs Range for Various A and h')
    plt.grid()
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    aircraft_type = AircraftType.PROP
    mission_type = MissionType.DESIGN
    Range = (2800+50)*1.852*1000
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

    plot_A(
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
    plot_h(
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
#     plot_A_h(
#     aircraft_type=aircraft_type,
#     mission_type=mission_type,
#     cruise_speed=cruise_speed,
#     jet_consumption=jet_consumption,
#     prop_consumption=prop_consumption,
#     prop_efficiency=prop_efficiency,
#     Cd0=Cd0,
#     e=e,
#     A=A,
#     tfo=tfo,
#     k=k,
#     n_engines=n_engines,
#     reserve_fuel=reserve_fuel,
#     CLmax_clean=CLmax_clean,
#     CLmax_takeoff=CLmax_takeoff,
#     CLmax_landing=CLmax_landing,
#     aspect_ratios=[A],
#     stall_speed_clean=stall_speed_clean,
#     stall_speed_takeoff=stall_speed_takeoff,
#     stall_speed_landing=stall_speed_landing,
#     cruise_altitude=cruise_altitude,
#     high_altitude=high_altitude,
#     hull_surface=hull_surface,
#     L=L,
#     rho_water=rho_water,
#     kinematic_viscosity=kinematic_viscosity
# )