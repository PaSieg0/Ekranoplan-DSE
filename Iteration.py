from WingLoading import main, AircraftType
from ClassIWeightEstimation import ClassI, MissionType


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
              reserve_fuel
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
        reserve_fuel=reserve_fuel
    )
    

    prev_MTOM = float('inf')
    curr_MTOM = float('inf')
    while (curr_MTOM-prev_MTOM)/prev_MTOM >= 0.05:
        CLASS_I.main()
        prev_MTOM = curr_MTOM
        curr_MTOM = CLASS_I.MTOM

        print((curr_MTOM-prev_MTOM)/prev_MTOM)



if __name__=='__main__':
    aircraft_type = AircraftType.PROP
    mission_type = MissionType
    cruise_speed=180*0.5144
    jet_consumption = 19e-6
    prop_consumption = 90e-9
    prop_efficiency = 0.82
    Cd0 = 0.02
    e=0.85
    A = 7
    tfo = 0.001
    reserve_fuel = 0

    CLmax_clean=[1.5, 1.6, 1.7]
    CLmax_takeoff=[1.6, 1.8, 2.0, 2.2]
    CLmax_landing=[1.6, 1.9, 2.2]
    aspect_ratios=[7]
    Cd0=0.02
    stall_speed_clean=100*0.5144
    stall_speed_takeoff=110*0.5144
    stall_speed_landing=120*0.5144
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
        reserve_fuel=reserve_fuel
        )