from ClassIWeightEstimation import ClassI, MissionType, AircraftType
from WingLoading import main
import numpy as np
from Iteration import solve_hb

aircraft_type = AircraftType.MIXED
mission_type = MissionType.DESIGN
cruise_speed = 180*0.51444
jet_consumption = 19e-6
prop_consumption = 90e-9
prop_efficiency = 0.82
Cd0 = 0.02
e = 0.85
A = 10
tfo = 0.001
reserve_fuel = 0
k = 2
h_b = solve_hb(k**2)
print(f"h_b {h_b}")

CLmax_clean=[1.5, 1.6, 1.7]
CLmax_takeoff=[1.6, 1.8, 2.0, 2.2]
CLmax_landing=[1.6, 1.9, 2.2]
aspect_ratios=[A]
e=0.85
stall_speed_clean=110*0.5144
stall_speed_takeoff=140*0.5144
stall_speed_landing=140*0.5144
cruise_altitude=100*0*0.3048
high_altitude=10000*0.3048
cruise_speed=180*0.5144
prop_efficiency=0.82
hull_surface=400
L=30
rho_water=1000.0
kinematic_viscosity=1.002e-6

WP, TW, WS = main(
        plot_type=aircraft_type,
        CLmax_clean=CLmax_clean,
        CLmax_takeoff=CLmax_takeoff,
        CLmax_landing=CLmax_landing,
        aspect_ratios=aspect_ratios,
        Cd0=Cd0,
        e=e,
        k=k,
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

class_i = ClassI(
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
                reserve_fuel=reserve_fuel,
                k=k
            )

class_i.main()
MTOW = class_i.MTOW
MTOM = class_i.MTOM
print(f"MTOM {MTOM:=,.2f} kg")
print(f"MTOW {MTOW}")
print(f"WS {WS}")
S = MTOW / WS
print(f"S {S}")
b = np.sqrt(S * A)
print(f"b {b}")
c = S / b
print(f"c {c}")

h = h_b*b
print(f"h {h}")