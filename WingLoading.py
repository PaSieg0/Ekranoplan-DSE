from enum import Enum, auto
from ISA_Class import ISA
import numpy as np
import matplotlib.pyplot as plt

class AircraftType(Enum):
    JET = auto()
    PROP = auto()
    MIXED = auto()

class WingLoading:
    def __init__(self,
                 aircraft_type: AircraftType,
                 CLmax_clean: list[float],
                 CLmax_takeoff: list[float],
                 CLmax_landing: list[float],
                 aspect_ratios: list[float],
                 Cd0: float,
                 e: float,
                 stall_speed_clean: float,
                 stall_speed_takeoff: float,
                 stall_speed_landing: float,
                 cruise_altitude: float,
                 high_altitude: float,
                 cruise_speed: float,
                 prop_efficiency: float,
                 hull_surface: float,
                 L: float,
                 rho_water: float,
                 kinematic_viscosity: float
                 ) -> None:
        self.aircraft_type = aircraft_type
        self.CLmax_clean = np.array(CLmax_clean)
        self.CLmax_takeoff = np.array(CLmax_takeoff)
        self.CLmax_landing = np.array(CLmax_landing)
        self.aspect_ratios = np.array(aspect_ratios)
        self.Cd0 = Cd0
        self.e = e
        self.stall_speed_clean = stall_speed_clean
        self.stall_speed_takeoff = stall_speed_takeoff
        self.stall_speed_landing = stall_speed_landing
        self.V_lof = 1.05*self.stall_speed_takeoff
        self.cruise_altitude = cruise_altitude
        self.high_altitude = high_altitude
        self.cruise_speed = cruise_speed
        self.isa_cruise = ISA(self.cruise_altitude)
        self.isa_high = ISA(self.high_altitude)
        self.prop_efficiency = prop_efficiency
        self.rho_water = rho_water
        self.hull_surface = hull_surface
        self.L = L
        self.kinematic_viscosity = kinematic_viscosity


        self.WS = np.arange(1, 5000, 1)

    def stall_requirement(self):
        x = 0.5*self.isa_cruise.rho * self.stall_speed_clean**2 * self.CLmax_clean
        return x
    
    def calculate_Cd(self):
        Re = self.V_lof*self.L / self.kinematic_viscosity
        Cd = 0.075 / (np.log10(Re) - 2)**2
        return Cd
    
    def take_off_requirement(self):
        CL_takeoff = self.CLmax_takeoff/1.21
        Cd = self.calculate_Cd()
        D = 0.5 * self.rho_water * (self.V_lof)**2 * Cd * self.hull_surface
        if self.aircraft_type == AircraftType.JET:
            print(f"Thrust shall be at least {D:=,.0f} N.")
        if self.aircraft_type == AircraftType.PROP:
            print(f"Power shall be at least {D*self.V_lof:=,.0f} W.")

        x = [CL*0.5*self.isa_cruise.rho * self.V_lof**2 for CL in CL_takeoff]
        return x

    
    def landing_requirement(self):
        V_land = self.stall_speed_landing * 1.3
        f = 0.8

        x = self.CLmax_landing * self.isa_cruise.rho * V_land**2 / (2*f)

        return x

    def cruise_requirement(self):
        x = self.WS.copy()
        
        if self.aircraft_type == AircraftType.PROP:
            y = [0.9/0.8 * self.prop_efficiency * (self.isa_cruise.rho * self.isa_cruise.rho0)**0.75 * ((self.Cd0*0.5*self.isa_cruise.rho*self.cruise_speed**3)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_cruise.rho*self.cruise_speed))**-1 for A in self.aspect_ratios]
        elif self.aircraft_type == AircraftType.JET:
            y = [0.8/0.9 * (self.isa_cruise.rho0 * self.isa_cruise.rho)**0.75 * ((self.Cd0*0.5*self.isa_cruise.rho*self.cruise_speed**2)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_cruise.rho*self.cruise_speed**2)) for A in self.aspect_ratios]

        return y
    
    def cruise_requirement_high(self):
        x = self.WS.copy()
        
        if self.aircraft_type == AircraftType.PROP:
            y = [0.9/0.8 * self.prop_efficiency * (self.isa_high.rho / self.isa_high.rho0)**0.75 * ((self.Cd0*0.5*self.isa_high.rho*self.cruise_speed**3)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_high.rho*self.cruise_speed))**-1 for A in self.aspect_ratios]
        elif self.aircraft_type == AircraftType.JET:
            y = [0.8/0.9 * (self.isa_high.rho0 / self.isa_high.rho)**0.75 * ((self.Cd0*0.5*self.isa_high.rho*self.cruise_speed**2)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_high.rho*self.cruise_speed**2)) for A in self.aspect_ratios]

        return y

    def climb_rate_requirement(self):
        x = self.WS.copy()
        c = 5.08

        Cd = 4*self.Cd0
        Cl = [np.sqrt(3*self.Cd0*np.pi*A*self.e) for A in self.aspect_ratios]

        if self.aircraft_type == AircraftType.PROP:
            y = [self.prop_efficiency/(c+(np.sqrt(x)*np.sqrt(2/self.isa_cruise.rho))/(1.345*(A*self.e)**0.75/(self.Cd0**0.25))) for A in self.aspect_ratios]
        elif self.aircraft_type == AircraftType.JET:
            y = [c/(np.sqrt(2*x/(self.isa_cruise.rho*CL))) + Cd/CL for CL in Cl]

        return y

    def climb_gradient_requirement(self):
        x = self.WS.copy()
        c_V = 0.083

        if self.aircraft_type == AircraftType.PROP:
            y = [self.prop_efficiency/(np.sqrt(x)*(c_V+4*self.Cd0/CL)*np.sqrt(2/(self.isa_cruise.rho*CL))) for CL in self.CLmax_clean]

        elif self.aircraft_type == AircraftType.JET:
            y = [c_V + 2*np.sqrt(self.Cd0/(np.pi*A*self.e)) for A in self.aspect_ratios]

        return y

    def main(self) -> tuple:
        stall_req = self.stall_requirement()
        take_off_req = self.take_off_requirement()
        landing_req = self.landing_requirement()
        cruise_req = self.cruise_requirement()
        cruise_high_req = self.cruise_requirement_high()
        climb_rate_req = self.climb_rate_requirement()
        climb_gradient_req = self.climb_gradient_requirement()

        return stall_req, take_off_req, landing_req, cruise_req, cruise_high_req, climb_rate_req, climb_gradient_req

def main(plot_type, CLmax_clean, CLmax_takeoff, CLmax_landing, aspect_ratios, Cd0, e, stall_speed_clean, stall_speed_takeoff, stall_speed_landing, cruise_altitude, high_altitude, cruise_speed, prop_efficiency, hull_surface, L, rho_water, kinematic_viscosity):
    prop = WingLoading(
        aircraft_type=AircraftType.PROP,
        CLmax_clean=CLmax_clean,
        CLmax_takeoff=CLmax_takeoff,
        CLmax_landing=CLmax_landing,
        aspect_ratios=aspect_ratios,
        Cd0=Cd0,
        e=e,
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
        kinematic_viscosity=kinematic_viscosity
    )

    jet = WingLoading(
        aircraft_type=AircraftType.JET,
        CLmax_clean=CLmax_clean,
        CLmax_takeoff=CLmax_takeoff,
        CLmax_landing=CLmax_landing,
        aspect_ratios=aspect_ratios,
        Cd0=Cd0,
        e=e,
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
        kinematic_viscosity=kinematic_viscosity
    )

    if plot_type == AircraftType.PROP:
        __plot_prop(prop)

    elif plot_type == AircraftType.JET:
        __plot_jet(jet)

    elif plot_type == AircraftType.MIXED:
        __plot_mixed(prop, jet)


def __plot_prop(WL):
    prop_stall, prop_take_off, prop_landing, prop_cruise, prop_cruise_high, prop_climb_rate, prop_climb_gradient = WL.main()
    fig, ax = plt.subplots(figsize=(10, 8))

    linestyles = ['-', '--', '-.', ':']

    for i, take_off in enumerate(prop_take_off):
        ax.axvline(x=take_off, label=f"Takeoff: CL={WL.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    for i, climb_rate in enumerate(prop_climb_rate):
        ax.plot(WL.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    for i, climb_gradient in enumerate(prop_climb_gradient):
        ax.plot(WL.WS, climb_gradient, label=f"Climb gradient: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:green')
    for i, cruise in enumerate(prop_cruise):
        ax.plot(WL.WS, cruise, label=f"Cruise: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    for i, cruise in enumerate(prop_cruise_high):
        ax.plot(WL.WS, cruise, label=f"Cruise high: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    for i, landing in enumerate(prop_landing):
        ax.axvline(x=landing, label=f"Landing: CL={WL.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(prop_stall):
        ax.axvline(x=stall, label=f"Stall: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')

    plt.suptitle("Wing Loading Requirements")
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 0.6)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()

def __plot_jet(WL):
    jet_stall, jet_take_off, jet_landing, jet_cruise, jet_cruise_high, jet_climb_rate, jet_climb_gradient = WL.main()

    fig, ax = plt.subplots(figsize=(10, 8))

    linestyles = ['-', '--', '-.', ':']

    for i, take_off in enumerate(jet_take_off):
        ax.axvline(x=take_off, label=f"Takeoff: CL={WL.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    for i, climb_rate in enumerate(jet_climb_rate):
        ax.plot(WL.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    for i, climb_gradient in enumerate(jet_climb_gradient):
        ax.axhline(y=climb_gradient, label=f"Climb gradient: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:green')
    for i, cruise in enumerate(jet_cruise):
        ax.plot(WL.WS, cruise, label=f"Cruise: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    for i, cruise in enumerate(jet_cruise_high):
        ax.plot(WL.WS, cruise, label=f"Cruise high: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    for i, landing in enumerate(jet_landing):
        ax.axvline(x=landing, label=f"Landing: CL={WL.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(jet_stall):
        ax.axvline(x=stall, label=f"Stall: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')

    plt.suptitle("Wing Loading Requirements")
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 0.6)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()


def __plot_mixed(WL_prop, WL_jet):
    prop_stall, prop_take_off, prop_landing, prop_cruise, prop_cruise_high, prop_climb_rate, prop_climb_gradient = WL_prop.main()
    jet_stall, jet_take_off, jet_landing, jet_cruise, jet_cruise_high, jet_climb_rate, jet_climb_gradient = WL_jet.main()
    fig, ax = plt.subplots(2, 1, figsize=(10, 12))

    linestyles = ['-', '--', '-.', ':']

    for i, take_off in enumerate(prop_take_off):
        ax[0].axvline(x=take_off, label=f"Takeoff: CL={WL_prop.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    # for i, climb_rate in enumerate(prop_climb_rate):
    #     ax[0].plot(WL_prop.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL_prop.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    # for i, climb_gradient in enumerate(prop_climb_gradient):
    #     ax[0].plot(WL_prop.WS, climb_gradient, label=f"Climb gradient: CL={WL_prop.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:green')
    for i, cruise in enumerate(prop_cruise):
        ax[0].plot(WL_prop.WS, cruise, label=f"Cruise: Aspect ratio={WL_prop.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    # for i, cruise in enumerate(prop_cruise_high):
    #     ax[0].plot(WL_prop.WS, cruise, label=f"Cruise high: Aspect ratio={WL_prop.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    for i, landing in enumerate(prop_landing):
        ax[0].axvline(x=landing, label=f"Landing: CL={WL_prop.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(prop_stall):
        ax[0].axvline(x=stall, label=f"Stall: CL={WL_prop.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')

    
    # for i, take_off in enumerate(jet_take_off):
    #     ax[1].axvline(x=take_off, label=f"Takeoff: CL={WL_jet.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    for i, climb_rate in enumerate(jet_climb_rate):
        ax[1].plot(WL_jet.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    for i, climb_gradient in enumerate(jet_climb_gradient):
        ax[1].axhline(y=climb_gradient, label=f"Climb gradient: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:green')
    # for i, cruise in enumerate(jet_cruise):
    #     ax[1].plot(WL_jet.WS, cruise, label=f"Cruise: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    for i, cruise in enumerate(jet_cruise_high):
        ax[1].plot(WL_jet.WS, cruise, label=f"Cruise: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    # for i, landing in enumerate(jet_landing):
    #     ax[1].axvline(x=landing, label=f"Landing: CL={WL_jet.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(jet_stall):
        ax[1].axvline(x=stall, label=f"Stall: CL={WL_jet.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')


    plt.suptitle("Wing Loading Requirements")
    ax[0].set_title("Propeller Aircraft")
    ax[0].set_xlim(0, 3000)
    ax[0].set_ylim(0, 0.6)
    ax[1].set_title("Jet Aircraft")
    ax[1].set_xlim(0, 3000)
    ax[1].set_ylim(0, 0.6)
    ax[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax[1].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()
    

if __name__ == "__main__":
    CLmax_clean=[1.5, 1.6, 1.7]
    CLmax_takeoff=[1.6, 1.8, 2.0, 2.2]
    CLmax_landing=[1.6, 1.9, 2.2]
    aspect_ratios=[6, 7, 8]
    Cd0=0.02
    e=0.85
    stall_speed_clean=100*0.5144
    stall_speed_takeoff=110*0.5144
    stall_speed_landing=120*0.5144
    cruise_altitude=100*0*0.3148
    high_altitude=10000*0.3148
    cruise_speed=180*0.5144
    prop_efficiency=0.8
    hull_surface=400
    L=30
    rho_water=1000.0
    kinematic_viscosity=1.002e-6

    main(plot_type=AircraftType.MIXED,
        CLmax_clean=CLmax_clean,
         CLmax_takeoff=CLmax_takeoff,
         CLmax_landing=CLmax_landing,
         aspect_ratios=aspect_ratios,
         Cd0=Cd0,
         e=e,
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
         kinematic_viscosity=kinematic_viscosity
    )
    

        