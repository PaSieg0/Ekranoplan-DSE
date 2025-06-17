import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
import matplotlib.pyplot as plt
from utils import Data, MissionType, ISA, AircraftType

class WingLoading:
    def __init__(self,
                aircraft_data: Data,
                mission_type: MissionType
                 ) -> None:
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f'design{self.design_number}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type
        self.aircraft_type = AircraftType[self.aircraft_data.data['inputs']['aircraft_type']]
        self.CLmax_clean = np.array([self.aircraft_data.data['inputs']['CLmax_clean']])
        self.CLmax_takeoff = np.array([self.aircraft_data.data['inputs']['CLmax_takeoff']])
        self.CLmax_landing = np.array([self.aircraft_data.data['inputs']['CLmax_landing']])
        self.aspect_ratios = np.array([self.aircraft_data.data['inputs']['aspect_ratio']])
        self.Cd0 = self.aircraft_data.data['inputs']['Cd0']
        self.e = self.aircraft_data.data['inputs']['oswald_factor']
        self.k = self.aircraft_data.data['outputs'][self.mission_type.name.lower()]['k']
        self.n_engines = np.array([self.aircraft_data.data['inputs']['n_engines']])
        self.n_fuselages = self.aircraft_data.data['inputs']['n_fuselages']
        self.cruise_speed = self.aircraft_data.data['requirements']['cruise_speed']
        self.stall_speed_clean = self.aircraft_data.data['requirements']['stall_speed_clean']
        self.stall_speed_takeoff = self.aircraft_data.data['requirements']['stall_speed_takeoff']
        self.stall_speed_landing = self.aircraft_data.data['requirements']['stall_speed_landing']
        self.stall_speed_high = self.aircraft_data.data['requirements']['stall_speed_high']
        self.V_lof = 1.05*self.stall_speed_takeoff
        self.L = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        self.tail_length = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone']
        # self.r_float = self.aircraft_data.data['inputs']['r_float']
        self.cruise_altitude = self.aircraft_data.data['inputs']['cruise_altitude']
        self.high_altitude = self.aircraft_data.data['requirements']['high_altitude']
        self.climb_rate = self.aircraft_data.data['requirements']['climb_rate']
        self.isa_cruise = ISA(self.cruise_altitude)
        self.isa_high = ISA(self.high_altitude)
        self.WS = np.arange(1, 10000, 1)
        self.kinematic_viscosity = self.aircraft_data.data['kinematic_viscosity']
        self.rho_water = self.aircraft_data.data['rho_water']
        self.depth = self.aircraft_data.data['inputs']['depth']
        self.prop_efficiency = self.aircraft_data.data['inputs']['prop_efficiency']
        # self.CL_hydro = self.aircraft_data.data['inputs']['CL_hydro']
        self.upsweep = self.aircraft_data.data['inputs']['upsweep']
        self.d_fuselage = self.aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station2']
        self.hull_surface = self.aircraft_data.data['outputs']['fuselage_dimensions']['hull_surface']
        self.TW = None
        self.WP = None
        self.max_WS = None

    def stall_requirement(self):
        x = 0.5*self.isa_cruise.rho * self.stall_speed_clean**2 * self.CLmax_clean
        return x
    
    def stall_requirement_high(self):
        x = 0.5*self.isa_cruise.rho * self.stall_speed_high**2 * self.CLmax_clean
        return x

    def calculate_Re(self):
        return self.V_lof*self.L / self.kinematic_viscosity
         
    def calculate_Cd(self):
        self.Re = self.calculate_Re()
        self.aircraft_data.data['outputs']['general']['Re'] = self.Re
        Cd = 0.075 / (np.log10(self.Re) - 2)**2
        return Cd
    
    def solve_piecewise(self, A):
        f1 = lambda x: self.w_fuselage * x / 2
        f2 = lambda x: self.w_fuselage * self.t_fuselage / 2 + self.w_fuselage * (x - self.t_fuselage)
        f1(self.t_fuselage)
        f2(self.t_fuselage)
        if f1(self.t_fuselage) >= A:
            # Solve f1(x) = A for x
            x = 2 * A / self.w_fuselage
            return x
        elif f2(self.t_fuselage) < A:
            # Solve f2(x) = A for x
            x = (A - self.w_fuselage * self.t_fuselage / 2) / self.w_fuselage + self.t_fuselage
            return x
        else:
            raise ValueError("No solution found for the piecewise function.")



    def calculate_hull_surface(self):
        self.w_fuselage = self.aircraft_data.data['inputs']['w_fuselage']
        self.h_fuselage = self.aircraft_data.data['inputs']['h_fuselage']
        self.t_fuselage = self.aircraft_data.data['inputs']['structures']['fuselage']['t_fuselage']

        rho_water = self.aircraft_data.data['rho_water']
        V_disp = self.aircraft_data.data['outputs']['max']['MTOM'] / rho_water
        A_disp = V_disp / (self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] - self.aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone'])
        depth = self.solve_piecewise(A_disp)
        self.aircraft_data.data['outputs']['general']['resting_depth'] = depth
        L = np.sqrt(1 + ((self.w_fuselage/2) / self.t_fuselage)**2)
        A_hull = 2* (self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] - self.aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone']) * depth * L
        return A_hull
    
    def take_off_requirement(self):
        CL_takeoff = self.CLmax_takeoff/1.21
        Cd = self.calculate_Cd()
        self.hull_surface = self.aircraft_data.data['outputs']['fuselage_dimensions']['hull_surface']
        self.aircraft_data.data['outputs']['general']['Cd_water'] = Cd
        self.aircraft_data.data['outputs']['general']['hull_surface'] = self.hull_surface
        D = 0.5 * self.rho_water * (self.V_lof)**2 * Cd * self.hull_surface

        if self.aircraft_type == AircraftType.JET:
            self.aircraft_data.data['outputs']['general']['take_off_thrust'] = D
            self.aircraft_data.data['outputs']['general']['take_off_power'] = None
        elif self.aircraft_type == AircraftType.PROP or self.aircraft_type == AircraftType.MIXED:
            self.aircraft_data.data['outputs']['general']['take_off_power'] = D * self.V_lof / self.prop_efficiency
            self.aircraft_data.data['outputs']['general']['take_off_thrust'] = None
        elif self.aircraft_type == AircraftType.MIXED:
            self.aircraft_data.data['outputs']['general']['take_off_thrust'] = D
            self.aircraft_data.data['outputs']['general']['take_off_power'] = D * self.V_lof / self.prop_efficiency
        
        x = [CL*0.5*self.isa_cruise.rho * self.V_lof**2 for CL in CL_takeoff]
        return x

    
    def landing_requirement(self):
        V_land = self.stall_speed_landing * 1.3
        f = 1

        x = self.CLmax_landing * self.isa_cruise.rho * V_land**2 / (2*f)

        return x

    def cruise_requirement(self):
        x = self.WS.copy()
        
        if self.aircraft_type == AircraftType.PROP or self.aircraft_type == AircraftType.MIXED:
            y = [0.9/0.8 * self.prop_efficiency * (self.isa_cruise.rho / self.isa_cruise.rho0)**0.7 * ((self.Cd0*0.5*self.isa_cruise.rho*self.cruise_speed**3)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_cruise.rho*self.cruise_speed))**-1 for A in self.k**2*self.aspect_ratios]
        elif self.aircraft_type == AircraftType.JET:
            y = [0.8/0.9 * (self.isa_cruise.rho0 / self.isa_cruise.rho)**0.7 * ((self.Cd0*0.5*self.isa_cruise.rho*self.cruise_speed**2)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_cruise.rho*self.cruise_speed**2)) for A in self.k**2*self.aspect_ratios]

        return y
    
    def cruise_requirement_high(self):
        x = self.WS.copy()
        
        if self.aircraft_type == AircraftType.PROP:
            y = [0.9/0.8 * self.prop_efficiency * (self.isa_high.rho / self.isa_high.rho0)**0.7 * ((self.Cd0*0.5*self.isa_high.rho*self.cruise_speed**3)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_high.rho*self.cruise_speed))**-1 for A in self.aspect_ratios]
        elif self.aircraft_type == AircraftType.JET or self.aircraft_type == AircraftType.MIXED:
            y = [0.8/0.9 * (self.isa_high.rho0 / self.isa_high.rho)**0.7 * ((self.Cd0*0.5*self.isa_high.rho*self.cruise_speed**2)/(0.8*x) + 0.8*x/(np.pi*A*self.e*0.5*self.isa_high.rho*self.cruise_speed**2)) for A in self.aspect_ratios]

        return y

    def climb_rate_requirement(self):
        x = self.WS.copy()
        c = self.climb_rate
        Cd = 4*self.Cd0
        Cl = [np.sqrt(3*self.Cd0*np.pi*A*self.e) for A in self.k**2 * self.aspect_ratios]

        if self.aircraft_type == AircraftType.PROP:
            y = [self.prop_efficiency/(c+(np.sqrt(x)*np.sqrt(2/self.isa_cruise.rho))/(1.345*(A*self.e)**0.75/(self.Cd0**0.25))) for A in self.k**2 * self.aspect_ratios]
        elif self.aircraft_type == AircraftType.JET or self.aircraft_type == AircraftType.MIXED:
            y = [c/(np.sqrt(2*x/(self.isa_cruise.rho*CL))) + Cd/CL for CL in Cl]

        return y

    def climb_gradient_requirement(self):
        x = self.WS.copy()
        c_V = 0.032

        CLs = [np.sqrt(3*self.Cd0*np.pi*A*self.e) for A in self.k**2 * self.aspect_ratios]
        CD = 4*self.Cd0

        if self.aircraft_type == AircraftType.PROP:
            y = [self.prop_efficiency/(np.sqrt(x)*((c_V)+CD/CL)*np.sqrt(2/(self.isa_cruise.rho*CL))) for CL in CLs]

        elif self.aircraft_type == AircraftType.JET or self.aircraft_type == AircraftType.MIXED:
            y = [(c_V + 2*np.sqrt(self.Cd0/(np.pi*A*self.e))) for A in self.k**2 * self.aspect_ratios]
        return y
    
    def climb_gradient_requirement_OEI(self):
        x = self.WS.copy()
        c_V = 0.024

        CLs = [np.sqrt(3*self.Cd0*np.pi*A*self.e) for A in self.k**2 * self.aspect_ratios]
        CD = 4*self.Cd0

        if self.aircraft_type == AircraftType.PROP:
            y = [(n-1)/n*self.prop_efficiency/(np.sqrt(x)*((c_V+0.003*(n-2))+CD/CL)*np.sqrt(2/(self.isa_cruise.rho*CL))) for CL in CLs for n in self.n_engines]

        elif self.aircraft_type == AircraftType.JET or self.aircraft_type == AircraftType.MIXED:
            y = [n/(n-1)*(c_V+0.003*(n-2) + 2*np.sqrt(self.Cd0/(np.pi*A*self.e))) for A in self.k**2 * self.aspect_ratios for n in self.n_engines]

        return y

    def main(self) -> tuple:
        stall_req = self.stall_requirement()
        stall_req_high = self.stall_requirement_high()
        take_off_req = self.take_off_requirement()
        landing_req = self.landing_requirement()
        cruise_req = self.cruise_requirement()
        cruise_high_req = self.cruise_requirement_high()
        climb_rate_req = self.climb_rate_requirement()
        climb_gradient_req = self.climb_gradient_requirement()
        climb_gradient_req_OEI = self.climb_gradient_requirement_OEI()

        if self.aircraft_type == AircraftType.PROP:
            all_vertical_lines = [take_off_req, landing_req, stall_req, stall_req_high]
            self.max_WS = float('inf')
            for values in all_vertical_lines:
                maxx = np.min(values)
                if maxx < self.max_WS:
                    self.max_WS = maxx

            all_curves = [cruise_req, cruise_high_req, climb_rate_req, climb_gradient_req, climb_gradient_req_OEI]
            
            intersections = []

            for y_vals in all_curves:
                for curve in y_vals:
                    y_at_leftmost = np.interp(self.max_WS, self.WS, curve)
                    intersections.append(y_at_leftmost)

            if self.aircraft_type == AircraftType.JET:
                self.TW = max(intersections)
            elif self.aircraft_type == AircraftType.PROP:
                self.WP = min(intersections)

        else:
            all_vertical_lines = [take_off_req, landing_req, stall_req, stall_req_high]
            self.max_WS = float('inf')
            
            for values in all_vertical_lines:
                maxx = np.min(values)
                if maxx < self.max_WS:
                    self.max_WS = maxx

            all_curves_prop = [cruise_req]
            all_curves_jet = [cruise_high_req, climb_rate_req]
            intersections_prop = []
            intersections_jet = []

            for y_vals in all_curves_prop:
                for curve in y_vals:
                    y_at_leftmost = np.interp(self.max_WS, self.WS, curve)
                    intersections_prop.append(y_at_leftmost)
            for y_vals in all_curves_jet:
                for curve in y_vals:
                    y_at_leftmost = np.interp(self.max_WS, self.WS, curve)
                    intersections_jet.append(y_at_leftmost)

            intersections_jet.append(max([max(climb_gradient_req),max(climb_gradient_req_OEI)]))
            self.TW = max(intersections_jet)
            self.WP = max(intersections_prop)

            self.aircraft_data.save_design(self.design_file)
                

        return stall_req, stall_req_high, take_off_req, landing_req, cruise_req, cruise_high_req, climb_rate_req, climb_gradient_req, climb_gradient_req_OEI

def main(aircraft_data: Data, 
         mission_type: MissionType,
         PLOT_OUTPUT: bool=False):
    plot_type = AircraftType[aircraft_data.data['inputs']['aircraft_type']]
    prop = WingLoading(
        aircraft_data=aircraft_data,
        mission_type=mission_type,
    )

    jet = WingLoading(
        aircraft_data=aircraft_data,
        mission_type=mission_type,
    )

    mixed = WingLoading(
        aircraft_data=aircraft_data,
        mission_type=mission_type,
    )


    if plot_type == AircraftType.PROP:
        if PLOT_OUTPUT:
            __plot_prop(prop, True)
        prop.main()
        return 0.98*prop.WP, None, 0.98*prop.max_WS

    elif plot_type == AircraftType.JET:
        if PLOT_OUTPUT:
            __plot_jet(jet, True)
        jet.main()
        return None, 1.02*jet.TW, 0.98*jet.max_WS

    elif plot_type == AircraftType.MIXED:
        if PLOT_OUTPUT:
            __plot_mixed(prop, jet, True)
        mixed.main()


        return 0.98*mixed.WP, 1.02*mixed.TW, 0.98*mixed.max_WS

    


def __plot_prop(WL, PLOT_OUTPUT: bool=False):
    prop_stall, prop_stall_high, prop_take_off, prop_landing, prop_cruise, prop_cruise_high, prop_climb_rate, prop_climb_gradient, prop_climb_gradient_OEI = WL.main()
    fig, ax = plt.subplots(figsize=(10, 8))

    linestyles = ['-', '--', '-.', ':', (0, (1, 1)), (0, (3, 5, 1, 5, 1, 5))]


    for i, take_off in enumerate(prop_take_off):
        ax.axvline(x=take_off, label=f"Takeoff: CL={WL.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    for i, climb_rate in enumerate(prop_climb_rate):
        ax.plot(WL.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    for i, climb_gradient in enumerate(prop_climb_gradient):
        A_idx = i % len(WL.aspect_ratios)
        n_idx = i // len(WL.aspect_ratios)
        ax.plot(WL.WS, climb_gradient, label=f"Climb gradient: Aspect ratio={WL.aspect_ratios[A_idx]}\nEngine={WL.n_engines[n_idx]:.0f}", linestyle=linestyles[i], color='tab:green')
    for i, climb_gradient in enumerate(prop_climb_gradient_OEI):
        A_idx = i % len(WL.aspect_ratios)
        n_idx = i // len(WL.aspect_ratios)
        ax.plot(WL.WS, climb_gradient, label=f"Climb gradient OEI: Aspect ratio={WL.aspect_ratios[A_idx]}\nEngine={WL.n_engines[n_idx]:.0f}", linestyle=linestyles[i], color='tab:olive')
    for i, cruise in enumerate(prop_cruise):
        ax.plot(WL.WS, cruise, label=f"Cruise: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    for i, cruise in enumerate(prop_cruise_high):
        ax.plot(WL.WS, cruise, label=f"Cruise high: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    for i, landing in enumerate(prop_landing):
        ax.axvline(x=landing, label=f"Landing: CL={WL.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(prop_stall):
        ax.axvline(x=stall, label=f"Stall: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')
    for i, stall in enumerate(prop_stall_high):
        ax.axvline(x=stall, label=f"Stall high: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:brown')

    WP, WS = 0.95*WL.WP, 0.98*WL.max_WS
    print(WP, WS)
    ax.scatter(WS, WP, label=f"Design Point", color='red', marker='o', s = 20, zorder = 10)

    # Find the index in WL.WS where WS is reached or just below
    idx = np.searchsorted(WL.WS, WL.max_WS, side='right')
    plt.fill_between(WL.WS[:idx], 0, prop_cruise_high[0][:idx], color='lightgreen', alpha=0.3)
    #plt.suptitle(f"Wing Loading Requirements")
    plt.ylabel('W/P [N/W]', fontsize=14)
    plt.xlabel('W/S [N/m^2]', fontsize=14)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)
    ax.set_xlim(0, 8000)
    ax.set_ylim(0, 0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=12)
    plt.tight_layout()
    if PLOT_OUTPUT:
        plt.show()

def __plot_jet(WL, PLOT_OUTPUT: bool=False):
    jet_stall, jet_stall_high, jet_take_off, jet_landing, jet_cruise, jet_cruise_high, jet_climb_rate, jet_climb_gradient, jet_climb_gradient_OEI = WL.main()

    fig, ax = plt.subplots(figsize=(10, 8))

    linestyles = ['-', '--', '-.', ':', (0, (1, 1)), (0, (3, 5, 1, 5, 1, 5))]

    for i, take_off in enumerate(jet_take_off):
        ax.axvline(x=take_off, label=f"Takeoff: CL={WL.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    for i, climb_rate in enumerate(jet_climb_rate):
        ax.plot(WL.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    for i, climb_gradient in enumerate(jet_climb_gradient):
        A_idx = i % len(WL.aspect_ratios)
        n_idx = i // len(WL.aspect_ratios)
        ax.axhline(y=climb_gradient, label=f"Climb gradient: Aspect ratio={WL.aspect_ratios[A_idx]}\nEngine={WL.n_engines[n_idx]}", linestyle=linestyles[i], color='tab:green')
    for i, climb_gradient in enumerate(jet_climb_gradient_OEI):
        A_idx = i % len(WL.aspect_ratios)
        n_idx = i // len(WL.aspect_ratios)
        ax.axhline(y=climb_gradient, label=f"Climb gradient OEI: Aspect ratio={WL.aspect_ratios[A_idx]}\nEngine={WL.n_engines[n_idx]}", linestyle=linestyles[i], color='tab:olive')
    for i, cruise in enumerate(jet_cruise):
        ax.plot(WL.WS, cruise, label=f"Cruise: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    for i, cruise in enumerate(jet_cruise_high):
        ax.plot(WL.WS, cruise, label=f"Cruise high: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    for i, landing in enumerate(jet_landing):
        ax.axvline(x=landing, label=f"Landing: CL={WL.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(jet_stall):
        ax.axvline(x=stall, label=f"Stall: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')
    for i, stall in enumerate(jet_stall_high):
        ax.axvline(x=stall, label=f"Stall high: CL={WL.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:brown')

    plt.suptitle("Wing Loading Requirements")
    ax.set_xlim(0, 8000)
    ax.set_ylim(0, 0.6)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    if PLOT_OUTPUT:
        plt.show()


def __plot_mixed(WL_prop, WL_jet, PLOT_OUTPUT: bool=False):
    prop_stall, prop_stall_high, prop_take_off, prop_landing, prop_cruise, prop_cruise_high, prop_climb_rate, prop_climb_gradient, prop_climb_gradient_OEI = WL_prop.main()
    jet_stall, jet_stall_high, jet_take_off, jet_landing, jet_cruise, jet_cruise_high, jet_climb_rate, jet_climb_gradient, jet_climb_gradient_OEI = WL_jet.main()
    fig, ax = plt.subplots(2, 1, figsize=(10, 12))

    linestyles = ['-', '--', '-.', ':', (0, (1, 1)), (0, (3, 5, 1, 5, 1, 5))]


    for i, take_off in enumerate(prop_take_off):
        ax[0].axvline(x=take_off, label=f"Takeoff: CL={WL_prop.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    # for i, climb_rate in enumerate(prop_climb_rate):
    #     ax[0].plot(WL_prop.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL_prop.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    # for i, climb_gradient in enumerate(prop_climb_gradient):
    #     ax[0].plot(WL_prop.WS, climb_gradient, label=f"Climb gradient: Aspect ratio={WL.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:green')
    for i, cruise in enumerate(prop_cruise):
        ax[0].plot(WL_prop.WS, cruise, label=f"Cruise: Aspect ratio={WL_prop.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    # for i, cruise in enumerate(prop_cruise_high):
    #     ax[0].plot(WL_prop.WS, cruise, label=f"Cruise high: Aspect ratio={WL_prop.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    for i, landing in enumerate(prop_landing):
        ax[0].axvline(x=landing, label=f"Landing: CL={WL_prop.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(prop_stall):
        ax[0].axvline(x=stall, label=f"Stall: CL={WL_prop.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')
    for i, stall in enumerate(prop_stall_high):
        ax[0].axvline(x=stall, label=f"Stall high: CL={WL_prop.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:brown')

    
    # for i, take_off in enumerate(jet_take_off):
    #     ax[1].axvline(x=take_off, label=f"Takeoff: CL={WL_jet.CLmax_takeoff[i]}", linestyle=linestyles[i], color='tab:blue')
    for i, climb_rate in enumerate(jet_climb_rate):
        ax[1].plot(WL_jet.WS, climb_rate, label=f"Climb rate: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:orange')
    for i, climb_gradient in enumerate(jet_climb_gradient):
        A_idx = i % len(WL_jet.aspect_ratios)
        n_idx = i // len(WL_jet.aspect_ratios)
        ax[1].axhline(y=climb_gradient, label=f"Climb gradient: Aspect ratio={WL_jet.aspect_ratios[A_idx]}\nEngine={WL_jet.n_engines[n_idx]}", linestyle=linestyles[i], color='tab:green')
    for i, climb_gradient in enumerate(jet_climb_gradient_OEI):
        A_idx = i % len(WL_jet.aspect_ratios)
        n_idx = i // len(WL_jet.aspect_ratios)
        ax[1].axhline(y=climb_gradient, label=f"Climb gradient OEI: Aspect ratio={WL_jet.aspect_ratios[A_idx]}\nEngine={WL_jet.n_engines[n_idx]}", linestyle=linestyles[i], color='tab:olive')
    # for i, cruise in enumerate(jet_cruise):
    #     ax[1].plot(WL_jet.WS, cruise, label=f"Cruise: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:red')
    for i, cruise in enumerate(jet_cruise_high):
        ax[1].plot(WL_jet.WS, cruise, label=f"Cruise high: Aspect ratio={WL_jet.aspect_ratios[i]}", linestyle=linestyles[i], color='tab:purple')
    # for i, landing in enumerate(jet_landing):
    #     ax[1].axvline(x=landing, label=f"Landing: CL={WL_jet.CLmax_landing[i]}", linestyle=linestyles[i], color='tab:cyan')
    for i, stall in enumerate(jet_stall):
        ax[1].axvline(x=stall, label=f"Stall: CL={WL_jet.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:pink')
    for i, stall in enumerate(jet_stall_high):
        ax[1].axvline(x=stall, label=f"Stall high: CL={WL_jet.CLmax_clean[i]}", linestyle=linestyles[i], color='tab:brown')


    plt.suptitle("Wing Loading Requirements")
    ax[0].set_title("Propeller Aircraft")
    ax[0].set_xlim(0, 8000)
    ax[0].set_ylim(0, 0.6)
    ax[1].set_title("Jet Aircraft")
    ax[1].set_xlim(0, 8000)
    ax[1].set_ylim(0, 0.6)
    ax[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax[1].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    if PLOT_OUTPUT:
        plt.show()
    

if __name__ == "__main__":
    aircraft_data = Data("design3.json")
    WP, TW, WS = main(
        aircraft_data=aircraft_data,
        mission_type=MissionType.DESIGN,
        PLOT_OUTPUT=True
    )

    print(f"WP {WP}, TW {TW} , WS {WS}")