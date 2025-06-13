import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import *
import numpy as np
from Class_II.Modified_CD0 import Cd0Estimation

class ClassII:
    def __init__(self, aircraft_data: Data) -> None:
        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"

        cdest = Cd0Estimation(
        aircraft_data=aircraft_data,
        mission_type=MissionType.DESIGN,
        class_ii_OEW=aircraft_data.data['outputs']['max']['OEW']
        )

        self.fudge_factor = 1.25 # fudge factor for weight estimation for flying boat


        # TODO: CHECK ALL UNITS AND VALUES AND LINK EVERYTHING TO AIRCRAFT DATA
        self.A = self.aircraft_data.data['outputs']['wing_design']['aspect_ratio'] # wing aspect ratio
        self.A_h = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['aspect_ratio'] # horizontal tail aspect ratio
        self.A_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['aspect_ratio'] # vertical tail aspect ratio
        self.B_h = m2ft(aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']) # horizontal tail span
        self.B_w = m2ft(aircraft_data.data['outputs']['wing_design']['b']) # wing span
        # self.D = m2ft(self.aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station2']) # fuselage structural depth
        # self.D_e = m2ft(5.3) # engine diameter
        # self.F_w = m2ft(aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']) # fuselage width at horizontal tail
        # self.H_t = m2ft(self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height']) - m2ft(self.aircraft_data.data['outputs']['fuselage_dimensions']['d_fuselage_equivalent_station3']) # horizontal tail height
        # self.H_t_H_v = 1
        self.I_y = kgmsq2lbsftsq(10000) # Yawing moment of interia
        # self.K_cb = 1 # landging gear so 0???
        # self.K_d = # Duct constant from figure
        # self.K_door = 1.12 # door constant from Raymer
        # self.K_dw = 1 # door constant from Raymer for delta wing
        # self.K_dwf = 1 # door constant from Raymer for delta wing
        # self.K_Lg = 1 # landing gear constant from Raymer maybe 0???
        # self.K_mc = 1.45 # mission completed after failure
        # self.K_mp = 1 # landing gear constant from Raymer maybe 0???
        self.K_ng = 1.017 # Pylon mounted nacelle
        # self.K_np = 1 # landing gear constant from Raymer maybe 0???
        # self.K_p = 1.4 # For propeller
        self.K_r = 1.133 # For reciprocating engine
        # self.K_rht = 1.047 # For rolling tail                                       # Check this
        self.K_tp = 0.793 # For turboprop
        # self.K_tpg = 1 # landing gear constant from Raymer maybe 0???
        # self.K_tr = 1 # For jet with thrust reverser
        # self.K_uht = 1.143 # For all moving horizontal tail                         # Check this
        # self.K_vg = 1 # For variable geometry
        # self.K_vs = 1 # Variable sweep
        # self.K_vsh = 1 # Variable sweep
        # self.taper_ratio = self.aircraft_data.data['outputs']['wing_design']['taper_ratio'] # wing taper ratio
        self.L_f = m2ft(self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']) # total fuselage length
        # self.sweep_c_4 = self.aircraft_data.data['outputs']['wing_design']['sweep_c_4'] # wing sweep at 25% MAC
        # self.sweep_c_4_ht = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['sweep_c_4'] # horizontal tail sweep at 25% MAC
        # self.sweep_c_4_vt = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep_c_4'] # vertical tail sweep at 25% MAC
        # self.K_ws = 0.75*((1 + 2 * self.taper_ratio) / (1 + self.taper_ratio)) * self.B_w * np.tan(deg2rad(self.sweep_c_4)) / self.L_f # fuselage constant
        # self.L = m2ft(aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']) # structural length
        self.L_a = m2ft(2*self.aircraft_data.data['outputs']['wing_design']['b'] + self.aircraft_data.data['outputs']['wing_design']['X_LE'] + 2*self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']) # electrical routing distance, generators to avionics to cockpit
        # self.L_d = # duct length
        # self.L_m = 0 # landing gear
        # self.L_n = 0 # # landing gear
        # self.L_s = # single duct length from figure
        # self.L_sh = # length of engine shroud
        # self.L_t = m2ft(self.aircraft_data.data['outputs']['fuselage_dimensions']['l_tailcone']) # tail cone length, wing c/4 to tail c/4
        # self.K_y = 0.3*self.L_t # Radius of gyration for pitching axis
        # self.K_z = self.L_t # Radius of gyration for yawing axis
        # self.L_tp = # length of tailpipe
        # self.M = ISA(self.aircraft_data.data['inputs']['cruise_altitude']).Mach(self.aircraft_data.data['requirements']['cruise_speed']) # Mach
        self.N_c = 5 # number of crew
        # self.N_ci = 2 # number of pilots
        self.N_en = self.aircraft_data.data['inputs']['n_engines'] # number of engines
        self.L_ec = self.N_en*m2ft(self.aircraft_data.data['outputs']['wing_design']['X_LE']*0.9) # length from engine front to cockpit, total if multiengine
        self.N_f = 3 # number of actions performed by control
        self.N_gen = 2 # number of generators
        # self.N_l = # ultimate landing load factor
        self.N_Lt = m2ft(4.2) # Nacelle length
        self.N_m = 2 # number of mechanical functions
        # self.N_mss = 0 # Number of main gear struts
        # self.N_mw = 0 # number of main wheels
        # self.N_nw = 0 # number of nose wheels
        self.N_p = 5 # number of people
        # self.N_s = 3 # number of flight control systems
        self.N_t = 2 # number of fuel tanks
        # self.N_u = 8 # Number of hydraulic utility functions
        self.N_w = m2ft(1.3) # nacelle width
        self.N_z = 1.5*self.aircraft_data.data['outputs']['general']['nmax'] # ultimate load factor
        # self.q = Pa2lbfpftsq(0.5*ISA(self.aircraft_data.data['inputs']['cruise_altitude']).rho * self.aircraft_data.data['requirements']['cruise_speed']**2) # dynamic pressure
        self.R_kva = 50 # system electrical rating based on typical values Raymer
        self.S_c = msq2ftsq(7 * self.aircraft_data.data['outputs']['fuselage_dimensions']['cargo_length']) # cargo floor surface area
        self.S_cs = msq2ftsq(self.aircraft_data.data['outputs']['control_surfaces']['aileron']['area_single']*2 + self.aircraft_data.data['outputs']['control_surfaces']['elevator']['area']*2 + self.aircraft_data.data['outputs']['control_surfaces']['rudder']['area']*2) # total control surface area
        self.S_csw = 2*msq2ftsq(self.aircraft_data.data['outputs']['control_surfaces']['aileron']['area_single']) # control surface area wing mounted 
        self.S_e = 2*msq2ftsq(self.aircraft_data.data['outputs']['control_surfaces']['aileron']['area_single']) # elevator area
        self.S_f = msq2ftsq(cdest.fuselage_wet())
        # self.S_fw = # firewall surface area
        # self.S_ht = msq2ftsq(self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']) # horizontal tail surface area
        self.S_n = msq2ftsq(cdest.nacelle_wet()) # nacelle wetted area
        # self.S_r = # rudder area
        # self.S_vt = msq2ftsq(self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S']) # vertical tail surface area
        # self.S_w = msq2ftsq(self.aircraft_data.data['outputs']['wing_design']['S']) # wing surface area
        # self.SFC = kgpJ2lbsphrphp(self.aircraft_data.data['inputs']['prop_consumption']) # specific fuel consumption
        # self.T = # total engine thrust
        # self.T_e = # engine thrust
        # self.t_c_root = 0.1426 # root chord thickness
        self.V_p = 0 # self-sealing tanks volume
        self.V_pr = 0 # volume of pressurized section
        self.V_t = L2gal(self.aircraft_data.data['outputs']['max']['max_fuel_L']) # total fuel volume
        self.V_i = self.V_t # integral tanks volume
        # self.W = # fuselage structural width
        self.W_c = kg2lbs(100000) # maximum weight of cargo
        self.W_dg = kg2lbs(self.aircraft_data.data['outputs']['max']['MTOM']) # design gross weight
        self.W_ec = kg2lbs(self.aircraft_data.data['inputs']['engine']['engine_weight']+100) # weight of engine and contents
        self.W_en = kg2lbs(self.aircraft_data.data['inputs']['engine']['engine_weight']) # weight of engine
        # self.W_fw = # weight of fuel in wing
        # self.W_l = # landing gross weight
        # self.W_press = 11.9 # weight penalty due to pressurization
        self.W_uav = 1400 # uninstalled avionics weight
        self.W_APU_uninstalled = 1336 # APU weight uninstalled
        self.W_emergency_APU = kg2lbs(300)



    def wing_weight(self) -> float:
        # W_wing_lbs = 0.0051 * (self.W_dg*self.N_z)**0.557 * self.S_w**0.649 * self.A**0.5 * self.t_c_root**-0.4 * (1+self.taper_ratio)**0.1 * np.cos(deg2rad(self.sweep_c_4))**-1.0 * self.S_csw**0.1
        # return lbs2kg(W_wing_lbs)*9.81
        return self.aircraft_data.data['outputs']['component_weights']['wing']

    def horizontal_tail(self) -> float:
        # W_horizontal_tail_lbs = 0.0379 * self.K_uht * (1 + self.F_w/self.B_h)**-0.25 * self.W_dg**0.639 * self.N_z**0.10 * self.S_ht**0.75 * self.L_t**-1.0 * self.K_y**0.704 * np.cos(deg2rad(self.sweep_c_4_ht))**-1.0 * self.A_h**0.166 * (1 + self.S_e/self.S_ht)**0.1
        # return lbs2kg(W_horizontal_tail_lbs)*9.81
        return self.aircraft_data.data['outputs']['component_weights']['horizontal_tail']

    def vertical_tail(self) -> float:
        # W_vertical_tail_lbs = 0.0026 * (1 + self.H_t_H_v)**0.225 * self.W_dg**0.556 * self.N_z**0.536 * self.L_t**-0.5 * self.S_vt**0.5 * self.K_z**0.875 * np.cos(deg2rad(self.sweep_c_4_vt))**-1.0 * self.A_v**0.35 * self.t_c_root**-0.5
        # return lbs2kg(W_vertical_tail_lbs)*9.81
        return self.aircraft_data.data['outputs']['component_weights']['vertical_tail']

    def fuselage(self) -> float:
        # W_fuselage_lbs = 0.3280 * self.K_door * self.K_Lg * (self.W_dg*self.N_z)**0.5 * self.L**0.25 * self.S_f**0.302 * (1 + self.K_ws)**0.4 * (self.L/self.D)**0.10
        # W_fuselage_lbs *= self.fudge_factor
        # return lbs2kg(W_fuselage_lbs)*9.81
        return self.aircraft_data.data['outputs']['component_weights']['fuselage']
    
    # def epoxy_fuselage(self) -> float:
    #     return self.aircraft_data.data['outputs']['component_weights']['epoxy_fuselage']
    
    # def epoxy_wing(self) -> float:
    #     return self.aircraft_data.data['outputs']['component_weights']['epoxy_wing']
    
    # def epoxy_vertical_tail(self) -> float:
    #     return self.aircraft_data.data['outputs']['component_weights']['epoxy_vertical']
    
    # def epoxy_horizontal_tail(self) -> float:
    #     return self.aircraft_data.data['outputs']['component_weights']['epoxy_horizontal']

    def main_landing_gear(self):
        return 0

    def nose_landing_gear(self):
        return 0
    
    def nacelle_group(self):
        W_nacelle_group_lbs = 0.6724 * self.K_ng * self.N_Lt**0.10 * self.N_w**0.294 * self.N_z**0.119 * self.W_ec**0.611 * self.N_en**0.984 * self.S_n**0.224
        return lbs2kg(W_nacelle_group_lbs)*9.81
    
    def engine(self):
        W_engine_lbs = 2.575 * self.W_en**0.922 * self.N_en
        return lbs2kg(W_engine_lbs)*9.81
    
    def engine_controls(self):
        W_engine_controls_lbs = 5.0*self.N_en + 0.80*self.L_ec
        return lbs2kg(W_engine_controls_lbs)*9.81
    
    def starter_pneumatic(self):
        W_starter_pneumatic_lbs = 49.19*(self.N_en*self.W_en/1000)**0.541
        return lbs2kg(W_starter_pneumatic_lbs)*9.81
    
    def fuel_system(self):
        W_fuel_system_lbs = 2.405*self.V_t**0.606*(1 + self.V_i/self.V_t)**-1.0 * (1 + self.V_p/self.V_t) * self.N_t**0.5
        return lbs2kg(W_fuel_system_lbs)*9.81
    
    def flight_control(self):
        W_flight_control_system_lbs = 145.9 * self.N_f**0.554 * (1 + self.N_m/self.N_f)**-1.0 * self.S_cs**0.20 * (self.I_y*10**-6)**0.07
        return lbs2kg(W_flight_control_system_lbs)*9.81
    
    def APU_installed(self):
        W_APU_installed_lbs = 1.5*self.W_APU_uninstalled
        return lbs2kg(W_APU_installed_lbs)*9.81
    
    def Emergency_APU(self):
        W_Emergency_APU_lbs = 1.5*self.W_emergency_APU
        return lbs2kg(W_Emergency_APU_lbs)*9.81
    
    def instruments(self):
        W_instruments_lbs = 4.509 * self.K_r * self.K_tp * self.N_c**0.541 * self.N_en * (self.N_f + self.B_w)**0.5
        return lbs2kg(W_instruments_lbs)*9.81
    
    def hydraulics(self):
        W_hydraulic_system_lbs = 0.2673 * self.N_f * (self.L_f + self.B_w)**0.937
        return lbs2kg(W_hydraulic_system_lbs)*9.81
    
    def electrical(self):
        W_electrical_system_lbs = 7.291 * self.R_kva**0.782 * self.L_a**0.346 * self.N_gen**0.10
        return lbs2kg(W_electrical_system_lbs)*9.81
    
    def avionics(self):
        W_avionics_lbs = 1.73 * self.W_uav**0.983
        return lbs2kg(W_avionics_lbs)*9.81
    
    def furnishings(self):
        W_furnishings_lbs = 0.0577 * self.N_c**0.1 * self.W_c ** 0.393 * self.S_f ** 0.75
        return lbs2kg(W_furnishings_lbs)*9.81
    
    def air_conditioning(self):
        W_air_conditioning_lbs = 62.36 * self.N_p**0.25 * (self.V_pr/1000)**0.604 * self.W_uav**0.10
        return lbs2kg(W_air_conditioning_lbs)*9.81
    
    def anti_ice(self):
        W_anti_ice_lbs = 0.002 * self.W_dg
        return lbs2kg(W_anti_ice_lbs)*9.81
    
    def handling_gear(self):
        W_handling_gear_lbs = 3*1e-4 * self.W_dg
        return lbs2kg(W_handling_gear_lbs)*9.81
    
    def military_cargo_handling_system(self):
        W_military_cargo_handling_system_lbs = 2.4 * self.S_c
        return lbs2kg(W_military_cargo_handling_system_lbs)*9.81
    
    def door(self) -> float:
        return 3100 *9.81    # https://aviator.aero/press/stelia-aerospace-delivers-the-first-belugaxl-cargo-door/
    
    def anchor(self):
        return 970*9.81 + 2500*9.81
    
    def hull(self):
        W_hull_lbs = 0.12*self.W_dg   # https://www.icas.org/icas_archive/ICAS2012/PAPERS/198.PDF
        return lbs2kg(W_hull_lbs)*9.81
    
    def floater(self):
        W_floater_lbs = 6/8 * (0.0365*self.W_dg + 43.5)   # https://www.icas.org/icas_archive/ICAS2012/PAPERS/198.PDF
        return lbs2kg(W_floater_lbs)*9.81
    
    def floater_endplate(self):
        W_floater_lbs = 2/8 * (0.0365*self.W_dg + 43.5)   # https://www.icas.org/icas_archive/ICAS2012/PAPERS/198.PDF
        return lbs2kg(W_floater_lbs)*9.81
                         
    
    def main(self):
        self.W_wing = self.wing_weight()
        self.W_horizontal_tail = self.horizontal_tail()
        self.W_vertical_tail = self.vertical_tail()
        self.W_fuselage = self.fuselage()
        self.W_nacelle_group = self.nacelle_group()
        self.W_engine_controls = self.engine_controls()
        self.W_engine = self.engine()
        self.W_starter_pneumatic = self.starter_pneumatic()
        self.W_fuel_system = self.fuel_system()
        self.W_flight_control = self.flight_control()
        self.W_APU_installed = self.APU_installed()
        self.W_emergency_APU_installed = self.Emergency_APU()
        self.W_instruments = self.instruments()
        self.W_hydraulics = self.hydraulics()
        self.W_electrical = self.electrical()
        self.W_avionics = self.avionics()
        self.W_furnishings = self.furnishings()
        self.W_air_conditioning = self.air_conditioning()
        self.W_anti_ice = self.anti_ice()
        self.W_handling_gear = self.handling_gear()
        self.W_military_cargo_handling_system = self.military_cargo_handling_system()
        self.W_door = self.door()
        self.W_anchor = self.anchor()
        self.W_floater = self.floater()
        self.W_floater_endplate = self.floater_endplate()
        self.W_hull = self.hull()
        # self.W_epoxy = self.epoxy_fuselage() + self.epoxy_wing() + self.epoxy_vertical_tail() + self.epoxy_horizontal_tail()

        self.perc_wing = self.W_wing / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_horizontal_tail = self.W_horizontal_tail / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_vertical_tail = self.W_vertical_tail / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_fuselage = self.W_fuselage / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_nacelle_group = self.W_nacelle_group / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_engine_controls = self.W_engine_controls / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_engine = self.W_engine / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_starter_pneumatic = self.W_starter_pneumatic / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_fuel_system = self.W_fuel_system / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_flight_control = self.W_flight_control / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_APU_installed = self.W_APU_installed / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_emergency_APU_installed = self.W_emergency_APU_installed / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_instruments = self.W_instruments / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_hydraulics = self.W_hydraulics / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_electrical = self.W_electrical / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_avionics = self.W_avionics / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_furnishings = self.W_furnishings / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_air_conditioning = self.W_air_conditioning / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_anti_ice = self.W_anti_ice / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_handling_gear = self.W_handling_gear / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_military_cargo_handling_system = self.W_military_cargo_handling_system / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_door = self.W_door / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_anchor = self.W_anchor / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_floater = self.W_floater / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_floater_endplate = self.W_floater_endplate / (self.aircraft_data.data['outputs']['max']['OEW']) * 100
        self.perc_hull = self.W_hull / (self.aircraft_data.data['outputs']['max']['OEW']) * 100

        self.OEW = (
            self.W_wing + 
            self.W_horizontal_tail + 
            self.W_vertical_tail + 
            self.W_fuselage +
            self.W_nacelle_group + 
            self.W_engine_controls + 
            self.W_engine +
            self.W_starter_pneumatic +
            self.W_fuel_system + 
            self.W_flight_control + 
            self.W_APU_installed +
            self.W_emergency_APU_installed +
            self.W_instruments + 
            self.W_hydraulics + 
            self.W_electrical + 
            self.W_avionics +
            self.W_furnishings + 
            self.W_air_conditioning + 
            self.W_anti_ice +
            self.W_handling_gear + 
            self.W_military_cargo_handling_system +
            self.W_door +
            self.W_anchor +
            self.W_floater +
            self.W_floater_endplate
            # self.W_epoxy
            # self.W_hull
        )

        self.perc = (
            self.perc_wing +
            self.perc_horizontal_tail +
            self.perc_vertical_tail +
            self.perc_fuselage +
            self.perc_nacelle_group +
            self.perc_engine_controls +
            self.perc_engine +
            self.perc_starter_pneumatic +
            self.perc_fuel_system +
            self.perc_flight_control +
            self.perc_APU_installed +
            self.perc_emergency_APU_installed +
            self.perc_instruments +
            self.perc_hydraulics +
            self.perc_electrical +
            self.perc_avionics +
            self.perc_furnishings +
            self.perc_air_conditioning +
            self.perc_anti_ice +
            self.perc_handling_gear +
            self.perc_military_cargo_handling_system +
            self.perc_door +
            self.perc_anchor +
            self.perc_floater +
            self.perc_floater_endplate
            # self.perc_hull
        )
        
        self.weights_dict = {
            'Wing': self.W_wing,
            'Horizontal tail': self.W_horizontal_tail,
            'Vertical tail': self.W_vertical_tail,
            'Fuselage': self.W_fuselage,
            'Nacelle group': self.W_nacelle_group,
            'Engine controls': self.W_engine_controls,
            'Engine': self.W_engine,
            'Starter pneumatic': self.W_starter_pneumatic,
            'Fuel system': self.W_fuel_system,
            'Flight control': self.W_flight_control,
            'APU installed': self.W_APU_installed,
            'Emergency APU installed': self.W_emergency_APU_installed,
            'Instruments': self.W_instruments,
            'Hydraulics': self.W_hydraulics,
            'Electrical': self.W_electrical,
            'Avionics': self.W_avionics,
            'Furnishings': self.W_furnishings,
            'Air conditioning': self.W_air_conditioning,
            'Anti-ice': self.W_anti_ice,
            'Handling gear': self.W_handling_gear,
            'Military cargo handling system': self.W_military_cargo_handling_system,
            'Door': self.W_door,
            'Anchor': self.W_anchor,
            'Floater': self.W_floater,
            'Floater endplate': self.W_floater_endplate,
            # 'Hull': self.W_hull,
            'Total OEW': self.OEW
        }
        self.perc_dict = {
            'Wing': self.perc_wing,
            'Horizontal tail': self.perc_horizontal_tail,
            'Vertical tail': self.perc_vertical_tail,
            'Fuselage': self.perc_fuselage,
            'Nacelle group': self.perc_nacelle_group,
            'Engine controls': self.perc_engine_controls,
            'Engine': self.perc_engine,
            'Starter pneumatic': self.perc_starter_pneumatic,
            'Fuel system': self.perc_fuel_system,
            'Flight control': self.perc_flight_control,
            'APU installed': self.perc_APU_installed,
            'Emergency APU installed': self.perc_emergency_APU_installed,
            'Instruments': self.perc_instruments,
            'Hydraulics': self.perc_hydraulics,
            'Electrical': self.perc_electrical,
            'Avionics': self.perc_avionics,
            'Furnishings': self.perc_furnishings,
            'Air conditioning': self.perc_air_conditioning,
            'Anti-ice': self.perc_anti_ice,
            'Handling gear': self.perc_handling_gear,
            'Military cargo handling system': self.perc_military_cargo_handling_system,
            'Door': self.perc_door,
            'Anchor': self.perc_anchor,
            'Floater': self.perc_floater,
            'Floater endplate': self.perc_floater_endplate,
            # 'Hull': self.perc_hull,
            'Total OEW': self.perc
        }

        self.update_parameters()

    def update_parameters(self):
        self.aircraft_data.data['outputs']['component_weights']['air_conditioning'] = self.W_air_conditioning
        self.aircraft_data.data['outputs']['component_weights']['anti_ice'] = self.W_anti_ice
        self.aircraft_data.data['outputs']['component_weights']['apu_installed'] = self.W_APU_installed
        self.aircraft_data.data['outputs']['component_weights']['avionics'] = self.W_avionics
        self.aircraft_data.data['outputs']['component_weights']['electrical'] = self.W_electrical
        self.aircraft_data.data['outputs']['component_weights']['furnishings'] = self.W_furnishings
        self.aircraft_data.data['outputs']['component_weights']['fuselage'] = self.W_fuselage
        self.aircraft_data.data['outputs']['component_weights']['handling_gear'] = self.W_handling_gear
        self.aircraft_data.data['outputs']['component_weights']['horizontal_tail'] = self.W_horizontal_tail
        self.aircraft_data.data['outputs']['component_weights']['instruments'] = self.W_instruments
        self.aircraft_data.data['outputs']['component_weights']['nacelle_group'] = self.W_nacelle_group
        self.aircraft_data.data['outputs']['component_weights']['starter_pneumatic'] = self.W_starter_pneumatic
        self.aircraft_data.data['outputs']['component_weights']['vertical_tail'] = self.W_vertical_tail
        self.aircraft_data.data['outputs']['component_weights']['wing'] = self.W_wing
        self.aircraft_data.data['outputs']['component_weights']['engine'] = self.W_engine
        self.aircraft_data.data['outputs']['component_weights']['engine_controls'] = self.W_engine_controls
        self.aircraft_data.data['outputs']['component_weights']['fuel_system'] = self.W_fuel_system
        self.aircraft_data.data['outputs']['component_weights']['flight_control'] = self.W_flight_control
        self.aircraft_data.data['outputs']['component_weights']['military_cargo_handling_system'] = self.W_military_cargo_handling_system
        self.aircraft_data.data['outputs']['component_weights']['door'] = self.W_door
        self.aircraft_data.data['outputs']['component_weights']['anchor'] = self.W_anchor
        self.aircraft_data.data['outputs']['component_weights']['floater'] = self.W_floater
        self.aircraft_data.data['outputs']['component_weights']['floater_endplate'] = self.W_floater_endplate
        self.aircraft_data.data['outputs']['component_weights']['total_OEW'] = self.OEW

        self.aircraft_data.save_design(self.design_file)
        
    def plot_pie_chart(self):
        """
        Plot a pie chart of component weights in actual kg, including crew, fuel, and payload from the data.
        Components under 1%% are grouped into 'Miscellaneous'.
        The chart is sorted by descending weight, with the largest at the top and moving clockwise.
        Values are displayed outside the pie chart, with lines to avoid overlap.
        """
        import matplotlib.pyplot as plt
        import numpy as np
        # Get component weights (kg)
        weights = dict(self.weights_dict)  # Copy to avoid mutating original
        weights.pop('Total OEW', None)
        # Get crew, fuel, and payload from self.aircraft_data.data
        total_crew_weight = self.aircraft_data.data['requirements']['design_crew']*9.81
        fuel_weight = self.aircraft_data.data['outputs']['max']['total_fuel']
        payload_weight = self.aircraft_data.data['requirements']['design_payload']*9.81
        weights['Crew'] = total_crew_weight
        weights['Fuel'] = fuel_weight
        weights['Payload'] = payload_weight
        # Remove zero or negative weights
        weights = {k: v for k, v in weights.items() if v > 0}
        total = sum(weights.values())
        # Calculate percentages and sort
        items = sorted(weights.items(), key=lambda x: x[1], reverse=True)
        major_labels = []
        major_values = []
        misc_value = 0
        for label, value in items:
            perc = value / total * 100
            if perc < 1.75:
                misc_value += value
            else:
                major_labels.append(label)
                major_values.append(value)
        if misc_value > 0:
            major_labels.append('Miscellaneous')
            major_values.append(misc_value)
        # The largest slice starts at the top (90 deg), then moves clockwise
        def kg_fmt(x):
            if x >= 1000:
                return f'{x/9.81/1000:.1f} tonnes'
            else:
                return f'{x/9.81:.0f} kg'
        fig, ax = plt.subplots(figsize=(12, 12))
        wedges, texts = ax.pie(
            major_values,
            labels=None,  # We'll add labels manually
            startangle=90,
            counterclock=False,
            textprops={'fontsize': 12}
        )
        # Place labels and values outside, with lines
        for i, w in enumerate(wedges):
            ang = (w.theta2 + w.theta1) / 2.
            x = w.r * 1.25 * np.cos(np.deg2rad(ang))
            y = w.r * 1.25 * np.sin(np.deg2rad(ang))
            # Draw a line from the wedge to the label
            x0 = w.r * np.cos(np.deg2rad(ang))
            y0 = w.r * np.sin(np.deg2rad(ang))
            ax.plot([x0, x], [y0, y], color='gray', lw=1)
            # Place the label and value, offset to avoid overlap
            ha = 'left' if x > 0 else 'right'
            ax.text(x, y, f"{major_labels[i]}\n{kg_fmt(major_values[i])}", ha=ha, va='center', fontsize=12, fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.8))
        ax.set_title('Aircraft Component Mass Breakdown', fontsize=16)
        plt.tight_layout()
        plt.show()
        

if __name__ == "__main__":
    aircraft_data = Data("design3.json")
    class_ii = ClassII(aircraft_data)

    class_ii.main()
    print("\nComponent weights and percentages (ascending by weight):")
    print("_________________________________________________________")
    for name, weight in sorted(class_ii.weights_dict.items(), key=lambda x: x[1]):
        perc = class_ii.perc_dict[name]
        if name == 'Total OEW':
            print("_________________________________________________________")
        print(f"{name:<30} {weight/9.81:>10,.0f} kg   {perc:>7.2f} %")

    print(f"{'OEW from Class I':<30} {class_ii.aircraft_data.data['outputs']['max']['OEW']/9.81:>10,.0f} kg")

    class_ii.plot_pie_chart()