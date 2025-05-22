import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import *
import numpy as np

class ClassII:
    def __init__(self, aircraft_data: Data) -> None:

        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"designs{self.design_number}.json"


        self.A = self.aircraft_data.data['outputs']['wing_design']['aspect_ratio'] # wing aspect ratio
        self.B_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b'] # horizontal tail span
        self.B_w = aircraft_data.data['outputs']['wing_design']['b'] # wing span
        self.D = # fuselage structural depth
        self.D_e = 5.3 #engine diameter
        self.F_w = aircraft_data.data['outputs']['general']['d_fuselage'] # fuselage width at horizontal tail
        self.H_t = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['tail_height'] - self.aircraft_data.data['outputs']['general']['d_fuselage'] # horizontal tail height
        self.H_t_H_v = 0 if self.aircraft_data.data['inputs']['tail_type'] == "CONVENTIONAL" else 1
        self.I_y = # Yawing moment of interia
        self.K_cb = 1 # landging gear so 0???
        self.K_d = # Duct constant from figure
        self.K_door = 1.12 # door constant from Raymer
        self.K_dw = 1 # door constant from Raymer for delta wing
        self.K_dwf = 1 # door constant from Raymer for delta wing
        self.K_lg = 1 # landing gear constant from Raymer maybe 0???
        self.K_mc = 1.45 # mission completed after failure
        self.K_mp = 1 # landing gear constant from Raymer maybe 0???
        self.K_ng = 1.017 # Pylon mounted nacelle
        self.K_np = 1 # landing gear constant from Raymer maybe 0???
        self.K_p = 1.4 # For propeller
        self.K_r = 1 # For reciprocating engine
        self.K_rht = 1 # For rolling tail                                   # Check this
        self.K_tp = 0.793 # For turboprop
        self.K_tpg = 1 # landing gear constant from Raymer maybe 0???
        self.K_tr = 1 # For jet with thrust reverser
        self.K_uht = 1 # For all moving horizontal tail                         # Check this
        self.K_vg = 1 # For variable geometry
        self.K_vs = 1 # Variable sweep
        self.K_vsh = 1 # Variable sweep
        self.K_ws = 0.75*((1 + 2 * self.taper_ratio) / (1 + self.taper_ratio)) * self.b_w_ft * np.tan(self.sweep_c_4_rad) / self.L_fuselage_ft # fuselage constant
        self.K_y = 0.3*self.L_t_ft # Radius of gyration for pitching axis
        self.K_z = self.L_t_ft # Radius of gyration for yawing axis
        self.L = aircraft_data.data['outputs']['general']['l_fuselage'] # structural length
        self.L_a = # electrical routing distance, generators to avionics to cockpit
        self.L_d = # duct length
        self.L_ec = # length from engine front to cockpit, total if multiengine
        self.L_f = # total fuselage length
        self.L_m = 0 # landing gear
        self.L_n = 0 # # landing gear
        self.L_s = # single duct length from figure
        self.L_sh = # length of engine shroud
        self.L_t = (self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos']+0.25*self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['MAC'])-(self.aircraft_data.data['outputs']['wing_design']['X_LEMAC'] + 0.25*self.aircraft_data.data['outputs']['wing_design']['MAC']) # tail cone length, wing c/4 to tail c/4
        self.L_tp = # length of tailpipe
        self.M = ISA(self.aircraft_data.data['inputs']['cruise_altitude']).Mach(self.aircraft_data.data['requirements']['cruise_speed']) # air density
        self.N_c = 5 # number of crew
        self.N_ci = 2 # number of pilots
        self.N_en = self.aircraft_data.data['inputs']['n_engines'] # number of engines
        self.N_f = # number of actions performed by control
        self.N_gen = # number of generators
        self.N_l = # ultimate landing load factor


        # Core aircraft properties
        self.MTOM_kg = aircraft_data.data['outputs']['max']['MTOM']
        self.n_ult = 1.5 * aircraft_data.data['outputs']['general']['nmax']
        self.MTOM_lbs = kg2lbs(self.MTOM_kg)
        self.W_dg_lbs = self.MTOM_lbs  # Design gross weight

        # Wing design
        self.S_w_ft2 = msq2ftsq(aircraft_data.data['outputs']['wing_design']['S'])
        self.aspect_ratio = aircraft_data.data['outputs']['wing_design']['aspect_ratio']
        # self.t_c_root = aircraft_data.data['outputs']['wing_design']['t_c_root']
        self.taper_ratio = aircraft_data.data['outputs']['wing_design']['taper_ratio']
        self.sweep_c_4_rad = deg2rad(aircraft_data.data['outputs']['wing_design']['sweep_c_4'])
        # self.S_csw_ft2 = msq2ftsq(aircraft_data.data['outputs']['control_design']['S_csw'])
        self.b = aircraft_data.data['outputs']['wing_design']['b']
        self.b_ft = m2ft(aircraft_data.data['outputs']['wing_design']['b'])

        # Horizontal tail
        self.F_w = aircraft_data.data['outputs']['general']['d_fuselage']
        self.b_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b']
        self.S_h = msq2ftsq(aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'])
        self.l_t = aircraft_data.data['outputs']['general']['l_tailcone']
        self.K_y_ft = 0.3 * self.l_t_ft
        self.lambda_ht_rad = deg2rad(aircraft_data.data['outputs']['horizontal_tail_design']['lambda_ht'])
        self.aspect_ratio_h = aircraft_data.data['outputs']['horizontal_tail_design']['aspect_ratio']
        self.S_e_ft2 = msq2ftsq(aircraft_data.data['outputs']['control_design']['S_e'])

        # Vertical tail
        self.S_v_ft2 = aircraft_data.data['outputs']['empennage_design']['S_v']
        self.K_z_ft = self.l_t_ft
        self.lambda_v_rad = aircraft_data.data['outputs']['empennage_design']['sweep_v']
        self.aspect_ratio_v = aircraft_data.data['outputs']['empennage_design']['aspect_ratio_v']
        self.t_c_root_v = aircraft_data.data['outputs']['empennage_design']['t_c_root_v']

        # Fuselage
        self.L_fuselage_ft = aircraft_data.data['outputs']['general']['L']
        self.taper_ratio_fuselage = aircraft_data.data['outputs']['fuselage_design']['taper_ratio']
        self.D_fuselage_ft = aircraft_data.data['outputs']['general']['D_fuselage']
        self.l_fuselage_ft = aircraft_data.data['outputs']['general']['l_fuselage']
        self.S_fuselage_ft2 = 2 * np.pi * (self.D_fuselage_ft / 2) * self.L_fuselage_ft
        self.K_ws = 0.75 * ((1 + 2 * self.taper_ratio_fuselage) / (1 + self.taper_ratio_fuselage)) * self.b_w_ft * np.tan(self.sweep_c_4_rad) / self.L_fuselage_ft
        self.LD_ratio = self.l_fuselage_ft / self.D_fuselage_ft

    def wing_weight(self) -> float:
        W_wing_lbs = 0.0051 * (self.MTOM_lbs * self.n_ult)**0.557 \
                           * self.S_w_ft2**0.649 \
                           * self.aspect_ratio**0.5 \
                           * self.t_c_root**-0.4 \
                           * (1 + self.taper_ratio)**0.1 \
                           * np.cos(self.sweep_c_4_rad)**-1 \
                           * self.S_csw_ft2**0.1
        return lbs2kg(W_wing_lbs)

    def horizontal_tail(self) -> float:
        W_horizontal_tail_lbs = 0.0379 * (1 + self.F_w_ft / self.b_h_ft)**-0.25 \
                                      * self.W_dg_lbs**0.639 \
                                      * self.n_ult**0.10 \
                                      * self.S_h_ft2**0.75 \
                                      * self.l_t_ft**-1 \
                                      * self.K_y_ft**0.704 \
                                      * np.cos(self.lambda_ht_rad)**-1 \
                                      * self.aspect_ratio_h**0.166 \
                                      * (1 + self.S_e_ft2 / self.S_h_ft2)**0.1
        return lbs2kg(W_horizontal_tail_lbs)

    def vertical_tail(self) -> float:
        Ht_Hv = 1  # T-tail configuration
        W_vertical_tail_lbs = 0.0026 * (1 + Ht_Hv)**0.225 \
                                    * self.W_dg_lbs**0.556 \
                                    * self.n_ult**0.536 \
                                    * self.l_t_ft**-0.5 \
                                    * self.S_v_ft2**0.5 \
                                    * self.K_z_ft**0.875 \
                                    * np.cos(self.lambda_v_rad)**-1 \
                                    * self.aspect_ratio_v**0.35 \
                                    * self.t_c_root_v**-0.5
        return lbs2kg(W_vertical_tail_lbs)

    def fuselage(self) -> float:
        K_door = 1.12
        K_lg = 1
        W_fuselage_lbs = 0.3280 * K_door * K_lg \
                               * (self.W_dg_lbs * self.n_ult)**0.5 \
                               * self.L_fuselage_ft**0.25 \
                               * self.S_fuselage_ft2**0.302 \
                               * (1 + self.K_ws)**0.04 \
                               * (self.LD_ratio)**-0.10
        return lbs2kg(W_fuselage_lbs)

    def main_landing_gear(self):
        return 0

    def nose_landing_gear(self):
        return 0

if __name__ == "__main__":
    aircraft_data = Data("design1.json")
    class_ii = ClassII(aircraft_data)

    print(f"Wing weight: {class_ii.wing_weight():,.2f} kg")
    print(f"Horizontal tail weight: {class_ii.horizontal_tail():,.2f} kg")
    print(f"Vertical tail weight: {class_ii.vertical_tail():,.2f} kg")
    print(f"Fuselage weight: {class_ii.fuselage():,.2f} kg")
