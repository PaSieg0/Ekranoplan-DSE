import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils import Data, W2hp

class vertical_tail_sizing:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"
        self.tail_type = aircraft_data.data['inputs']['tail_type']
        self.A = aircraft_data.data['inputs']['aspect_ratio']
        self.S = aircraft_data.data['outputs']['wing_design']['S']
        self.b = aircraft_data.data['outputs']['wing_design']['b']
        self.c_root = aircraft_data.data['outputs']['wing_design']['chord_root']
        self.c_tip = aircraft_data.data['outputs']['wing_design']['chord_tip']
        self.most_aft_cg = aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        self.most_fwd_cg = aircraft_data.data['outputs']['cg_range']['most_forward_cg']
        self.lv = aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v']

        self.lemac = aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.MTOW = self.aircraft_data.data['outputs']['max']['MTOW']
        self.V_cruise = self.aircraft_data.data['requirements']['cruise_speed']

        self.Cn_beta = 0.016940719873178536
        self.Cn_beta_A_h = -0.001983579626566832
        self.Cy_beta_v = -0.02735301051550483
        self.lv = self.lv
        self.S = self.S
        self.b = self.b
        self.sidewash = 0
        self.Vv_V = 0.9
        self.nv = self.Vv_V**2
        self.Ye = max(aircraft_data.data['outputs']['engine_positions']['y_engines'])

        self.P_engine = aircraft_data.data['inputs']['engine']['engine_power']
        self.V_stall = aircraft_data.data['requirements']['stall_speed_landing']
     

    #method of Torenbek, static directional stability
    def get_vertical_tail_size_static_stab(self):
        Cn_beta = self.Cn_beta
        Cn_beta_A_h = self.Cn_beta_A_h
        Cy_beta_v = self.Cy_beta_v
        lv = self.lv
        S = self.S
        b = self.b
        Vv_V = 0.9
        nv = self.nv
        sidewash = 1.15566/nv-1 #d sigma/d beta
        Sv = (Cn_beta-Cn_beta_A_h)*(S*b)/(Cy_beta_v*(1-sidewash)*Vv_V**2*lv) #Sv >
        return Sv

    #one engine inoperative case
    def get_vertical_tail_size_one_engine_inoperative(self):
        nv = self.nv
        Cl_alpha_v = 0.124  # deg
        Cl_alpha_v_rad = np.rad2deg(Cl_alpha_v)
        Cy_v_alpha = Cl_alpha_v_rad
        S = self.S
        CL = self.aircraft_data.data['inputs']['CLmax_landing'] # self.getCL()
        Ye = self.Ye
        lv = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v']
        deltaTe = self.P_engine / self.V_stall
        W = self.MTOW
        beta = 0
        Cn_beta_A_h = self.Cn_beta_A_h
        b = self.b
        tau_v = -0.7061
        del_r = self.aircraft_data.data['inputs']['control_surfaces']['rudder_deflection']
        sigma_v = np.deg2rad(5)

        Sv = S * (CL * (Ye / lv) * (deltaTe / W) + beta * Cn_beta_A_h * (b / lv)) / (
            (tau_v * del_r - (beta - sigma_v)) * nv * Cy_v_alpha
        )
        self.X_axis = deltaTe * Ye * CL / (W * lv)

        self.y_axis_thingy = 0.4
        self.S_v = self.y_axis_thingy * S / (nv * Cy_v_alpha)

        return self.S_v
    
    def get_x_axis_fig_9_23(self):
        ye = self.Ye
        lv = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v']
        Peq = W2hp(self.P_engine)
        CLmax_TO = self.aircraft_data.data['inputs']['CLmax_takeoff']
        W = self.aircraft_data.data['outputs']['max']['MTOM']
        W_pmax = 100000

        X_axis = ye/lv * (Peq*CLmax_TO)/(W - W_pmax)
        print("X_axis (fig 9.23):", X_axis)


    def get_Sv_from_fig_9_23(self, y_axis):
        S = self.S
        # TODO : CHECK IF THIS IS STILL CORRECT
        k_delta_r = 1.1
        k_v = 1.1
        S_r = self.aircraft_data.data['outputs']['control_surfaces']['rudder']['area']
        A_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['aspect_ratio']
        chord_ratio = self.aircraft_data.data['inputs']['control_surfaces']['rudder_chord']
        b_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']
        v_sweep = np.deg2rad(self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep_c_4'])
        ct_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
        cr_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']

        tip_c_v_offset = b_v*np.tan(v_sweep)
        tip_start_r = tip_c_v_offset + (1-chord_ratio-0.25)*ct_v

        root_start_r = (1-chord_ratio-0.25)*cr_v
        
        sweep_r = np.arctan((tip_start_r- root_start_r) / (b_v))

        S_v = ((y_axis*S)/(k_delta_r * k_v * S_r**(1/3) * A_v**(1/3) * np.cos(sweep_r)**(1/3) ))**(3/2)
        return S_v
    
    def get_vertical_tail_size(self):
        static_stability = self.get_vertical_tail_size_static_stab()
        one_engine_inoperative = self.get_vertical_tail_size_one_engine_inoperative()
        # TODO: UPDATE y_axis VALUE
        fig_23 = self.get_Sv_from_fig_9_23(y_axis=0.15)
        tail_volume = self.check_tail_volume()

        print(f"Static Stability: {static_stability}, One Engine Inoperative: {one_engine_inoperative}, Fig 9.23: {fig_23}, Tail Volume: {tail_volume}")


        tail_size = max(static_stability, one_engine_inoperative, fig_23, tail_volume)
        self.update_vertical_tail_dimensions(tail_size)
        self.aircraft_data.save_design(design_file=self.design_file)
        return tail_size
    
    def update_vertical_tail_dimensions(self, tail_size):
        tail_size /= 2
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S'] = tail_size
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'] = np.sqrt(tail_size * self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['aspect_ratio'])
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root'] = 2 * tail_size / (self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'] * (1 + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper']))
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip'] = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root'] * self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper']
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['MAC'] = (2 / 3) * self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root'] * ((1 + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper'] + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper']**2) / (1 + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper']))
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['z_MAC'] = (self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'] / 6) * (1 + 2 * self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper']) / (1 + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['taper'])

        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos'] = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'] - self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_root']
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v'] = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos'] - self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['relative_pos_mac/4'] = (self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos'] + 0.25*self.aircraft_data.data['outputs']['wing_design']['MAC']) / (self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage'])
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['quarter_tip'] = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['LE_pos'] + self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b']*np.tan(np.deg2rad(self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['sweep'])) + 0.25 * self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['chord_tip']
        vertical_attachment = (self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b'] - self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage']) / 2
        desired_vertical_attachment = self.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['b'] / 3
        print(vertical_attachment, desired_vertical_attachment)
        self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['attachement_angle'] = np.rad2deg(np.arctan(vertical_attachment - desired_vertical_attachment) / self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['b'])

    
    def calculate_K_beta(self):
        l_cg = self.aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        l_f = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        h_f_max = self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage']
        K_beta = 0.3*l_cg/l_f + 0.75*h_f_max/l_f - 0.105
        return K_beta
    
    
    def calculate_C_n_beta_f(self):
        K_beta = self.calculate_K_beta()
        S_fs = self.aircraft_data.data['outputs']['fuselage_dimensions']['S_sideview']
        l_f = self.aircraft_data.data['outputs']['fuselage_dimensions']['l_fuselage']
        h_f1 = self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_1_4']
        h_f2 = self.aircraft_data.data['outputs']['fuselage_dimensions']['h_fuselage_3_4']
        b_f1 = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage_1_4']
        b_f2 = self.aircraft_data.data['outputs']['fuselage_dimensions']['w_fuselage_3_4']
        S = self.aircraft_data.data['outputs']['wing_design']['S']
        b = self.aircraft_data.data['outputs']['wing_design']['b']

        C_n_beta_f = -K_beta * S_fs*l_f/(S*b) * (h_f1/h_f2)**(1/2) * (b_f1/b_f2)**(1/3)   
        return C_n_beta_f
    
    def calculate_C_n_beta_i(self):
        return -0.017 # HIGH WING
    
    def calculate_C_n_beta_p(self):
        # TODO: CHECK IF VALUES IN JSON ARE CORRECT
        B_p = self.aircraft_data.data['inputs']['engine']['n_blades']
        S = self.aircraft_data.data['outputs']['wing_design']['S']
        b = self.aircraft_data.data['outputs']['wing_design']['b']
        D_p = self.aircraft_data.data['inputs']['engine']['prop_diameter']
        C_n_beta_p = -0.053*B_p*sum(2*l_p * D_p**2 / (S*b) for l_p in self.aircraft_data.data['outputs']['engine_positions']['x_engines'])
        return C_n_beta_p

    def check_tail_volume(self):
        C_n_beta_f = self.calculate_C_n_beta_f()
        C_n_beta_i = self.calculate_C_n_beta_i()
        C_n_beta_p = self.calculate_C_n_beta_p()
        l_v = self.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['l_v']

        print("sum of C_n_beta:", C_n_beta_f + C_n_beta_i + C_n_beta_p)

        # TODO: UPDATE THIS VALUE 
        volume = 0.065

        min_Sv = volume * self.S * self.b / l_v
        return min_Sv
    
if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    vert_tail = vertical_tail_sizing(aircraft_data=aircraft_data)

    tail_size = vert_tail.get_vertical_tail_size()
    vert_tail.get_x_axis_fig_9_23()
    print('Vertical Tail Area:', tail_size)
