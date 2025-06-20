import os
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils import Data

class Tail_area:
    def __init__(self, aircraft_data: Data, fwd_cg, aft_cg):
        self.aircraft_data = aircraft_data
        self.design_id = aircraft_data.data['design_id']
        self.design_file = f"design{self.design_id}.json"
        self.tail_type = aircraft_data.data['inputs']['tail_type']
        self.A = aircraft_data.data['inputs']['aspect_ratio']
        self.S = aircraft_data.data['outputs']['wing_design']['S']
        self.c_root = aircraft_data.data['outputs']['wing_design']['chord_root']
        self.c_tip = aircraft_data.data['outputs']['wing_design']['chord_tip']
        self.most_aft_cg = aircraft_data.data['outputs']['cg_range']['most_aft_cg']
        self.most_fwd_cg = aircraft_data.data['outputs']['cg_range']['most_forward_cg']

        self.lemac = aircraft_data.data['outputs']['wing_design']['X_LEMAC']
        self.MAC = self.aircraft_data.data['outputs']['wing_design']['MAC']
        self.most_aft_cg = aft_cg * self.MAC + self.lemac
        self.most_fwd_cg = fwd_cg * self.MAC + self.lemac
        self.horizontal_tail_pos = aircraft_data.data['outputs']['component_positions']['horizontal_tail']
        self.l_h = aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['l_h']  # horizontal tail arm length

    def get_downwash(self):
        if self.tail_type == "T_TAIL":
            downwash = 0.46
            return downwash
        else:
            downwash = 4/(self.A+2)
            return downwash
        
    def get_aerodynamic_center(self):
        # TODO: link to json
        X_ac = 0.277360658914395 #hard coded for now, change later to link to aero
        return X_ac
    
    def get_normalised_cg(self):
        c = self.MAC
        X_cg_fwd_bar = (self.most_fwd_cg-self.lemac)/c
        X_cg_aft_bar = (self.most_aft_cg-self.lemac)/c
        X_cg = np.array([X_cg_fwd_bar,X_cg_aft_bar])
        return X_cg
            
    def Sh_S_stability(self,X_cg):
        # TODO: link to json
        CL_alpha_h = 0.1187 #need to account for the tail
        CL_alpha_A_h = 0.10126
        lh = self.l_h
        downwash = self.get_downwash()
        c = self.MAC
        X_ac = self.get_aerodynamic_center()
        Stability_margin = 0.1 #maybe look up if this needs to be bigger for ekrano
        Vh_V = 1

        Sh_S = 1/((CL_alpha_h/CL_alpha_A_h)*(1-downwash)*(lh/c)*Vh_V**2)*X_cg - (X_ac-Stability_margin)/((CL_alpha_h/CL_alpha_A_h)*(1-downwash)*(lh/c)*Vh_V**2)
        #print("Sh/h:",Sh_S)
        return Sh_S
    

    def Sh_S_controllability(self,X_cg):
        X_ac = self.get_aerodynamic_center()
        # TODO: link to json
        C_m_ac = -0.25 #outta Martin's ass, change later 
        CL_A_h = 1.13 #5def AOA
        CL_h = 0.59
        Vh_V = 1
        lh = self.l_h
        c = self.MAC
        Sh_S = -(X_cg-X_ac+(C_m_ac/CL_A_h))/((CL_h/CL_A_h)*(lh/c)*Vh_V**2)
        return Sh_S


    def plot(self, Xcg_range):
        Sh_S_stability_vals = [self.Sh_S_stability(xcg) for xcg in Xcg_range]
        Sh_S_control_vals = [self.Sh_S_controllability(xcg) for xcg in Xcg_range]
        X_cg_bar = self.get_normalised_cg()
        X_cg_fwd_bar = X_cg_bar[0]
        X_cg_aft_bar = X_cg_bar[1]

        plt.figure(figsize=(8, 5))
        plt.plot(Xcg_range, Sh_S_stability_vals, label='Sh/S (Stability)', color='red', linewidth=2)
        plt.plot(Xcg_range, Sh_S_control_vals, label='Sh/S (Controllability)', color='blue', linestyle='--', linewidth=2)

        # Add vertical lines for CG range
        plt.axvline(x=X_cg_fwd_bar, color='green', linestyle='-.', linewidth=1.5, label='X_cg (min)')
        plt.axvline(x=X_cg_aft_bar, color='purple', linestyle='-.', linewidth=1.5, label='X_cg (max)')

        plt.xlabel('X_cg')
        plt.ylabel('Sh/S')
        plt.title('Scissor plot')
        plt.grid(True)
        plt.legend()
        plt.xlim(-0.2, 1)
        plt.ylim(0, 1)
        plt.tight_layout()
        plt.show()



    def get_tail_area(self):
        X_cg_bar = self.get_normalised_cg()
        X_cg_fwd_bar = X_cg_bar[0]
        X_cg_aft_bar = X_cg_bar[1]
        Sh_stability = self.Sh_S_stability(X_cg_aft_bar)*self.S
        Sh_controllability = self.Sh_S_controllability(X_cg_fwd_bar)*self.S
        #print('tail_area:',Sh)
        return Sh_stability,Sh_controllability
        

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    Xcg_values = np.linspace(-0.5, 1.2, 200)
    fwd_cg = 0.2416
    aft_cg = 0.4874
    tail = Tail_area(aircraft_data=aircraft_data, fwd_cg=fwd_cg, aft_cg=aft_cg)
    plot = tail.plot(Xcg_values)
    tail_area = tail.get_tail_area()
    print('tail_area:',tail_area)

