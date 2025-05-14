from utils import *
import numpy as np



class ClassII:
    def __init__(self, aircraft_data: Data) -> None:
        self.aircraft_data = aircraft_data
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f"designs{self.design_number}.json"

    def wing_weight(self) -> float:
        MTOM = kg2lbs(self.aircraft_data.data['outputs']['max']['MTOM'])
        n_ult = 1.5*self.aircraft_data.data['outputs']['general']['n_limit']
        S = msq2ftsq(self.aircraft_data.data['outputs']['wing_design']['S'])
        aspect_ratio = self.aircraft_data.data['outputs']['wing_design']['aspect_ratio']
        t_c_root = self.aircraft_data.data['outputs']['wing_design']['t_c_root']
        taper_ratio = self.aircraft_data.data['outputs']['wing_design']['taper_ratio']
        sweep_c_4 = deg2rad(self.aircraft_data.data['outputs']['wing_design']['sweep_c_4'])
        S_csw = msq2ftsq(self.aircraft_data.data['outputs']['control_design']['S_csw'])
        
        # TODO: control surface area and t_c at root
        W_wing = 0.0051 * (MTOM*n_ult)**0.557*S**0.649*aspect_ratio**0.5*t_c_root**-0.4*(1+taper_ratio)**0.1*np.cos(sweep_c_4)**-1*S_csw**0.1
        return lbs2kg(W_wing)
    
    def horizontal_tail(self):
        K_uht = 1
        F_w = self.aircraft_data.data['outputs']['wing_design']['F_w'] # Fuselage width at horizontal tail intersection in ft
        b_h = self.aircraft_data.data['outputs']['horizontal_tail_design']['b_h'] # Horizontal tail span in ft
        W_dg = kg2lbs(self.aircraft_data.data['outputs']['max']['MTOM']) # Design gross weight in lbs
        n_ult = 1.5*self.aircraft_data.data['outputs']['general']['n_limit'] # Ultimate load factor
        S_h = msq2ftsq(self.aircraft_data.data['outputs']['horizontal_tail_design']['S_h']) # Horizontal tail area in ft^2
        l_t = self.aircraft_data.data['outputs']['horizontal_tail_design']['l_t'] # Tail length; wing quarter chord to horizontal tail quarter chord in ft
        K_y = 0.3*l_t # aircraft pitching radius of gyration in ft approx 0.3*l_t
        lambda_ht = deg2rad(self.aircraft_data.data['outputs']['horizontal_tail_design']['lambda_ht']) # Horizontal sweep at quarter chord in radians
        aspect_ratio_h = self.aircraft_data.data['outputs']['horizontal_tail_design']['aspect_ratio'] # Horizontal tail aspect ratio
        S_e = msq2ftsq(self.aircraft_data.data['outputs']['control_design']['S_e']) # Elevator area in ft^2

        W_horizontal_tail = 0.0379*K_uht*(1+F_w/b_h)**-0.25*W_dg**0.639*n_ult**0.10*S_h**0.75*l_t**-1*K_y**0.704*np.cos(lambda_ht)**-1*aspect_ratio_h**0.166*(1+S_e/S_h)**0.1
        return lbs2kg(W_horizontal_tail)
    
    def vertical_tail(self):
        Ht_Hv = 1 # Horizontal tail height above fuselage over veritcal tail height 1 for t-tail; 0 for conventional tail
        W_dg = kg2lbs(self.aircraft_data.data['outputs']['max']['MTOM']) # design gross weight in lbs
        l_t = self.aircraft_data.data['outputs']['horizontal_tail_design']['l_t'] # tail length in ft
        n_ult = 1.5*self.aircraft_data.data['outputs']['general']['n_limit']# ultimate load factor
        S_v = self.aircraft_data.data['outputs']['empennage_design']['S_v'] # vertical tail area in ft^2
        K_z = l_t # aircraft yawing radius of gyration in ft approx l_t
        lambda_v = self.aircraft_data.data['outputs']['empennage_design']['sweep_v'] # sweep angle vertical tail at quarter chord in radians
        aspect_ratio_v = self.aircraft_data.data['outputs']['empennage_design']['aspect_ratio_v'] # vertical tail aspect ratio
        t_c_root = self.aircraft_data.data['outputs']['empennage_design']['t_c_root_v'] # vertical tail thickness to chord ratio at root

        W_vertical_tail = 0.0026*(1+Ht_Hv)**0.225*W_dg**0.556*n_ult**0.536*l_t**-0.5*S_v**0.5*K_z**0.875*np.cos(lambda_v)**-1*aspect_ratio_v**0.35*t_c_root**-0.5
        return lbs2kg(W_vertical_tail)
    
    def fuselage(self):
        K_door = 1.12 # depends on number of doors and type of door
        K_lg = 1 # Depends on landing gear type
        W_dg = kg2lbs(self.aircraft_data.data['outputs']['max']['MTOM']) # design gross weight in lbs
        n_ult = 1.5*self.aircraft_data.data['outputs']['general']['n_limit'] # ultimate load factor
        L = self.aircraft_data.data['outputs']['general']['L'] # fuselage length in ft excludes radome tail cap
        taper_ratio = self.aircraft_data.data['outputs']['fuselage_design']['taper_ratio'] # fuselage taper ratio
        b_w = self.aircraft_data.data['outputs']['wing_design']['b_w'] # wing span in ft
        sweep_c_4 = deg2rad(self.aircraft_data.data['outputs']['wing_design']['sweep_c_4']) # wing quarter chord sweep in radians
        S_f = 2*np.pi*self.aircraft_data.data['outputs']['general']['D_fuselage']/2*L # fuselage wetted area in ft^2
        K_ws = 0.75*((1+2*taper_ratio)/(1+taper_ratio))*b_w*np.tan(sweep_c_4)/L # wing to fuselage interference factor
        LD = self.aircraft_data.data['outputs']['general']['l_fuselage'] / self.aircraft_data.data['outputs']['general']['D_fuselage'] # fuselage length to diameter ratio
    
        W_fuselage = 0.3280*K_door*K_lg*(W_dg*n_ult)**0.5*L**0.25*S_f**0.302*(1+K_ws)**0.04*(LD)**-0.10
        return lbs2kg(W_fuselage)

if __name__ == "__main__":
    # Example usage
    aircraft_data = Data("design1.json")
    class_ii = ClassII(aircraft_data)
    wing_weight = class_ii.wing_weight()
    horizontal_tail_weight = class_ii.horizontal_tail()
    vertical_tail_weight = class_ii.vertical_tail()
    fuselage_weight = class_ii.fuselage()
    print(f"Wing weight: {wing_weight/9.81:=,} kg")
    print(f"Horizontal tail weight: {horizontal_tail_weight/9.81:=,} kg")
    print(f"Vertical tail weight: {vertical_tail_weight/9.81:=,} kg")
    print(f"Fuselage weight: {fuselage_weight/9.81:=,} kg")