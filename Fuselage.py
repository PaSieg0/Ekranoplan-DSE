import numpy as np
from utils import Data

class Fuselage:
    def __init__(self,
                 aircraft_data: Data
                 ) -> None:
                 
        self.aircraft_data = aircraft_data
        self.design_number = self.aircraft_data.data['design_id']
        self.design_file = f"design{self.design_number}.json"

        # from Raymer's book
        self.ltcd = 4.5 
        self.lfd = 8.5 
        self.lnd = 1.5 

        self.cargo_width = self.aircraft_data.data['requirements']['cargo_width']
        self.cargo_height = self.aircraft_data.data['requirements']['cargo_height']
        self.cargo_length = self.aircraft_data.data['requirements']['cargo_length']
        self.cargo_density = self.aircraft_data.data['requirements']['cargo_density']
        self.payload = self.aircraft_data.data['requirements']['design_payload']

        self.upsweep = self.aircraft_data.data['inputs']['upsweep']
        self.n_fuselages = self.aircraft_data.data['inputs']['n_fuselages']

    def get_tot_fus_length(self):
        return self.payload/self.cargo_density/self.cargo_width/self.cargo_height/self.n_fuselages

    def get_minimum_diameter(self):
        return np.sqrt(self.cargo_width**2 + self.cargo_height**2)

    def deg2rad(self,deg):
        return deg * np.pi / 180

    def CalcFuseDia(self):
        Dmin = self.get_minimum_diameter()
        d = (self.tot_cargo_length+Dmin/np.tan(self.deg2rad(self.upsweep)))/(self.lfd-self.ltcd-self.lnd+1/np.tan(self.deg2rad(self.upsweep)))
        if d >= Dmin:
            return d
        else:
            return Dmin #minimum needed to satisfy the rEqUirEmeNt of cargo hold area

    def calculate_y_in_tail(self):
        y_in_tail = (self.d-self.cargo_height)/np.tan(self.deg2rad(self.upsweep))
        cargo_straight = self.tot_cargo_length - y_in_tail
        return cargo_straight

    def CalcFuseLen(self):   #length of cargo bay in "straight body region", upsweep of tail, lf/d, ltc/d, ln/d
        self.tot_cargo_length = self.get_tot_fus_length()
        self.d = self.CalcFuseDia()
        self.lf = self.lfd*self.d
        self.ltc = self.ltcd*self.d
        self.ln = self.lnd*self.d
        self.cargo_straight = self.calculate_y_in_tail()
        self.update_attributes()
        self.aircraft_data.save_design(self.design_file)


    def update_attributes(self):
        self.aircraft_data.data['outputs']['general']['d_fuselage'] = self.d
        self.aircraft_data.data['outputs']['general']['r_fuselage'] = self.d/2
        self.aircraft_data.data['outputs']['general']['l_fuselage'] = self.lf
        self.aircraft_data.data['outputs']['general']['l_tailcone'] = self.ltc
        self.aircraft_data.data['outputs']['general']['l_nose'] = self.ln
        self.aircraft_data.data['outputs']['general']['l_cargo_straight'] = self.cargo_straight


if __name__ == "__main__":
    aircraft_data = Data("design4.json")
    fuselage = Fuselage(aircraft_data=aircraft_data)
    fuselage.CalcFuseLen()
    print(f"Fuselage Diameter: {fuselage.d} m")
    print(f"l_fus {fuselage.lf}")
    print(f"l_tailcone {fuselage.ltc}")
    print(f"l_nose {fuselage.ln}")

