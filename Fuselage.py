import numpy as np
from utils import Data

class Fuselage:
    def __init__(self,
                 aircraft_data: Data
                 ) -> None:
                 
        self.aircraft_data = aircraft_data

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

    def get_cargo_bay_length(self):
        return self.payload/self.cargo_density/self.cargo_width/self.cargo_height/self.n_fuselages

    def get_minimum_diameter(self):
        return np.sqrt(self.cargo_width**2 + self.cargo_height**2)

    def deg2rad(self,deg):
        return deg * np.pi / 180

    def CalcFuseDia(self):
        Dmin = self.get_minimum_diameter()
        d = (self.l+Dmin/np.tan(self.deg2rad(self.upsweep)))/(self.lfd-self.ltcd-self.lnd+1/np.tan(self.deg2rad(self.upsweep)))
        if d >= Dmin:
            return d
        else:
            return Dmin #minimum needed to satisfy the rEqUirEmeNt of cargo hold area

    def CalcFuseLen(self):   #length of cargo bay in "straight body region", upsweep of tail, lf/d, ltc/d, ln/d
        self.l = self.get_cargo_bay_length()
        self.d = self.CalcFuseDia()
        self.lf = self.lfd*self.d
        self.ltc = self.ltcd*self.d
        self.ln = self.lnd*self.d
        self.total_length = self.lf + self.ltc + self.ln

        self.update_attributes()

    def update_attributes(self):
        self.aircraft_data.data['inputs']['d_fuselage'] = self.d
        self.aircraft_data.data['inputs']['r_fuselage'] = self.d/2
        self.aircraft_data.data['inputs']['l_fuselage'] = self.lf
        self.aircraft_data.data['inputs']['l_tailcone'] = self.ltc
        self.aircraft_data.data['inputs']['l_nose'] = self.ln
        self.aircraft_data.data['inputs']['L'] = self.total_length

if __name__ == "__main__":
    aircraft_data = Data("design1.json")
    fuselage = Fuselage(aircraft_data=aircraft_data)
    fuselage.CalcFuseLen()
    print(f"Fuselage Diameter: {fuselage.d} m")
    print(f"l_fus {fuselage.lf}")
    print(f"l_tailcone {fuselage.ltc}")
    print(f"l_nose {fuselage.ln}")
    print(f"Fuselage Length: {fuselage.total_length} m")
