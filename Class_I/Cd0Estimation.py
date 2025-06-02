import numpy as np
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, EmpType, AircraftType, MissionType
from Class_I.ClassIWeightEstimation import ClassI, MissionType, AircraftType
from Class_I.Iteration import AircraftIteration

class Cd0Estimation:
    #TODO: this is very preliminary, need to consider form factors, IFF Cfc etc. later
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f'design{self.design_number}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type
        self.tail_type = EmpType[aircraft_data.data['inputs']['tail_type']]
        self.t_c_wing = self.aircraft_data.data['inputs']['airfoils']['wing']
        self.t_c_htail = self.aircraft_data.data['inputs']['airfoils']['horizontal_tail']
        self.t_c_vtail = self.aircraft_data.data['inputs']['airfoils']['vertical_tail']
        self.d_F = self.aircraft_data.data['outputs']['general']['d_fuselage_equivalent_straight']
        self.l_F = self.aircraft_data.data['outputs']['general']['l_fuselage']
        self.lambda_F = self.l_F / self.d_F
        self.l_nacelle = self.aircraft_data.data['inputs']['engine']['engine_length']
        self.d_nacelle = 0.92  # diameter in meters
        self.IF_wingtail = 1.05 # https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_13_Drag.pdf
        self.IF_fus = 1. # https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_13_Drag.pdf
        self.IF_nacelle = 1.5 # https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_13_Drag.pdf

        self.tolerance = 0.0000001
        self.max_iterations = 100
        self.iteration_number = 0

        self.iteration = AircraftIteration(
            aircraft_data=self.aircraft_data,
            mission_type=self.mission_type
        )
    
    def wing_wet(self) -> float:
        term = 1 + 0.25 * self.t_c_wing # assimung same airfoil for all wing sections
        return 2 * self.iteration.aircraft_data.data['outputs']['max']['S'] * term

    def tail_wet(self) -> float:
        term_h = 1 + 0.25 * self.t_c_htail # assimung same airfoil for all wing sections
        term_v = 1 + 0.25 *self.t_c_vtail # assimung same airfoil for all wing sections
        return 2 * (self.iteration.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] * term_h +
                    self.iteration.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S'] * term_v)
    
    def wingtail_FF(self) -> float:
        isa = ISA(altitude=self.iteration.aircraft_data.data['inputs']['cruise_altitude'])
        mach = isa.Mach(self.iteration.aircraft_data.data['requirements']['cruise_speed'])
        sweep_maxt = self.iteration.aircraft_data.data['outputs']['wing_design']['sweep_c_4']
        x_c_m = 0.25 # assuming 25% chord for the wing, this is a simplification
        return (1 + 0.6/x_c_m * self.t_c_wing + 100*self.t_c_wing**4) * (1.34*mach**0.18*np.cos(sweep_maxt)**0.28)

    def fuselage_wet(self) -> float: # Torenbeek 1998
        term1 = np.pi * self.d_F * self.l_F
        term2 = (1 - 2 / self.lambda_F) ** (2 / 3)
        term3 = 1 + 1 / (self.lambda_F ** 2)
        return term1 * term2 * term3

    def fuselage_FF(self) -> float:
        f = self.l_F/self.d_F
        return 1 + 60/f**3 + f/400

    def nacelle_wet(self):
        correction_factor = 1.1  # Example correction factor, adjust as needed

        S_wet = np.pi * self.d_nacelle * self.l_nacelle * self.aircraft_data.data['inputs']['n_engines'] * correction_factor

        return S_wet

    
    def nacelle_FF(self) -> float:
        A_max = np.pi * self.d_nacelle**2 / 4  # Maximum cross-sectional area of the nacelle

        f = self.l_nacelle/np.sqrt(4/np.pi * A_max)
        return 1 + 0.35/f
    
    def additional_drag(self) -> float:
        A_max = np.pi * self.d_F**2 / 4
        fuselage_upsweep_drag = 3.83*np.radians(self.aircraft_data.data['inputs']['upsweep'])**2.5*A_max
        return fuselage_upsweep_drag
    
    def get_Cfc(self, l, laminar_ratio, k=1e-5) -> float:
        # Calculate the Reynolds number based on the air density, velocity, and viscosity
        isa = ISA(altitude=self.iteration.aircraft_data.data['inputs']['cruise_altitude'])
        Re = isa.rho * self.iteration.aircraft_data.data['requirements']['cruise_speed'] * l / self.iteration.aircraft_data.data['viscosity_air']
        Re2 = (38.21*(l/k)**1.053)
        Re = min(Re, Re2)
        mach = isa.Mach(self.iteration.aircraft_data.data['requirements']['cruise_speed'])
        Cfc_lam = 1.328 / np.sqrt(Re)
        Cfc_turb = 0.455 / (np.log10(Re)**2.58*(1 + 0.144 * mach**2)**0.65)
        Cfc = laminar_ratio*Cfc_lam + (1-laminar_ratio)*Cfc_turb
        return Cfc
    
    def get_S_ref(self) -> float:
        return self.iteration.aircraft_data.data['outputs']['max']['S']
    
    def update_attributes(self):
            mission_type = self.mission_type.name.lower()
            self.aircraft_data.data['outputs']['general']['LD_g'] = self.iteration.aircraft_data.data['outputs']['general']['LD_g']
            self.aircraft_data.data['outputs'][mission_type]['MTOM'] = self.iteration.aircraft_data.data['outputs'][mission_type]['MTOM']
            self.aircraft_data.data['outputs'][mission_type]['MTOW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['MTOW']
            self.aircraft_data.data['outputs'][mission_type]['OEW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['OEW']
            self.aircraft_data.data['outputs'][mission_type]['ZFW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['ZFW']
            self.aircraft_data.data['outputs'][mission_type]['EW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['EW'] 
            self.aircraft_data.data['outputs'][mission_type]['total_fuel'] = self.iteration.aircraft_data.data['outputs'][mission_type]['total_fuel']
            self.aircraft_data.data['outputs'][mission_type]['mission_fuel'] = self.iteration.aircraft_data.data['outputs'][mission_type]['mission_fuel']
            self.aircraft_data.data['outputs'][mission_type]['reserve_fuel'] = self.iteration.aircraft_data.data['outputs'][mission_type]['reserve_fuel'] 
            self.aircraft_data.data['outputs'][mission_type]['S'] = self.iteration.aircraft_data.data['outputs'][mission_type]['S']
            self.aircraft_data.data['outputs'][mission_type]['b'] = self.iteration.aircraft_data.data['outputs'][mission_type]['b']
            self.aircraft_data.data['outputs'][mission_type]['MAC'] = self.iteration.aircraft_data.data['outputs'][mission_type]['S'] / self.iteration.aircraft_data.data['outputs'][mission_type]['b']
            self.aircraft_data.data['outputs'][mission_type]['h_b'] = self.iteration.aircraft_data.data['outputs'][mission_type]['h_b']
            self.aircraft_data.data['outputs'][mission_type]['k'] = self.iteration.aircraft_data.data['outputs'][mission_type]['k']
            self.aircraft_data.data['outputs'][mission_type]['WP'] = self.iteration.aircraft_data.data['outputs'][mission_type]['WP']
            self.aircraft_data.data['outputs'][mission_type]['TW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['TW']
            self.aircraft_data.data['outputs'][mission_type]['WS'] = self.iteration.aircraft_data.data['outputs'][mission_type]['WS']
            self.aircraft_data.data['outputs'][mission_type]['Mff'] = self.iteration.aircraft_data.data['outputs'][mission_type]['Mff']
            self.aircraft_data.data['outputs'][mission_type]['LD'] = self.iteration.aircraft_data.data['outputs'][mission_type]['LD']
            
            if self.iteration.aircraft_data.data['outputs'][mission_type]['WP']:
                self.aircraft_data.data['outputs'][mission_type]['P'] = self.iteration.aircraft_data.data['outputs'][mission_type]['MTOW'] / self.iteration.aircraft_data.data['outputs'][mission_type]['WP']
            else: 
                self.aircraft_data.data['outputs'][mission_type]['P'] = None
            if self.iteration.aircraft_data.data['outputs'][mission_type]['TW']:
                self.aircraft_data.data['outputs'][mission_type]['T'] = self.iteration.aircraft_data.data['outputs'][mission_type]['MTOW'] * self.iteration.aircraft_data.data['outputs'][mission_type]['TW']
            else:
                self.aircraft_data.data['outputs'][mission_type]['T'] = None

            '''if self.mission_type == MissionType.DESIGN:
                self.aircraft_data.data['outputs'][mission_type]['fuel_economy'] = self.iteration.aircraft_data.data['outputs'][mission_type]['Fuel_used'] / 9.81 / 0.82 / (self.aircraft_data.data['requirements']['design_payload']/1000) / (self.iteration.aircraft_data.data['requirements']['design_range'] / 1000)
            elif self.mission_type == MissionType.ALTITUDE:
                self.aircraft_data.data['outputs'][mission_type]['fuel_economy'] = self.iteration.aircraft_data.data['outputs'][mission_type]['fuel_used'] / 9.81 / 0.82 / (self.aircraft_data.data['requirements']['altitude_payload']/1000) / ((self.iteration.aircraft_data.data['requirements']['altitude_range_WIG']+self.iteration.aircraft_data.data['requirements']['altitude_range_WOG']) / 1000)
            '''
            self.aircraft_data.data['outputs']['max']['MTOM'] = max(self.iteration.aircraft_data.data['outputs']['design']['MTOM'], self.iteration.aircraft_data.data['outputs']['ferry']['MTOM'], self.iteration.aircraft_data.data['outputs']['altitude']['MTOM'])
            self.aircraft_data.data['outputs']['max']['MTOW'] = self.iteration.aircraft_data.data['outputs']['max']['MTOM']*9.81
            self.aircraft_data.data['outputs']['max']['S'] = max(self.iteration.aircraft_data.data['outputs']['design']['S'], self.iteration.aircraft_data.data['outputs']['ferry']['S'], self.iteration.aircraft_data.data['outputs']['altitude']['S'])
            self.aircraft_data.data['outputs']['max']['b'] = max(self.iteration.aircraft_data.data['outputs']['design']['b'], self.iteration.aircraft_data.data['outputs']['ferry']['b'], self.iteration.aircraft_data.data['outputs']['altitude']['b'])
            self.aircraft_data.data['outputs']['max']['MAC'] = max(self.iteration.aircraft_data.data['outputs']['design']['MAC'], self.iteration.aircraft_data.data['outputs']['ferry']['MAC'], self.iteration.aircraft_data.data['outputs']['altitude']['MAC'])
            self.aircraft_data.data['outputs']['max']['fuel_economy'] = min(self.iteration.aircraft_data.data['outputs']['design']['fuel_economy'], self.iteration.aircraft_data.data['outputs']['altitude']['fuel_economy'])
            self.aircraft_data.data['outputs']['max']['Mff'] = min(self.iteration.aircraft_data.data['outputs']['design']['Mff'], self.iteration.aircraft_data.data['outputs']['ferry']['Mff'], self.iteration.aircraft_data.data['outputs']['altitude']['Mff'])
            self.aircraft_data.data['outputs']['max']['OEW'] = max(self.iteration.aircraft_data.data['outputs']['design']['OEW'], self.iteration.aircraft_data.data['outputs']['ferry']['OEW'], self.iteration.aircraft_data.data['outputs']['altitude']['OEW'])
            self.aircraft_data.data['outputs']['max']['total_fuel'] = max(self.iteration.aircraft_data.data['outputs']['design']['total_fuel'], self.iteration.aircraft_data.data['outputs']['ferry']['total_fuel'], self.iteration.aircraft_data.data['outputs']['altitude']['total_fuel'])
            self.aircraft_data.data['outputs']['max']['mission_fuel'] = max(self.iteration.aircraft_data.data['outputs']['design']['mission_fuel'], self.iteration.aircraft_data.data['outputs']['ferry']['mission_fuel'], self.iteration.aircraft_data.data['outputs']['altitude']['mission_fuel'])
            self.aircraft_data.data['outputs']['max']['reserve_fuel'] = max(self.iteration.aircraft_data.data['outputs']['design']['reserve_fuel'], self.iteration.aircraft_data.data['outputs']['ferry']['reserve_fuel'], self.iteration.aircraft_data.data['outputs']['altitude']['reserve_fuel'])
            self.aircraft_data.data['outputs']['max']['max_fuel'] = 1.1 * self.iteration.aircraft_data.data['outputs']['max']['total_fuel']
            self.aircraft_data.data['outputs']['max']['LD'] = max(self.iteration.aircraft_data.data['outputs']['design']['LD'], self.iteration.aircraft_data.data['outputs']['ferry']['LD'], self.iteration.aircraft_data.data['outputs']['altitude']['LD'])


    def mainloop(self):
        self.iteration.run_iteration()

        self.prev_Cd0 = self.iteration.aircraft_data.data['inputs']['Cd0']
        
        while True:
            self.iteration_number += 1

            S_ref = self.get_S_ref()
            cfc_wingtail = self.get_Cfc(self.aircraft_data.data['outputs']['wing_design']['MAC'], 0.1, k=6.35e-6) # Datcom smooth paint
            cfc_fuselage = self.get_Cfc(self.l_F, 0.05, k=6.35e-6) # Datcom smooth paint
            cfc_nacelle = self.get_Cfc(self.l_nacelle, 0.1, k=6.35e-6) # Datcom smooth paint

            wingtail = 1/S_ref * cfc_wingtail * self.wingtail_FF() * self.IF_wingtail * (self.wing_wet() + self.tail_wet())
            fuselage = 1/S_ref * cfc_fuselage * self.fuselage_FF() * self.IF_fus * self.fuselage_wet()
            nacelle =  1/S_ref * cfc_nacelle * self.nacelle_FF() * self.IF_nacelle * self.nacelle_wet()
            additional_drag = self.additional_drag() / S_ref

            self.Cd0 = wingtail + fuselage + nacelle + additional_drag
            self.iteration.aircraft_data.data['inputs']['Cd0'] = self.Cd0
            self.iteration.run_iteration()

            self.curr_Cd0 = self.Cd0


            stop_condition = abs((self.curr_Cd0 - self.prev_Cd0) / self.prev_Cd0) < self.tolerance or self.iteration_number >= self.max_iterations
            if stop_condition:
                self.update_attributes()
                self.aircraft_data.save_design(self.design_file)
                break

            self.prev_Cd0 = self.curr_Cd0
        

if __name__ == '__main__':
    data = Data('design3.json')
    est = Cd0Estimation(
        aircraft_data=data,
        mission_type=MissionType.DESIGN
    )

    est.mainloop()
    print(est.Cd0)