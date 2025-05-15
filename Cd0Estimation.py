import numpy as np
from utils import Data
from ClassIWeightEstimation import ClassI, MissionType, AircraftType
from Iteration import AircraftIteration
from ISA_Class import ISA
from empennage import EmpType

class Cd0Estimation:
    #TODO: this is very preliminary, need to consider form factors, IFF Cfc etc. later
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f'design{self.design_number}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type
        self.tail_type = EmpType[aircraft_data.data['inputs']['tail_type']]

        self.tolerance = 0.0000001
        self.max_iterations = 100
        self.iteration_number = 0

        self.iteration = AircraftIteration(
            aircraft_data=self.aircraft_data,
            mission_type=self.mission_type
        )
    
    def wing_wet(self) -> float:
        return 1.07*2*self.iteration.aircraft_data.data['outputs']['max']['S']
        
    def fuselage_wet(self) -> float:
        #very preliminary estimate

        return (2*np.pi*self.aircraft_data.data['outputs']['general']['l_fuselage']*self.aircraft_data.data['outputs']['general']['r_fuselage'] + 2*np.pi*self.aircraft_data.data['outputs']['general']['r_fuselage']**2)*self.aircraft_data.data['inputs']['n_fuselages']

    def tail_wet(self) -> float:
        if self.tail_type == EmpType.NONE:
            return 0
        
        if self.tail_type == EmpType.H_TAIL:
            factor = 2
        else:
            factor = 1
        #very preliminary estimate, implement actual tail area's and such later, horizontal + vertical
        return 1.05*2*self.iteration.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S'] + 1.05*2*self.iteration.aircraft_data.data['outputs']['empennage_design']['vertical_tail']['S']*factor
    
    def get_Cfc(self) -> float:
        # Calculate the Reynolds number based on the air density, velocity, and viscosity
        isa = ISA(altitude=self.iteration.aircraft_data.data['inputs']['cruise_altitude'])
        Re = isa.rho * self.iteration.aircraft_data.data['requirements']['cruise_speed']*0.5144444 * self.iteration.aircraft_data.data['outputs']['wing_design']['MAC'] / self.iteration.aircraft_data.data['viscosity_air']
        Re2 = (38.21*(self.iteration.aircraft_data.data['outputs']['wing_design']['MAC']/1e-5)**1.053)
        Re = min(Re, Re2)
        mach = isa.Mach(self.iteration.aircraft_data.data['requirements']['cruise_speed']*0.5144444)
        Cfc_lam = 1.328 / np.sqrt(Re)
        Cfc_turb = 0.455 / (np.log10(Re)**2.58*(1 + 0.144 * mach**2)**0.65)
        Cfc = 0.1*Cfc_lam + 0.9*Cfc_turb
        return Cfc
    
    def get_S_ref(self) -> float:
        # implement actual tail areas later
        if self.tail_type == EmpType.NONE:
            horizontal_tail_area = 0
        else:
            horizontal_tail_area = self.iteration.aircraft_data.data['outputs']['empennage_design']['horizontal_tail']['S']
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

            wing_wet = self.wing_wet()
            tail_wet = self.tail_wet()
            fuselage_wet = self.fuselage_wet()
            S_ref = self.get_S_ref()
            coefficient = self.get_Cfc()
            print(wing_wet, tail_wet, fuselage_wet, S_ref, coefficient)

            self.Cd0 = coefficient*(wing_wet + tail_wet + fuselage_wet)/S_ref * 1.2
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
    data = Data('design2.json')
    est = Cd0Estimation(
        aircraft_data=data,
        mission_type=MissionType.DESIGN
    )

    est.mainloop()
    print(est.Cd0)