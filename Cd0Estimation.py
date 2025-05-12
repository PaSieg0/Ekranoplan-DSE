import numpy as np
from utils import Data
from ClassIWeightEstimation import ClassI, MissionType, AircraftType
from Iteration import AircraftIteration


class Cd0Estimation:
    #TODO: this is very preliminary, need to consider form factors, IFF Cfc etc. later
    def __init__(self, aircraft_data: Data, mission_type: MissionType) -> None:
        self.design_number = aircraft_data.data['design_id']
        self.design_file = f'design{self.design_number}.json'
        self.aircraft_data = aircraft_data
        self.mission_type = mission_type

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
        #very preliminary estimate, implement actual tail area's and such later, horizontal + vertical
        return 3*1.05*2*0.073*self.iteration.aircraft_data.data['outputs']['max']['S']
    
    def get_Cfc(self) -> float:
        # Calculate the skin friction coefficient based on the Reynolds number
        '''Re = self.iteration.aircraft_data.data['rho_water'] * self.iteration.aircraft_data.data['V'] * self.iteration.aircraft_data.data['L'] / self.iteration.aircraft_data.data['mu']
        Cfc = 0.455 / (np.log10(Re)**2.58)'''
        #return statistical value now, implement actual calculation later
        return 0.005
    
    def get_S_ref(self) -> float:
        # implement actual tail areas later
        return self.iteration.aircraft_data.data['outputs']['max']['S'] + 0.073*2*self.iteration.aircraft_data.data['outputs']['max']['S'] 
    
    def update_attributes(self):
            mission_type = self.mission_type.name.lower()
            self.aircraft_data.data['outputs'][mission_type]['MTOM'] = self.iteration.aircraft_data.data['outputs'][mission_type]['MTOM']
            self.aircraft_data.data['outputs'][mission_type]['MTOW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['MTOW']
            self.aircraft_data.data['outputs'][mission_type]['OEW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['OEW']
            self.aircraft_data.data['outputs'][mission_type]['ZFW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['ZFW']
            self.aircraft_data.data['outputs'][mission_type]['EW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['EW'] 
            self.aircraft_data.data['outputs'][mission_type]['total_fuel'] = self.iteration.aircraft_data.data['outputs'][mission_type]['total_fuel']
            self.aircraft_data.data['outputs'][mission_type]['fuel_mission'] = self.iteration.aircraft_data.data['outputs'][mission_type]['fuel_mission']
            self.aircraft_data.data['outputs'][mission_type]['fuel_reserve'] = self.iteration.aircraft_data.data['outputs'][mission_type]['fuel_reserve'] 
            self.aircraft_data.data['outputs'][mission_type]['S'] = self.iteration.aircraft_data.data['outputs'][mission_type]['S']
            self.aircraft_data.data['outputs'][mission_type]['b'] = self.iteration.aircraft_data.data['outputs'][mission_type]['b']
            self.aircraft_data.data['outputs'][mission_type]['MAC'] = self.iteration.aircraft_data.data['outputs'][mission_type]['S'] / self.iteration.aircraft_data.data['outputs'][mission_type]['b']
            self.aircraft_data.data['outputs'][mission_type]['h_b'] = self.iteration.aircraft_data.data['outputs'][mission_type]['h_b']
            self.aircraft_data.data['outputs'][mission_type]['k'] = self.iteration.aircraft_data.data['outputs'][mission_type]['k']
            self.aircraft_data.data['outputs'][mission_type]['WP'] = self.iteration.aircraft_data.data['outputs'][mission_type]['WP']
            self.aircraft_data.data['outputs'][mission_type]['TW'] = self.iteration.aircraft_data.data['outputs'][mission_type]['TW']
            self.aircraft_data.data['outputs'][mission_type]['WS'] = self.iteration.aircraft_data.data['outputs'][mission_type]['WS']
            
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

    def mainloop(self):
        self.iteration.run_iteration()

        self.prev_Cd0 = self.iteration.aircraft_data.data['inputs']['Cd0']
        
        while True:
            print(self.prev_Cd0)
            self.iteration_number += 1

            wing_wet = self.wing_wet()
            tail_wet = self.tail_wet()
            fuselage_wet = self.fuselage_wet()
            S_ref = self.get_S_ref()
            coefficient = self.get_Cfc()

            self.Cd0 = coefficient*(wing_wet + tail_wet + fuselage_wet)/S_ref * 1.2
            self.iteration.aircraft_data.data['inputs']['Cd0'] = self.Cd0
            self.iteration.run_iteration()

            self.curr_Cd0 = self.Cd0

            stop_condition = abs((self.curr_Cd0 - self.prev_Cd0) / self.prev_Cd0) < self.tolerance or self.iteration_number >= self.max_iterations
            print(stop_condition)
            print(abs((self.curr_Cd0 - self.prev_Cd0) / self.prev_Cd0))
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