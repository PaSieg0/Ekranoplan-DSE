import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
import matplotlib.pyplot as plt

aircraft_data = Data('final_design.json') #TODO check if all components are here from main, otherwise fix pls
print(aircraft_data.data)
Masses = aircraft_data.data['outputs']['component_weights'] #{ add cargo and fuel
            # "anti_ice": 3980.672971974585,
            # "apu_installed": 6852.601370538002,
            # "avionics": 9528.503770036557,
            # "electrical": 7279.988998921629,
            # "furnishings": 33372.840217730416,
            # "fuselage": 170356.87605933566,
            # "handling_gear": 597.1009457961878,
            # "horizontal_tail": 24853.765242920694,
            # "instruments": 3059.3933270163357,
            # "nacelle_group": 29832.405191728256,
            # "starter_pneumatic": 1316.387415661655,
            # "vertical_tail": 13094.386615428253,
            # "wing": 199622.38214452507,
            # "engine": 161263.78945732486,
            # "engine_controls": 836.9498784129045,
            # "fuel_system": 1981.219047887346,
            # "flight_control": 1875.2357236660441,
            # "military_cargo_handling_system": 22631.128739693377,
            # "door": 30411.0,
            # "anchor": 34040.7,
            # "floater": 72840.84547854812,
            # "total_fuel": 270581.77749993064,
            # "cargo_mass": 90000*9.81
            # }
print(Masses)
Masses.pop('total_OEW')


Masses_new={}



Masses_new['wing']=Masses['wing']/9.81
Masses_new['fuselage']=Masses['fuselage']/9.81
Masses_new['nacelle_group']=Masses['nacelle_group']/9.81
Masses_new['engine']=Masses['engine']/9.81
Masses_new['misscelaneous']=(Masses['anti_ice']+Masses['apu_installed']+Masses['avionics']+Masses['electrical']+Masses['furnishings']+Masses['handling_gear']+Masses['instruments']+Masses['starter_pneumatic']+Masses['engine_controls']+Masses['fuel_system']+Masses['flight_control']+Masses['military_cargo_handling_system'])/9.81
Masses_new['empennage']=(Masses['horizontal_tail']+Masses['vertical_tail'])/9.81
Masses_new['door']=Masses['door']/9.81
Masses_new['anchor']=Masses['anchor']/9.81
Masses_new['floater']=(Masses['floater']+Masses['floater_endplate'])/9.81
# Masses_new['cargo_mass']=90000
# Masses_new['fuel']=aircraft_data.data['outputs']['max']['max_fuel']/9.81



print(Masses_new)
print(sum(Masses_new.values())*9.81)

plt.pie(list(Masses_new.values()), labels=Masses_new.keys())
plt.show()