import numpy as np 
from utils import Data

diameter_engine = 5 # m
clearance_engine_fuselage = 2 # m
clearance_enginetips = 0.0254 # m
sea_water_density = 1020 # kg/m^3


def calculate_floating_depth(length_fuselage, width_fuselage, sea_water_density):
    MTOW = aircraft_data.data['outputs']['design']['MTOM']
    depth = MTOW / (sea_water_density*length_fuselage*width_fuselage)
    return depth

# Process all designs (1 through 4)
for i in range(1, 5):

    json_file = f"design{i}.json"
    aircraft_data = Data(json_file)
    wing_type = aircraft_data.data['inputs']['wing_type']
    d_fuselage = aircraft_data.data['outputs']['general']['d_fuselage']
    l_fuselage = aircraft_data.data['outputs']['general']['l_fuselage']
    taper_ratio = aircraft_data.data['outputs']['wing_design']['taper_ratio']
    sweep_c_4 = aircraft_data.data['outputs']['wing_design']['sweep_c_4']
    dihedral = aircraft_data.data['outputs']['wing_design']['dihedral']
    sweep_x_c = aircraft_data.data['outputs']['wing_design']['sweep_x_c']
    chord_root = aircraft_data.data['outputs']['wing_design']['chord_root']
    chord_tip = aircraft_data.data['outputs']['wing_design']['chord_tip']
    y_MAC = aircraft_data.data['outputs']['wing_design']['y_MAC']
    X_LEMAC = aircraft_data.data['outputs']['wing_design']['X_LEMAC']
    X_LE = aircraft_data.data['outputs']['wing_design']['X_LE']
    n_engines = aircraft_data.data['inputs']['n_engines']


    floating_depth = calculate_floating_depth(l_fuselage, d_fuselage, sea_water_density)
    y_engines = np.zeros(int(n_engines/2))
    for j in range(int(n_engines/2)):
        if j == 0:
            y_engines[j] = 1 + (diameter_engine/2)
        else:
            y_engines[j] = y_engines[j-1] + (diameter_engine + clearance_enginetips)

    z_engines = np.zeros_like(y_engines)
    wing_tip_clearance = np.zeros_like(z_engines)
    print(floating_depth)
    if wing_type == "HIGH":
        z_engines = d_fuselage + np.tan(np.deg2rad(dihedral))*y_engines
        wing_tip_clearance = z_engines - (diameter_engine/2) - floating_depth
        print(f"Clearance of propeller tips: {wing_tip_clearance} from water in sea state 0.")
    
    


        
