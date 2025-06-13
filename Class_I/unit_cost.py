import numpy as np
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
from matplotlib import pyplot as plt
import json



'''Unit Cost'''
def calculate_unit_cost(aircraft_data: Data, plot=True):
    We =        aircraft_data.data["outputs"]["design"]["EW"]*0.2248089431 # lb
    V =         aircraft_data.data["outputs"]['optimum_speeds']['max']/0.51444444 # kts
    Q =         50 # Quantity
    FTA =       4 # Flight test aircraft

    N_eng = aircraft_data.data["inputs"]["n_engines"] # Number of engines
    T_max = 110000*0.2248089431 # Engine max thrust lb
    M_max = 0.84 # Max mach number engine
    T_turbine = 1450*1.8 # Turbine inlet temperature in Rankine


    R_E = 59.10 # Engineering hourly rate
    R_T = 60.70 # Tooling hourly rate
    R_Q = 55.40 # Quality control hourly rate
    R_M = 50.10 # Manufacturing hourly rate


    H_E = 4.86 * We**0.777 * V**0.894 * Q**0.163 # Engineering hours
    H_T = 5.99 * We**0.777 * V**0.696 * Q**0.263 # Tooling hours
    H_M = 7.37 * We**0.820 * V**0.484 * Q**0.641 # Manufacturing hours
    H_Q = 0.076 * H_M

    C_D = 45.42 * We**0.630 * V**1.3 # Dev support cost
    C_F = 1243.03 * We**0.325 * V**0.822 * FTA**1.21 # Flight test cost
    C_M_mat = 11 * We**0.921 * V**0.621 * Q**0.799 # Manufacturing material cost
    C_eng = N_eng * 1548*(0.043*T_max + 243.25*M_max + 0.969*T_turbine-2228) # Engine cost
    C_avionics = 10000000 # Avionics cost

    C_E = H_E * R_E
    C_T = H_T * R_T
    C_Q = H_Q * R_Q
    C_M = H_M * R_M


    cost_labels = ['Dev Support', 'Flight Test', 'Material', 'Engine', 'Avionics', 'Engineering', 'Tooling', 'Quality', 'Manufacturing']
    cost_values = [
        C_D,
        C_F,
        C_M_mat,
        C_eng,
        C_avionics,
        C_E,
        C_T,
        C_Q,
        C_M
    ]
    if plot:
        plt.figure(figsize=(8, 6))
        plt.bar(cost_labels, [c / 1e6 for c in cost_values])
        plt.ylabel('Cost (Million $)')
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.show()


    C = (C_D + C_F + C_M_mat + C_eng*N_eng + C_avionics + R_T*H_T + R_Q*H_Q + R_M*H_M)*2.92

    unit_costs = {
        "cost_with_inflation_correction": (C/1e6),
        "unit_cost_with_inflation_correction": (C/1e6)/(Q+FTA)
    }

    return unit_costs, cost_values

'''Operational Cost'''

def calculate_operational_cost(aircraft_data: Data):
    time_design_mission = 12  # h
    time_design_mission_c5 = 5 # h
    time_design_mission_c17 = 5.38 # h 
    time_design_mission_c130 = 9.43 # h

    amount_design_missions = 50 # Assume about 50 design missions per year
   
    flighthours_year = amount_design_missions * time_design_mission 
    flighthours_year_c5 = 760 
    flighthours_year_c17 = 7800
    flighthours_year_c130 = 720

    amount_design_missions_c5 = flighthours_year_c5 / time_design_mission_c5
    amount_design_missions_c17 = flighthours_year_c17 / time_design_mission_c17
    amount_design_missions_c130 = flighthours_year_c130 / time_design_mission_c130
    
    
    density_SAF = 0.76  # kg/liter
    price_SAF = 1.18    # $/liter
    density_kerosene = 0.82 # kg/liter
    price_kerosene = 1.11 # $/liter    
    fuel_mass_design_mission = aircraft_data.data['outputs']['design']['total_fuel'] / 9.81  # kg
    fuel_mass_c130 = 20108 # kg
    fuel_volume_design = fuel_mass_design_mission / density_SAF # liters
    fuel_volume_c5 = 194370 # liters
    fuel_volume_c17 = 134560 # liters 
    fuel_volume_c130 = fuel_mass_c130 / density_kerosene # liters
   
    fuel_price = fuel_volume_design * price_SAF
    fuel_price_c5 = fuel_volume_c5 * price_kerosene
    fuel_price_c17 = fuel_volume_c17 * price_kerosene
    fuel_price_c130 = fuel_volume_c130 * price_kerosene

    cruise_speed = aircraft_data.data['requirements']['cruise_speed'] / 0.5144444  # kts
    MTOM = aircraft_data.data['outputs']['design']['MTOM'] * 0.2248089431  # lb

    cruise_speed_c5 = 869*0.539956803 # kts
    MTOM_c5 = 840000 # lb

    cruise_speed_c17 = 830*0.539956803 # kts
    MTOM_c17 = 585000 # lb

    cruise_speed_c130 = 602*0.539956803 # kts
    MTOM_c130 = 70305** 0.2248089431  # lb

    # n crew = 4. Formula is for n crew = 2, so multiply by 2
    crew_cost_hour = 2.92 * 2 * (35 * (cruise_speed * (MTOM / (10**5)))**0.3 + 84) # With inflation correction factor of 2.92
    crew_cost = crew_cost_hour * time_design_mission

    # n crew = 7
    crew_cost_hour_c5 =  2.92 * 3.5 * (35 * (cruise_speed_c5 * (MTOM_c5 / (10**5)))**0.3 + 84) # With inflation correction factor of 2.92
    crew_cost_c5 = crew_cost_hour_c5 * time_design_mission_c5

    # n crew = 3
    crew_cost_hour_c17 =  2.92 * (47 * (cruise_speed_c17 * (MTOM_c17 / (10**5)))**0.3 + 118) # With inflation correction factor of 2.92
    crew_cost_c17 = crew_cost_hour_c17 * time_design_mission_c17

    # n crew = 5 
    crew_cost_hour_c130 = 2.92 * (47 * (cruise_speed_c130 * (MTOM_c130 / (10**5)))**0.3 + 118) + (2.92 * (35 * (cruise_speed_c130 * (MTOM_c130 / (10**5)))**0.3 + 84))
    crew_cost_c130 = crew_cost_hour_c130 * time_design_mission_c130

    MMH_FH = 41   # Maintenance Man Hours per Flight Hour, conservative choice of about 1.5 the hours as C5 due to marine environment
    labor_wrap_rate = 140  # $/hour
    maintenance_cost = (flighthours_year * MMH_FH * labor_wrap_rate) / amount_design_missions
    maintenance_cost_c5 = (flighthours_year_c5 * MMH_FH * labor_wrap_rate) / amount_design_missions_c5
    maintenance_cost_c17 = (flighthours_year_c17 * MMH_FH * labor_wrap_rate) / amount_design_missions_c17
    maintenance_cost_c130 = (flighthours_year_c130 * MMH_FH * labor_wrap_rate) / amount_design_missions_c130
   
    operational_cost_design_mission = crew_cost + fuel_price #+ (maintenance_cost*1.5)
    operational_cost_design_mission_per_hr = operational_cost_design_mission / time_design_mission

    operational_cost_design_mission_c5 = crew_cost_c5 +  fuel_price_c5 #+ maintenance_cost_c5
    operational_cost_design_mission_per_hr_c5 = operational_cost_design_mission_c5 / time_design_mission_c5

    operational_cost_design_mission_c17 = crew_cost_c17 + fuel_price_c17 #+ maintenance_cost_c17
    operational_cost_design_mission_per_hr_c17 = operational_cost_design_mission_c17 / time_design_mission_c17

    operational_cost_design_mission_c130 = crew_cost_c130 + fuel_price_c130 #+ maintenance_cost_c130
    operational_cost_design_mission_per_hr_c130 = operational_cost_design_mission_c130 / time_design_mission_c130


    design_payload = aircraft_data.data['requirements']['design_payload'] / 1000  # Design payload in tonnes
    design_range = aircraft_data.data['requirements']['design_range'] / 1000          # Design range in km
    operational_cost_payload = operational_cost_design_mission / (design_payload * design_range)

    design_payload_c5 = 122.5 # tonnes
    design_range_c5 = 3982 # km 
    operational_cost_payload_c5 = operational_cost_design_mission_c5 / (design_payload_c5 * design_range_c5)

    design_payload_c17 = 77.519 # tonnes
    design_range_c17 = 4480 # km 
    operational_cost_payload_c17 = operational_cost_design_mission_c17 / (design_payload_c17 * design_range_c17)

    design_payload_c130 = 15.876 # tonnes
    design_range_c130 = 5245 # km 
    operational_cost_payload_c130 = operational_cost_design_mission_c130 / (design_payload_c130 * design_range_c130)


    # Collect all outputs in a dictionary.
    operational_costs = {
        "fuel_price_design_mission": fuel_price,
        "crew_cost_design_mission": crew_cost,
        "maintenance_cost_per_mission": maintenance_cost,
        "total_operational_cost_design_mission": operational_cost_design_mission,
        "operational_cost_design_mission_per_hr": operational_cost_design_mission_per_hr,
        "operational_cost_per_tonne_km": operational_cost_payload,
    }

    operational_costs_c5 = {
        "fuel_price_design_mission": fuel_price_c5,
        "crew_cost_design_mission": crew_cost_c5,
        "maintenance_cost_per_mission": maintenance_cost_c5,
        "total_operational_cost_design_mission": operational_cost_design_mission_c5,
        "operational_cost_design_mission_per_hr": operational_cost_design_mission_per_hr_c5,
        "operational_cost_per_tonne_km": operational_cost_payload_c5,
    }

    operational_costs_c17 = {
        "fuel_price_design_mission": fuel_price_c17,
        "crew_cost_design_mission": crew_cost_c17,
        "maintenance_cost_per_mission": maintenance_cost_c17,
        "total_operational_cost_design_mission": operational_cost_design_mission_c17,
        "operational_cost_design_mission_per_hr": operational_cost_design_mission_per_hr_c17,
        "operational_cost_per_tonne_km": operational_cost_payload_c17,
    }

    operational_costs_c130 = {
        "fuel_price_design_mission": fuel_price_c130,
        "crew_cost_design_mission": crew_cost_c130,
        "maintenance_cost_per_mission": maintenance_cost_c130,
        "total_operational_cost_design_mission": operational_cost_design_mission_c130,
        "operational_cost_design_mission_per_hr": operational_cost_design_mission_per_hr_c130,
        "operational_cost_per_tonne_km": operational_cost_payload_c130,
    }

    return operational_costs, operational_costs_c5, operational_costs_c17, operational_costs_c130

def main_cost(aircraft_data: Data, plot=True, design_file = "design3.json"):
    unit_costs = calculate_unit_cost(aircraft_data,plot)[0]  
    operational_costs, operational_costs_c5, operational_costs_c17, operational_costs_c130 = calculate_operational_cost(aircraft_data)  
    aircraft_data.data["outputs"]["costs"] = {
        "unit_costs": unit_costs,
        "operational_costs": operational_costs,
        "operational_costs_c5": operational_costs_c5,
        "operational_costs_c17": operational_costs_c17,
        "operational_costs_c130": operational_costs_c130
    }

    aircraft_data.save_design(design_file=design_file)


if __name__ == "__main__":
    aircraft_data = Data("design3.json")  # Load the design data
    main_cost(aircraft_data=aircraft_data, design_file="design3.json")

    print("Design data has been updated with costs!")
    a , b = calculate_unit_cost(aircraft_data)
    print(b)

    