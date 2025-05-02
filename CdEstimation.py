import numpy as np

V_to = (100*0.514444)*1.05 # Convert from knots to m/s
fuselage_length = 35 # m
kinematic_viscosity = 1.002e-6 # m^2/s, for water at 20 degrees Celsius


Reynolds_nr = V_to*fuselage_length / kinematic_viscosity
CD = 0.075 / (np.log10(Reynolds_nr) - 2)**2
print(CD)