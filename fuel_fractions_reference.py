import numpy as np

def fuel_fraction(R, c, V_eta, L_D):
    return (1-np.exp(-R*9.81*c/V_eta/L_D)) / 0.9

def fuel_used(fraction, MTOM):
    return fraction*MTOM

def fuel_economy(fuel_mass, range, payload):
    return fuel_mass/0.82/(range/1000)/(payload/1000)

C5 = fuel_fraction(2200*1.852*1000, 8.9e-6, 241.4, 11.28)
C17 = fuel_fraction(4480000, 9.3e-6, 230.55, 10.77)
C130 = fuel_fraction(3789000, 9.22e-8, 0.82, 14.13)

print(C5)
print(C17)
print(C130)

C5_mass = fuel_used(C5, 397655)
C17_mass = fuel_used(C17, 265352)
C130_mass = fuel_used(C130, 70305)

print(C5_mass/0.82)
print(C17_mass/0.82)
print(C130_mass/0.82)

C5_eco = fuel_economy(C5_mass, 3981800, 122500)
C17_eco = fuel_economy(C17_mass, 4480000, 71214)
C130_eco = fuel_economy(C130_mass, 3789000, 19356)

print(C5_eco)
print(C17_eco)
print(C130_eco)