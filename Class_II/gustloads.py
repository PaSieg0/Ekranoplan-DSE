import numpy as np

CLalpha = 5 # rad/s

K_g = 0.88*mu/(5.3+mu) 
mu = 2*w/(rho*c*CL_alpha*g) 
w = ... # lb/in^2
def U_ref(altitude):
    U_ref = 17.07-((17.07-13.41)/15000)*altitude # m/s
    return U_ref

print (U_ref(15000)) # 17.07 m/s

V_b = V_s * (1+ (K_g*U_ref*V_c*a)/(498*w))^0.5 # m/s
