import math
#            SS3        SS4
H_S =       [1.75,      2.25] #m
lambda_m =  [160,       160] # m
R =         [7400E3,    8E3] # m


P_T = [1E-2,1E-3] #probability 



def minimum_height(H_S,lambda_m,R,P):
    return H_S/2*math.sqrt(-2*math.log(1-(1-P)**(lambda_m/R)))

for i in range(len(H_S)):
    for P in P_T:
        min_h = minimum_height(H_S[i],lambda_m[i],R[i],P)
        print(f"SS{i+3} P={P:.0E} m, H={min_h:.2f} m")