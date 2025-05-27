# Based on Raymer (FY1986)

concepts = ['Design 1', 'Design 2', 'Design 3']

We =        [1252834, 1278477, 1286030] # N
V =         145 # m/s
Q =         50 #quantity
FTA =       4 #flight test aircraft

N_eng = 8 #number of engines
T_max = 57800 # engine max thrust
M_max = 0.72 # max mach number engine
T_turbine = 1450 # turbine inlet temperature
C_avionics = 0 # avionics cost

R_E = 59.10 # engineering hourly rate
R_T = 60.70 # tooling hourly rate
R_Q = 55.40 # quality control hourly rate
R_M = 50.10 # manufacturing hourly rate

for design in range(len(concepts)):
    H_E = 4.86 * We[design]**0.777 * V**0.894 * Q**0.163
    H_T = 5.99 * We[design]**0.777 * V**0.696 * Q**0.263
    H_M = 7.37 * We[design]**0.820 * V**0.484 * Q**0.641
    H_Q = 0.076 * H_M

    C_D = 45.42 * We[design]**0.630 * V**1.3 #dev support cost
    C_F = 1243.03 * We[design]**0.325 * V**0.822 * FTA**1.21 #flight test cost
    C_M_mat = 11 * We[design]**0.921 * V**0.621 * Q**0.799 #manufacturing material cost
    C_eng = N_eng * 1548*(0.043*T_max + 243.25*M_max + 0.969*T_turbine-2228) #engine cost

    C_E = H_E * R_E
    C_T = H_T * R_T
    C_Q = H_Q * R_Q
    C_M = H_M * R_M

    import matplotlib.pyplot as plt

    cost_labels = ['Dev Support', 'Flight Test', 'Material', 'Engine', 'Avionics', 'Engineering', 'Tooling', 'Quality', 'Manufacturing']
    cost_values = [
        C_D,
        C_F,
        C_M_mat,
        C_eng * N_eng,
        C_avionics * R_E * H_E,
        C_E,
        C_T,
        C_Q,
        C_M
    ]

    plt.figure(figsize=(8, 6))
    plt.bar(cost_labels, [c / 1e6 for c in cost_values])
    plt.ylabel('Cost (Million $)')
    plt.title(f'Cost Breakdown for {concepts[design]}')
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()


    C = C_D + C_F + C_M_mat + C_eng*N_eng + C_avionics * R_E*H_E + R_T*H_T + R_Q*H_Q + R_M*H_M

    print(f"Concept {concepts[design]}: {C/10**6:.2f} M$") #cost in million dollars