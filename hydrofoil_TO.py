import matplotlib.pyplot as plt

rho_w = 1020  # kg/m^3
rho_a = 1.225  # kg/m^3
g = 9.81  # m/s^2

N_fus = 2  # Number of fuselages

if N_fus ==1:
    l_tail = 39.6  # m (tail cone length)
    l_fus = 74.9  # m (fuselage length)
    d = 8.8  # m (fuselage diameter)
elif N_fus == 2:
    l_tail = 27.9  # m (tail cone length)
    l_fus = 52
    d = 6.2 # m (fuselage diameter)

W = 3476117 # N
P_max = 37600000 # W
tau_list = [0.5,0.55,0.6,0.65,0.7] # Thrust required for lift-off
tau_list = [0.55]

colors = [
    "#ADD8E6",  # Light Blue
    "#87CEEB",  # Sky Blue
    "#4682B4",  # Steel Blue
    "#4169E1",  # Royal Blue
    "#00008B",  # Dark Blue
]

colors = ["#4682B4"]


C_F = 0.001269  # Friction coefficient for hydrofoil
C_L_f = 1.0  # Lift coefficient for hydrofoil
C_L = 1.7  # Lift coefficient for airfoil
LD = 16  # Lift-to-drag ratio for airfoil
C_D0_f = 0.05  # Drag coefficient for hydrofoil

S = 900  # m^2 (Main wing area)
S_fus = N_fus*d*(l_fus-l_tail) # m^2 (Fuselage area)

print(f"S_fus: {S_fus:.2f} m^2")
print(f"S: {S:.2f} m^2")

V_TO = 65


def calculate_P(V):
    P_hull = C_F * 1/2 * rho_w * S_fus * V**3
    P_hydrofoil = 1/2*V**3*(C_L/LD*rho_a*S + C_D0_f*rho_w*S_f)
    return P_hull, P_hydrofoil

counter = 0
for tau in tau_list:

    V_LO = ((tau * P_max) / ((C_F / 2)* rho_w * S_fus))**(1/3)


    S_f = 1/(C_L_f*rho_w)*((2*W)/(V_LO**2)-C_L*rho_a*S)

    P_TO = 1/2*V_TO**3*(C_L/LD*rho_a*S + C_D0_f*rho_w*S_f)
    
    P_hull_total_w = []
    P_hydrofoil_total_w = []
    P_hull_total_a = []
    P_hydrofoil_total_a = []
    velocities_a = []
    velocities_w = []
    for pV in range(1000):
        V = pV/1000 * V_TO

        P_hull, P_hydrofoil = calculate_P(V)

        #print(f"V: {V:.2f} m/s, P_hull: {P_hull:.2f} W, P_hydrofoil: {P_hydrofoil:.2f} W")

        if V < V_LO:
            #print(f'water take-off velocity: {V:.2f} m/s {V_LO:.2f} m/s')
            velocities_w.append(V)
            P_hull_total_w.append(P_hull)
            P_hydrofoil_total_w.append(P_hydrofoil)
        else: 
            #print(f'air take-off velocity: {V:.2f} m/s')
            velocities_a.append(V)
            P_hull_total_a.append(P_hull)
            P_hydrofoil_total_a.append(P_hydrofoil)



    plt.plot(velocities_w, P_hull_total_w, color=colors[counter], label=f'tau={tau}')
    plt.plot(velocities_a, P_hydrofoil_total_a, color=colors[counter])
    plt.plot(velocities_a, P_hull_total_a, linestyle='--', color=colors[counter])
    plt.plot(velocities_w, P_hydrofoil_total_w, linestyle='--', color=colors[counter])
    if V_LO < V_TO:
        plt.plot([V_LO, V_LO], [P_hull_total_w[-1],P_hydrofoil_total_a[0]], color=colors[counter])
    #plt.plot(velocities, P_TO, linestyle='--', label='P_TO')
    counter += 1

    print(f"tau: {tau:.2f}, V_LO: {V_LO:.2f} m/s, S_f: {S_f:.2f} m^2")
    print(f"P_hull: {P_hull/10**6:.2f} MW, P_hydrofoil: {P_hydrofoil/10**6:.2f} MW, reduction: {(P_hull-P_hydrofoil)/P_hull*100:.2f} %")

plt.xlabel("Velocity (V) [m/s]")
plt.ylabel("Power (P) [W]")
plt.title("Power vs Velocity")
plt.grid()
plt.legend()
plt.show()