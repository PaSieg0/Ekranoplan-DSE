import matplotlib.pyplot as plt

# Constants and inputs
CL_max_hfoil = 1.0  # Max lift coefficient for hydrofoil
CD_hfoil = 0.05      # Assumed drag coefficient for hydrofoil
CL_max = 1.6        # Max lift coefficient for airfoil
CL_max_h = -1.0      # Max lift coefficient for horizontal stabilizer

rho_water = 1020    # kg/m^3
rho_air = 1.225     # kg/m^3

Shfoil = 1.5        # m^2
S = 850             # Main wing area (m^2)
#Sh = 200            # Horizontal stabilizer area (m^2)

g = 9.81            # m/s^2
V_liftoff = 65      # m/s

TOW = 350000        # kg

# Geometry
chord = 10          # m
h_cruise = 5        # m
L_fus = 75          # m
x_wing = 30         # m
x_h = 70            # m
x_cg = 37           # m

x_hydrofoil = [0.2 * L_fus]
z_hydrofoil = [30]  # m

# Force calculation functions
def lift_force(rho, V, area, CL):
    return -0.5 * rho * V**2 * CL * area

def drag_force(rho, V, area, CD):
    return 0.5 * rho * V**2 * CD * area

def calculate_moment(force_x, force_z, x, z):
    return force_x * z - force_z * x

# Store results for plotting
ShS_values = []
M_total_x_wing_values = []
total_lift = []
weight =[]
velocities = []

for x in range(1000):
    V = x/1000 * V_liftoff  # Velocity in m/s

    # Calculate lift and drag forces for the main wing
    L_wing = lift_force(rho_air, V, S, CL_max)

    L_hfoil = lift_force(rho_water, V, Shfoil, CL_max_hfoil)  # Lift on hydrofoils

    velocities.append(V)
    total_lift.append(-L_wing - L_hfoil)
    weight.append(TOW * g)

plt.plot(velocities, total_lift)
plt.plot(velocities, weight)
plt.xlabel('Force (N)')
plt.ylabel('Velocity (m/s)')
plt.title('Lift and Weight Forces vs Velocity')

plt.grid()
plt.show()


for x in range(1000):
    ShS = x/1000
    Sh = ShS * S  # Horizontal stabilizer area (m^2)

    # Weight moment
    M_W = calculate_moment(0, TOW * g, x_cg, 0)
    #print(f"Weight moment: {M_W:.2f} Nm")

    # Wing lift moment
    L_wing = lift_force(rho_air, V_liftoff, S, CL_max)
    M_L = calculate_moment(0, L_wing, x_wing, 0)
    #print(f"Wing lift moment: {M_L:.2f} Nm")

    # Horizontal stabilizer lift moment
    L_hstab = lift_force(rho_air, V_liftoff, Sh, CL_max_h)
    M_H = calculate_moment(0, L_hstab, x_h, 0)
    #print(f"Horizontal stabilizer lift moment: {M_H:.2f} Nm")

    # Hydrofoils lift + drag moments

    M_hfoil = 0
    for i in range(len(x_hydrofoil)):
        L_hf = lift_force(rho_water, V_liftoff, Shfoil, CL_max_hfoil)
        D_hf = drag_force(rho_water, V_liftoff, Shfoil, CD_hfoil)
        M_hfoil += calculate_moment(D_hf, L_hf, x_hydrofoil[i], z_hydrofoil[i])
    #print(f"Hydrofoil lift + drag moment: {M_hfoil:.2f} Nm")

    # Total moment
    M_total = M_W + M_L + M_H + M_hfoil

    # Calculate the moment around x_wing
    M_total_x_wing = M_total + TOW * g * (x_cg - x_wing) + L_wing * (x_wing - x_wing) + L_hstab * (x_h - x_wing)




    ShS_values.append(ShS)
    M_total_x_wing_values.append(M_total_x_wing)

    print(f"Stabilizer: {ShS} --> Total moment: {M_total_x_wing:.2f} Nm")

# Plot the results
plt.plot(ShS_values, M_total_x_wing_values)
plt.xlabel('Sh/S (Stabilizer Area Ratio)')
plt.ylabel('Total Moment around x_wing (Nm)')
plt.title('Total Moment vs Stabilizer Area Ratio')
plt.grid()
plt.show()