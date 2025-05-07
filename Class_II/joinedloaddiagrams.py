from maneuverloads import calculate_n, calculante_n_limits, max_n, min_n
from gustloads import point_determination, Calculate_K_g, Calculate_mu, Calculate_V_b, Calculate_U_ref

import numpy as np
import matplotlib.pyplot as plt
from utils import Data
from ISA_Class import ISA


def plot_complete_load_diagram(aircraft_data, h):
    # ====== Gust Load Point Calculation ======
    g = 9.80665  # m/s²
    W_final = aircraft_data.data["MTOM"] * g
    U_ref = Calculate_U_ref(h / 0.3048)  # Altitude in ft
    S = W_final / aircraft_data.data["WS"]
    rho = ISA(h).rho
    CL_alpha = aircraft_data.data["Cl_alpha"]
    w = aircraft_data.data["WS"]  # N/m^2
    w = w * 2.20462262 / (g * 3.2808399**2)  # lb/ft^2
    b = np.sqrt(aircraft_data.data["aspect_ratio"] * S)
    chord = S / b
    mu = Calculate_mu(w, rho * 0.0019403203319541, chord, CL_alpha, g)
    K_g = Calculate_K_g(mu)
    V_b = Calculate_V_b(
        aircraft_data.data["stall_speed_clean"],
        K_g, U_ref,
        aircraft_data.data["cruise_speed"],
        CL_alpha, w
    )
    dive_speed = aircraft_data.data["cruise_speed"] / 0.8  # m/s, TBD

    gust_data = [
        [float(V_b), U_ref],
        [aircraft_data.data["cruise_speed"], U_ref],
        [dive_speed, U_ref / 2]
    ]

    for i, (V_eas, U_gust) in enumerate(gust_data):
        delta_n = rho * V_eas * CL_alpha * U_gust * S / (2 * W_final)
        gust_data[i].append(float(delta_n))

    # ====== Maneuver Load Calculation ======
    CLmax_clean = aircraft_data.data["CLmax_clean"]
    W = W_final
    nmax = max_n(W)
    nmin = min_n()
    V_cruise = aircraft_data.data["cruise_speed"]
    V_dive = V_cruise / 0.8
    V_range = np.arange(0, V_dive, 0.1)
    n_positive = [calculante_n_limits(rho, CLmax_clean, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]
    n_negative = [calculante_n_limits(rho, -CLmax_clean, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]

    # ====== Plotting ======
    plt.figure(figsize=(10, 6))
    plt.title("Load Diagram with Gust Envelope")
    plt.xlabel("Velocity (V) [m/s]")
    plt.ylabel("Load Factor (n)")

    # Maneuver envelope
    plt.plot(V_range, n_positive, label="Maneuver Envelope", color="blue")
    plt.plot(V_range, n_negative, color="blue")
    plt.plot([V_dive, V_dive], [0, nmax], color='blue', linestyle='-', linewidth=1.5)

    # ====== Gust Envelope ======
    pos_V, pos_n = [0.0], [1.0]
    neg_V, neg_n = [0.0], [1.0]
    for V_eas, _, delta_n in gust_data:
        pos_V.append(V_eas)
        pos_n.append(1.0 + delta_n)
        neg_V.append(V_eas)
        neg_n.append(1.0 - delta_n)

    order = np.argsort(pos_V)
    pos_V, pos_n = np.array(pos_V)[order], np.array(pos_n)[order]
    neg_V, neg_n = np.array(neg_V)[order], np.array(neg_n)[order]

    plt.plot(pos_V, pos_n, color="black", label="Gust Envelope")
    plt.plot(neg_V, neg_n, color="black")

    max_V = pos_V[-1]
    plt.plot([max_V, max_V], [neg_n[-1], pos_n[-1]], color="black")

    # ====== Shading the Union ======
    for i, n in enumerate(n_positive):
        if n >= 1:
            V_at_n1 = V_range[i]
            break
    V_combined = np.linspace(V_at_n1, V_dive, 1000)
    n_maneuver_positive = np.interp(V_combined, V_range, n_positive)
    n_maneuver_negative = np.interp(V_combined, V_range, n_negative)
    n_gust_positive = np.interp(V_combined, pos_V, pos_n)
    n_gust_negative = np.interp(V_combined, neg_V, neg_n)

    n_upper = np.maximum(n_maneuver_positive, n_gust_positive)
    n_lower = np.minimum(n_maneuver_negative, n_gust_negative)

    plt.fill_between(V_combined, n_lower, n_upper, color="lightgreen", alpha=0.5, label="Allowable Condition")

    # Annotate vertical limits for the green region
    max_n_allowable = np.max(n_upper)
    min_n_allowable = np.min(n_lower)
    plt.axhline(max_n_allowable, color="green", linestyle="--", linewidth=1.2, label=f"Max Allowable n = {max_n_allowable:.2f}")
    plt.axhline(min_n_allowable, color="green", linestyle="--", linewidth=1.2, label=f"Min Allowable n = {min_n_allowable:.2f}")

    plt.grid(True, linestyle=":", alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Load the aircraft data
    aircraft_data = Data("design1.json")
    
    # Plot the load diagram
    plot_complete_load_diagram(aircraft_data, h=0)