import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Class_II.maneuverloads import calculate_n_limits, max_n, min_n, calculate_n
from Class_II.gustloads import Calculate_K_g, Calculate_mu, Calculate_V_b, Calculate_U_ref
from utils import Data, ISA

class LoadDiagram:
    def __init__(self, aircraft_data: Data):
        self.aircraft_data = aircraft_data
        self.h = 0  # Default altitude in meters

    def plot_complete_load_diagram(self, plot=False):
        # ====== Gust Load Point Calculation ======
        g = 9.80665  # m/s²
        W_final = self.aircraft_data.data["outputs"]["design"]["MTOW"]
        U_ref = Calculate_U_ref(self.h / 0.3048)  # Altitude in ft
        S = self.aircraft_data.data["outputs"]["design"]["S"]
        rho = ISA(self.h).rho
        CL_alpha = 5 #            !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!     HARDOCDED FOR NOW
        w = W_final/S  # N/m^2
        w = w * 2.20462262 / (g * 3.2808399**2)  # lb/ft^2
        b = np.sqrt(self.aircraft_data.data["inputs"]["aspect_ratio"] * S)
        chord = S / b
        mu = Calculate_mu(w, rho * 0.0019403203319541, chord, CL_alpha, g)
        K_g = Calculate_K_g(mu)
        V_b = Calculate_V_b(
            self.aircraft_data.data["requirements"]["stall_speed_clean"],
            K_g, U_ref,
            self.aircraft_data.data["requirements"]["cruise_speed"],
            CL_alpha, w
        )
        dive_speed = self.aircraft_data.data["requirements"]["cruise_speed"] / 0.8  # m/s, TBD

        gust_data = [
            [float(V_b), U_ref],
            [self.aircraft_data.data["requirements"]["cruise_speed"], U_ref],
            [dive_speed, U_ref / 2]
        ]

        for i, (V_eas, U_gust) in enumerate(gust_data):
            delta_n = rho * V_eas * CL_alpha * U_gust * S / (2 * W_final)
            gust_data[i].append(float(delta_n))

        # ====== Maneuver Load Calculation ======
        CLmax_clean = self.aircraft_data.data["inputs"]["CLmax_clean"]
        W = W_final
        nmax = max_n(W)
        nmin = min_n()
        V_cruise = self.aircraft_data.data["requirements"]["cruise_speed"]
        V_dive = V_cruise / 0.8
        V_range = np.arange(0, V_dive, 0.1)
        n_positive = [calculate_n_limits(rho, CLmax_clean, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]
        n_stall_positive = [calculate_n(rho, CLmax_clean, W, S, V) for V in V_range]

        n_negative = [calculate_n_limits(rho, -CLmax_clean, W, S, V, nmax, nmin, V_cruise, V_dive) for V in V_range]
        n_stall_negative = [calculate_n(rho, -CLmax_clean, W, S, V) for V in V_range]

        # ====== Plotting ======
        plt.figure(figsize=(10, 6))
        plt.title("Load Diagram with Gust Envelope")
        plt.xlabel("Velocity (V) [m/s]")
        plt.ylabel("Load Factor (n)")

        # Maneuver envelope
        plt.plot(V_range, n_positive, label="Maneuver Envelope", color="blue")
        plt.plot(V_range, n_stall_positive, color="blue", linestyle=":")
        plt.plot(V_range, n_negative, color="blue")
        plt.plot(V_range, n_stall_negative, color="blue", linestyle=":")

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
        # Find the velocity where n_maneuver_positive first reaches 2.5
        for i, n in enumerate(n_positive):
            if n >= 2.5:
                V_at_n2_5 = V_range[i]
                break
        V_combined1 = np.linspace(V_at_n1, V_at_n2_5, 1000)
        V_combined2 = np.linspace(V_at_n2_5, V_dive, 1000)

        n_maneuver_positive_1 = np.interp(V_combined1, V_range, n_positive)
        n_maneuver_negative_1 = np.interp(V_combined1, V_range, n_negative)
        n_stall_positive_1 = np.interp(V_combined1, V_range, n_stall_positive)
        n_stall_negative_1 = np.interp(V_combined1, V_range, n_stall_negative)
        n_gust_positive_1 = np.interp(V_combined1, pos_V, pos_n)
        n_gust_negative_1 = np.interp(V_combined1, neg_V, neg_n)

        # Adjust the upper and lower bounds to respect n_positive as a hard limit
        n_upper_1 = np.minimum(n_maneuver_positive_1, n_stall_positive_1)
        n_lower_1 = np.maximum(n_maneuver_negative_1, n_stall_negative_1)

        plt.fill_between(V_combined1, n_lower_1, n_upper_1, color="lightgreen", alpha=0.3, label="Allowable Condition", edgecolor='none')

        n_maneuver_positive_2 = np.interp(V_combined2, V_range, n_positive)
        n_maneuver_negative_2 = np.interp(V_combined2, V_range, n_negative)
        n_stall_positive_2 = np.interp(V_combined2, V_range, n_stall_positive)
        n_stall_negative_2 = np.interp(V_combined2, V_range, n_stall_negative)
        n_gust_positive_2 = np.interp(V_combined2, pos_V, pos_n)
        n_gust_negative_2 = np.interp(V_combined2, neg_V, neg_n)

        # Adjust the upper and lower bounds to respect n_positive as a hard limit
        n_upper_2 = np.minimum(np.maximum(n_maneuver_positive_2, n_gust_positive_2), n_stall_positive_2)
        n_lower_2 = np.maximum(np.minimum(n_maneuver_negative_2, n_gust_negative_2), n_stall_negative_2)

        plt.fill_between(V_combined2, n_lower_2, n_upper_2, color="lightgreen", alpha=0.3, edgecolor='none')

        # Annotate vertical limits for the green region
        max_n_allowable = np.max([*n_upper_1, *n_upper_2])
        min_n_allowable = np.min([*n_lower_1, *n_lower_2])
        plt.axhline(max_n_allowable, color="green", linestyle="--", linewidth=1.2, label=f"Max Allowable n = {max_n_allowable:.2f}")
        plt.axhline(min_n_allowable, color="green", linestyle="--", linewidth=1.2, label=f"Min Allowable n = {min_n_allowable:.2f}")

        # Find the velocity where n_positive is equal to 1
        plt.plot([V_at_n1, V_at_n1], [-1, 1], color='magenta', linestyle='--', label='Stall Speed', linewidth=1.5)

        plt.ylim(min_n_allowable - 0.2, max_n_allowable + 0.2)
        plt.grid(True, linestyle=":", alpha=0.7)
        plt.legend()
        plt.tight_layout()
        if plot:
            plt.show()
        else:
            plt.close()

        self.nmax = max_n_allowable
        self.nmin = min_n_allowable

        return max_n_allowable, min_n_allowable

    def update_attributes(self):
        """
        Update the aircraft data.
        """
        self.plot_complete_load_diagram(plot=False)
        self.aircraft_data.data['outputs']['general']['nmax'] = self.nmax
        self.aircraft_data.data['outputs']['general']['nmin'] = self.nmin
        return


if __name__ == "__main__":
    # Load the aircraft data
    design_file = "design3.json"
    aircraft_data = Data(design_file)
    load_diagram = LoadDiagram(aircraft_data)
    
    # Plot the load diagram
    n_max, n_min = load_diagram.plot_complete_load_diagram(plot=True)
    load_diagram.update_attributes()
    aircraft_data.save_design(design_file)
    print(f"Max n: {n_max}, Min n: {n_min}")