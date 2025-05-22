import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def P_PTO_comparison():
    Ps = []
    P_TOs = []

    # Load data and convert from W to MW
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        Ps.append(aircraft_data.data['outputs']['design']['P'] / 1e6)
        P_TOs.append(aircraft_data.data['outputs']['general']['take_off_power'] / 1e6)

    # Labels and positions
    designs = ['Design 2', 'Design 7', 'Design 10']
    x = list(range(len(designs)))
    bar_width = 0.35
    x1 = [i - bar_width / 2 for i in x]
    x2 = [i + bar_width / 2 for i in x]

    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(8, 6))

    # Plot bars on the same axis
    bars1 = ax.bar(x1, Ps, width=bar_width, color='blue', label='Design Power (MW)')
    bars2 = ax.bar(x2, P_TOs, width=bar_width, color='orange', label='Take-off Power (MW)')

    # Titles and labels
    ax.set_title('Power vs Take-off Power by Design')
    ax.set_ylabel('Power (MW)')
    ax.set_xticks(x)
    ax.set_xticklabels(designs)

    # Legend
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    P_PTO_comparison()
