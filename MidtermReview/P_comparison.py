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

    # Labels and values (flipped for top-to-bottom order)
    designs = ['Design 2', 'Design 7', 'Design 10']
    designs_flipped = designs[::-1]
    Ps_flipped = Ps[::-1]
    P_TOs_flipped = P_TOs[::-1]

    y = list(range(len(designs)))
    bar_height = 0.35
    y1 = [i - bar_height / 2 for i in y]
    y2 = [i + bar_height / 2 for i in y]

    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot horizontal bars
    bars1 = ax.barh(y1, Ps_flipped, height=bar_height, color='blue', label='Design Power (MW)')
    bars2 = ax.barh(y2, P_TOs_flipped, height=bar_height, color='orange', label='Take-off Power (MW)')

    # Titles and labels
    ax.set_title('Power vs Take-off Power by Design')
    ax.set_xlabel('Power (MW)')
    ax.set_yticks(y)
    ax.set_yticklabels(designs_flipped)

    # Legend
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    P_PTO_comparison()
