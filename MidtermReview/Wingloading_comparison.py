import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def WP_WS_comparison():
    # Initialize component lists
    WPs = []
    WSs = []

    # Load data and convert from N to kg
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        WSs.append(aircraft_data.data['outputs']['design']['WS'])
        WPs.append(aircraft_data.data['outputs']['design']['WP'])  # MW

    # Labels and positions
    designs = ['Design 2', 'Design 7', 'Design 10']
    x = list(range(len(designs)))

    bar_width = 0.35
    x1 = [i - bar_width / 2 for i in x]
    x2 = [i + bar_width / 2 for i in x]

    # Create the figure and primary axis
    fig, ax1 = plt.subplots(figsize=(8, 6))

    # Secondary y-axis
    ax2 = ax1.twinx()

    # Plot bars
    bars1 = ax1.bar(x1, WSs, width=bar_width, color='blue')
    bars2 = ax2.bar(x2, WPs, width=bar_width, color='orange')

    # Titles and labels
    ax1.set_title('Wing Loading and Weight-to-Power-Ratio by Design')
    ax1.set_ylabel('Wing Loading (N/m²)', color='tab:blue')
    ax2.set_ylabel('Weight-to-Power-Ratio (N/W)', color='tab:orange')

    ax1.set_xticks(x)
    ax1.set_xticklabels(designs)

    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax2.tick_params(axis='y', labelcolor='tab:orange')

    # Create custom legend
    # ax1.legend([bars1[0], bars2[0]], ['W/S', 'W/P'], bbox_to_anchor=(1.05, 1), loc='upper left')

    plt.tight_layout()
    plt.show()

def P_S_comparison():
    # Initialize component lists
    Ps = []
    Ss = []

    # Load data and convert from N to kg
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        Ss.append(aircraft_data.data['outputs']['design']['S'])
        Ps.append(aircraft_data.data['outputs']['design']['P']/1e6)  # MW

    # Labels and positions
    designs = ['Design 2', 'Design 7', 'Design 10']
    x = list(range(len(designs)))

    bar_width = 0.35
    x1 = [i - bar_width / 2 for i in x]
    x2 = [i + bar_width / 2 for i in x]

    # Create the figure and primary axis
    fig, ax1 = plt.subplots(figsize=(8, 6))

    # Secondary y-axis
    ax2 = ax1.twinx()

    # Plot bars
    bars1 = ax1.bar(x1, Ss, width=bar_width, color='blue')
    bars2 = ax2.bar(x2, Ps, width=bar_width, color='orange')

    # Titles and labels
    ax1.set_title('Wing Area and Power by Design')
    ax1.set_ylabel('Wing Area (m²)', color='tab:blue')
    ax2.set_ylabel('Power (MW)', color='tab:orange')

    ax1.set_xticks(x)
    ax1.set_xticklabels(designs)

    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax2.tick_params(axis='y', labelcolor='tab:orange')

    # Create custom legend
    # ax1.legend([bars1[0], bars2[0]], ['S', 'P'], bbox_to_anchor=(1.05, 1), loc='upper left')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    WP_WS_comparison()
    P_S_comparison()
