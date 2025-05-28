import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def WP_WS_comparison():
    WPs = []
    WSs = []
    designs_raw = ['Design 2', 'Design 7', 'Design 10']

    # Load data and collect values in correct order
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        WSs.append(aircraft_data.data['outputs']['design']['WS'])
        WPs.append(aircraft_data.data['outputs']['design']['WP'])

    # Reverse for top-down order
    WSs = WSs[::-1]
    WPs = WPs[::-1]
    designs = designs_raw[::-1]

    y = list(range(len(designs)))
    bar_height = 0.35
    y1 = [i - bar_height / 2 for i in y]
    y2 = [i + bar_height / 2 for i in y]

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twiny()

    bars1 = ax1.barh(y1, WSs, height=bar_height, color='blue')
    bars2 = ax2.barh(y2, WPs, height=bar_height, color='orange')

    ax1.set_title('Wing Loading and Weight-to-Power-Ratio by Design', fontsize=16)
    ax1.set_xlabel('Wing Loading (N/m²)', color='tab:blue')
    ax2.set_xlabel('Weight-to-Power-Ratio (N/W)', color='tab:orange')

    ax1.set_yticks(y)
    ax1.set_yticklabels(designs)

    ax1.tick_params(axis='x', labelcolor='tab:blue')
    ax2.tick_params(axis='x', labelcolor='tab:orange')

    ax1.legend([bars1[0], bars2[0]], ['W/S', 'W/P'], bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=16)

    plt.tight_layout()
    plt.show()

def P_S_comparison():
    Ps = []
    Ss = []
    designs_raw = ['Design 2', 'Design 7', 'Design 10']

    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        Ss.append(aircraft_data.data['outputs']['design']['S'])
        Ps.append(aircraft_data.data['outputs']['design']['P']/1e6)

    # Reverse for top-down order
    Ss = Ss[::-1]
    Ps = Ps[::-1]
    designs = designs_raw[::-1]

    y = list(range(len(designs)))
    bar_height = 0.35
    y1 = [i - bar_height / 2 for i in y]
    y2 = [i + bar_height / 2 for i in y]

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twiny()

    bars1 = ax1.barh(y1, Ss, height=bar_height, color='blue')
    bars2 = ax2.barh(y2, Ps, height=bar_height, color='orange')

    ax1.set_title('Wing Area and Power by Design', fontsize=16)
    ax1.set_xlabel('Wing Area (m²)', color='tab:blue')
    ax2.set_xlabel('Power (MW)', color='tab:orange')

    ax1.set_yticks(y)
    ax1.set_yticklabels(designs)

    ax1.tick_params(axis='x', labelcolor='tab:blue')
    ax2.tick_params(axis='x', labelcolor='tab:orange')

    ax1.legend([bars1[0], bars2[0]], ['Wing Area', 'Power'], bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=16)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    WP_WS_comparison()
    P_S_comparison()
