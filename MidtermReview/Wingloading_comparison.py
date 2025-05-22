import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def Wingloading_comparison():
    # Initialize component lists
    WPs = []
    Ss = []

    # Load data and convert from N to kg
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        Ss.append(aircraft_data.data['outputs']['wing_design']['S'])
        Ss.append(aircraft_data.data['outputs']['max']['WP'])  # MW

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
    bars1 = ax1.bar(x1, WPs, width=bar_width, color='tab:blue')
    bars2 = ax2.bar(x2, Ss, width=bar_width, color='tab:orange')

    # Titles and labels
    ax1.set_title('Hull Surface and Take Off Power by Design')
    ax1.set_ylabel('Hull Surface (m²)', color='tab:blue')
    ax2.set_ylabel('Take Off Power (MW)', color='tab:orange')

    ax1.set_xticks(x)
    ax1.set_xticklabels(designs)

    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax2.tick_params(axis='y', labelcolor='tab:orange')

    # Create custom legend
    ax1.legend([bars1[0], bars2[0]], ['Hull Surface', 'Take Off Power'], bbox_to_anchor=(1.05, 1), loc='upper left')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    Wingloading_comparison()
