import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def MTOM_breakdown():
    # Initialize component lists
    OEWs = []
    mission_fuels = []
    Payloads = [90, 90, 90]
    Crews = [425/1000, 425/1000, 425/1000]
    MTOMs = []


    # Load data and convert from N to kg
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        OEWs.append(aircraft_data.data['outputs']['max']['OEW'] / 9.81 / 1000)
        mission_fuels.append(aircraft_data.data['outputs']['max']['mission_fuel'] / 9.81 / 1000)
        MTOMs.append(aircraft_data.data['outputs']['max']['MTOM'] / 1000)

    # Labels and positions
    designs = ['Design 2', 'Design 7', 'Design 10']
    x = list(range(len(designs)))  # [0, 1, 2]

    # Stack the bars manually
    fig, ax = plt.subplots(figsize=(8, 6))

    bottom = [0, 0, 0]

    ax.bar(x, Payloads, bottom=bottom, label='Payload', color='blue')
    bottom = [b + p for b, p in zip(bottom, Payloads)]

    ax.bar(x, Crews, bottom=bottom, label='Crew', color='red')
    bottom = [b + c for b, c in zip(bottom, Crews)]

    ax.bar(x, OEWs, bottom=bottom, label='OEW', color='orange')
    bottom = [b + o for b, o in zip(bottom, OEWs)]

    ax.bar(x, mission_fuels, bottom=bottom, label='Mission Fuel', color='green')




    # Labels and formatting
    ax.set_title('MTOM Composition by Design')
    ax.set_ylabel('Weight (tonnes)')
    ax.set_xticks(x)
    ax.set_xticklabels(designs)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    MTOM_breakdown()