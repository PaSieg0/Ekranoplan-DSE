import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def MTOM_breakdown():
    # Initialize component lists
    OEWs = []
    mission_fuels = []
    Payloads = [90, 90, 90]  # in tonnes
    Crews = [425/1000, 425/1000, 425/1000]  # in tonnes
    MTOMs = []

    # Load data and convert from N to tonnes
    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        OEWs.append(aircraft_data.data['outputs']['max']['OEW'] / 9.81 / 1000)
        mission_fuels.append(aircraft_data.data['outputs']['max']['mission_fuel'] / 9.81 / 1000)
        MTOMs.append(aircraft_data.data['outputs']['max']['MTOM'] / 1000)

    # Reverse for top-to-bottom ordering
    Payloads = Payloads[::-1]
    Crews = Crews[::-1]
    OEWs = OEWs[::-1]
    mission_fuels = mission_fuels[::-1]
    designs = ['Design 2', 'Design 7', 'Design 10'][::-1]
    y = list(range(len(designs)))

    # Stack the horizontal bars
    fig, ax = plt.subplots(figsize=(10, 6))
    left = [0, 0, 0]

    bars1 = ax.barh(y, Payloads, left=left, label='Payload', color='blue')
    left = [l + p for l, p in zip(left, Payloads)]

    bars2 = ax.barh(y, Crews, left=left, label='Crew', color='red')
    left = [l + c for l, c in zip(left, Crews)]

    bars3 = ax.barh(y, OEWs, left=left, label='OEW', color='orange')
    left = [l + o for l, o in zip(left, OEWs)]

    bars4 = ax.barh(y, mission_fuels, left=left, label='Mission Fuel', color='green')

    # Labels and formatting
    ax.set_title('MTOM Composition by Design', fontsize=16)
    ax.set_xlabel('Weight (tonnes)')
    ax.set_yticks(y)
    ax.set_yticklabels(designs)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=16)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    MTOM_breakdown()
