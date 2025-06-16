import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from utils import Data

def mass_breakdown(aircraft_data):
    pass  # Placeholder for the existing mass_breakdown function

def mass_breakdown_OEW(aircraft_data):
    import matplotlib.pyplot as plt
    import numpy as np
    weights = dict(aircraft_data.data['outputs']['component_weights'])
    total_OEW = weights.get('total_OEW', sum(weights.values()))
    # Remove zero or negative weights and total_OEW itself
    weights = {k: v for k, v in weights.items() if v > 0 and k != 'total_OEW'}
    # Calculate percentages
    items = sorted(weights.items(), key=lambda x: x[1], reverse=True)
    major_labels = []
    major_values = []
    misc_value = 0
    for label, value in items:
        perc = value / total_OEW * 100
        if perc < 2.2:
            misc_value += value
        else:
            major_labels.append(label)
            major_values.append(value)
    if misc_value > 0:
        major_labels.append('Miscellaneous')
        major_values.append(misc_value)
    def perc_fmt(x):
        return f'{x/total_OEW*100:.1f}%' if x > 0 else ''
    fig, ax = plt.subplots(figsize=(12, 12))
    wedges, texts = ax.pie(
        major_values,
        labels=None,  # We'll add labels manually
        startangle=90,
        counterclock=False,
        textprops={'fontsize': 12}
    )
    # Place labels and values outside, with lines
    for i, w in enumerate(wedges):
        ang = (w.theta2 + w.theta1) / 2.
        x = w.r * 1.25 * np.cos(np.deg2rad(ang))
        y = w.r * 1.25 * np.sin(np.deg2rad(ang))
        x0 = w.r * np.cos(np.deg2rad(ang))
        y0 = w.r * np.sin(np.deg2rad(ang))
        ax.plot([x0, x], [y0, y], color='gray', lw=1)
        ha = 'left' if x > 0 else 'right'
        ax.text(x, y, f"{major_labels[i]}\n{perc_fmt(major_values[i])}", ha=ha, va='center', fontsize=12, fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.8))
    ax.set_title('Aircraft Component Mass Breakdown (as % of OEW)', fontsize=16)
    plt.tight_layout()
    plt.show()

def mass_breakdown_MTOW(aircraft_data):
    import matplotlib.pyplot as plt
    import numpy as np
    weights = dict(aircraft_data.data['outputs']['component_weights'])
    # Add crew, payload, and fuel
    crew_weight = aircraft_data.data['requirements'].get('design_crew', 0) * 9.81
    payload_weight = aircraft_data.data['requirements'].get('design_payload', 0) * 9.81
    fuel_weight = aircraft_data.data['outputs']['max'].get('total_fuel', 0)
    # Remove zero or negative weights and total_OEW itself
    weights = {k: v for k, v in weights.items() if v > 0 and k != 'total_OEW'}
    weights['Crew'] = crew_weight
    weights['Payload'] = payload_weight
    weights['Fuel'] = fuel_weight
    # Calculate MTOW
    MTOW = aircraft_data.data['outputs']['max']['MTOW']
    print(MTOW)
    # Calculate percentages
    items = sorted(weights.items(), key=lambda x: x[1], reverse=True)
    major_labels = []
    major_values = []
    misc_value = 0
    for label, value in items:
        perc = value / MTOW * 100
        if perc < 1.5:
            misc_value += value
        else:
            major_labels.append(label)
            major_values.append(value)
    if misc_value > 0:
        major_labels.append('Miscellaneous')
        major_values.append(misc_value)
    def perc_fmt(x):
        return f'{x/MTOW*100:.1f}%' if x > 0 else ''
    fig, ax = plt.subplots(figsize=(12, 12))
    wedges, texts = ax.pie(
        major_values,
        labels=None,  # We'll add labels manually
        startangle=90,
        counterclock=False,
        textprops={'fontsize': 12}
    )
    for i, w in enumerate(wedges):
        ang = (w.theta2 + w.theta1) / 2.
        x = w.r * 1.25 * np.cos(np.deg2rad(ang))
        y = w.r * 1.25 * np.sin(np.deg2rad(ang))
        x0 = w.r * np.cos(np.deg2rad(ang))
        y0 = w.r * np.sin(np.deg2rad(ang))
        ax.plot([x0, x], [y0, y], color='gray', lw=1)
        ha = 'left' if x > 0 else 'right'
        ax.text(x, y, f"{major_labels[i]}\n{perc_fmt(major_values[i])}", ha=ha, va='center', fontsize=12, fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.8))
    ax.set_title('Aircraft Mass Breakdown (as % of MTOW)', fontsize=16)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    aircraft_data = Data('final_design.json')
    mass_breakdown_OEW(aircraft_data=aircraft_data)
    mass_breakdown_MTOW(aircraft_data=aircraft_data)