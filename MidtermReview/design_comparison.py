import pandas as pd
import sys
import os
from plotnine import ggplot, aes, geom_bar, ggtitle, xlab, ylab, theme, element_text
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data

def compare_designs():
    # Component lists
    OEWs = []
    mission_fuels = []
    Payloads = [90000, 90000, 90000]
    Crews = [425, 425, 425]

    for i in range(1, 4):
        aircraft_data = Data(f"design{i}.json")
        OEWs.append(aircraft_data.data['outputs']['max']['OEW']/9.81)
        mission_fuels.append(aircraft_data.data['outputs']['max']['mission_fuel']/9.81)

    designs = ['Design 1', 'Design 2', 'Design 3']

    # Create long-form DataFrame
    data = []
    for i in range(3):
        data.append({'Design': designs[i], 'Component': 'OEW', 'Value': OEWs[i]})
        data.append({'Design': designs[i], 'Component': 'Payload', 'Value': Payloads[i]})
        data.append({'Design': designs[i], 'Component': 'Crew', 'Value': Crews[i]})
        data.append({'Design': designs[i], 'Component': 'Mission Fuel', 'Value': mission_fuels[i]})

    df_long = pd.DataFrame(data)

    # Plot stacked bar chart
    plot = (
        ggplot(df_long, aes(x='Design', y='Value', fill='Component')) +
        geom_bar(stat='identity') +
        ggtitle('MTOM Composition by Design') +
        ylab('Weight (kg)') +
        theme(axis_text_x=element_text(rotation=30, hjust=1))
    )
    return plot

if __name__ == "__main__":
    plot = compare_designs()
    plot.show()
