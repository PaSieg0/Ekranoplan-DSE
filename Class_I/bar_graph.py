import os
import plotnine as gg   # pip install plotnine
import pandas as pd
import numpy as np
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data
from Class_I.ClassIWeightEstimation import MissionType, AircraftType

def generate_df():
    all_rows = []
    for i in range(1, 5):
        file_path = os.path.join(os.path.dirname(__file__), "Data", f"design{i}.json")
        if not os.path.exists(file_path):
            print(f"File {file_path} does not exist. Skipping.")
            continue

        aircraft_data = Data(file_path)
        base_data = aircraft_data.data.copy()

        mission_keys = {"design", "ferry", "altitude"}
        static_data = {k: v for k, v in base_data.items() if k not in mission_keys and not isinstance(v, dict)}

        for mission in MissionType:
            mission_name = mission.name.lower()
            if mission_name not in base_data:
                continue

            mission_data = base_data[mission_name]
            row = {**static_data, **mission_data}
            row["mission_type"] = mission_name
            row["design_id"] = base_data.get("design_id", i)
            all_rows.append(row)

    df = pd.DataFrame(all_rows)
    return df


def plot_bar_graph(df, parameter: str):
    p = (gg.ggplot(df, gg.aes(x='design_id', y=parameter, fill='mission_type'))
         + gg.geom_bar(stat='identity', position='dodge')
         + gg.ggtitle(f'{parameter} by design and Mission Type')
         + gg.xlab('Design')
         + gg.ylab(parameter)
         + gg.scale_fill_manual(values=['#FF9999', '#66B3FF', '#99FF99'])
        #  + gg.geom_line(gg.aes(y=500000), color='red', linetype='dashed')
         )
    return p



if __name__ == "__main__":
    df = generate_df()
    plot = plot_bar_graph(df, 'Fuel')
    plot.show()