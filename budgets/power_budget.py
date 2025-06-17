import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import matplotlib.pyplot as plt
import pandas as pd

# Data from the first image
data = {
    'Component': ['ECS', 'Avionic', 'Equipment', 'Fuel System', 'Hydraulic System',
                  'WIPS', 'Window Anti Icing', 'Lights', 'Water Waste', 'Cabin'],
    'Cruise': [85.71429, 9.52381, 57.14286, 4.761905, 80, 0, 9.52381, 9.52381, 0.952381, 11.42857],
    'High Altitude': [142.8571, 9.52381, 57.14286, 4.761905, 80, 133.3333, 11.42857, 9.52381, 0, 14.2857]
}

df = pd.DataFrame(data)

# Set 'Component' as the index for easier plotting
df = df.set_index('Component')

# Define the columns for plotting
categories = ['Cruise', 'High Altitude']
components = df.index.tolist() # Get the component names as a list

# Set up the figure and axes
plt.figure(figsize=(10, 7))
# plt.title('Non paper', fontsize=16)

# Plot the stacked bars
bottom_cruise = [0] * len(df)
bottom_high_altitude = [0] * len(df)

colors = plt.cm.get_cmap('tab10', len(components)) # Using a colormap for distinct colors

for i, component in enumerate(components):
    plt.bar(categories[0], df.loc[component, 'Cruise'],
            bottom=bottom_cruise[0], label=component, color=colors(i))
    bottom_cruise[0] += df.loc[component, 'Cruise']

    plt.bar(categories[1], df.loc[component, 'High Altitude'],
            bottom=bottom_high_altitude[0], color=colors(i)) # Use the same color for consistency
    bottom_high_altitude[0] += df.loc[component, 'High Altitude']

# Update bottom values for the next iteration in the loop (correct way for stacked bar)
# This part is implicitly handled by `plt.bar`'s `bottom` argument in the loop above
# but it's good to understand how cumulative sums work for stacking.
# A more explicit way to calculate bottom would be:
# for cat in categories:
#     current_bottom = 0
#     for comp in components:
#         plt.bar(cat, df.loc[comp, cat], bottom=current_bottom, label=comp if cat == categories[0] else "", color=colors(components.index(comp)))
#         current_bottom += df.loc[comp, cat]


# Customize plot
plt.ylabel('Power [kW]') # No Y-axis label in the example
# plt.xlabel('') # No X-axis label in the example
plt.ylim(0, 500) # Set Y-axis limit based on your example image
plt.yticks(range(0, 501, 50)) # Set Y-axis ticks

# plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.2), ncol=5, frameon=False)
# plt.grid(axis='y', linestyle='-', alpha=0.7) # Add subtle horizontal grid lines
plt.legend(prop = { "size": 11 })
# plt.tight_layout(rect=[0, 0.1, 1, 1]) # Adjust layout to make space for the legend
plt.show()



