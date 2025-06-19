import matplotlib.pyplot as plt
import numpy as np

# Aircraft labels
aircraft = ['WAVE', 'C5', 'C17', 'C130']

# Real data
ferry_range = [13630, 11667, 8704, 7399]
fuel_economy = [0.125, 0.14, 0.163, 0.384]

y1_labels = np.arange(2000, 14001, 2000)
y2_labels = np.round(np.arange(0, 0.4001, 0.05), 2)

# Positions
x = np.arange(len(aircraft))
width = 0.35

# Create figure and axes with transparent background
fig, ax1 = plt.subplots(figsize=(10, 6), facecolor='none')
ax2 = ax1.twinx()
ax2.set_facecolor('none')

# Bars
bar1 = ax1.bar(x - width/2, ferry_range, width, label='Ferry Range (km)', color='tab:green')
bar2 = ax2.bar(x + width/2, fuel_economy, width, label='Fuel Economy (L/tonne/km)', color='tab:orange')

# Set white text for axes, labels, and ticks
ax1.set_xlabel('Aircraft', color='white', fontsize=20)
ax1.set_ylabel('Ferry Range (km)', color='white', fontsize=20)
ax2.set_ylabel('Fuel Economy (L/tonne/km)', color='white', fontsize=20)
ax1.set_title('Ferry Range and Fuel Economy by Aircraft', color='white', fontsize=20)

ax1.set_xticks(x)
ax1.set_xticklabels(aircraft, color='white', fontsize=20)
ax1.set_yticklabels(y1_labels, color='white', fontsize=20)
ax2.set_yticklabels(y2_labels, color='white', fontsize=20)
ax1.tick_params(axis='y', colors='white')
ax2.tick_params(axis='y', colors='white')

# Set spines (axes lines) to white or transparent
for spine in ax1.spines.values():
    spine.set_color('white')
for spine in ax2.spines.values():
    spine.set_color('white')

# Grid
# ax1.grid(True, which='both', axis='y', linestyle='--', alpha=0.5)
# ax2.grid(True, which='both', axis='y', linestyle='--', alpha=0.5)

# Legend
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
legend = ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper center', fontsize=18)

legend.get_frame().set_facecolor('none')
legend.get_frame().set_edgecolor('none')

for text in legend.get_texts():
    text.set_color("white")



# Tight layout
fig.tight_layout()

# Show plot
plt.show()

# Optional: Save with transparent background
fig.savefig("C:\\Users\\owenm\\OneDrive\\Documents\\Data\\BSc 3\\DSE_files\\Final Review\\Poster_chart.png", transparent=True)
