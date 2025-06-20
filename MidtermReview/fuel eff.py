import numpy as np
import matplotlib.pyplot as plt

ylst = np.array([0.163, 0.14])
xlst = np.array(['C17', 'C5'])
y2lst = np.append(ylst, 0)
x2lst = np.append(xlst, 'WAVE')

# Reverse the arrays for flipped order
ylst_flipped = ylst[::-1]
xlst_flipped = xlst[::-1]
y2lst_flipped = y2lst[::-1]
x2lst_flipped = x2lst[::-1]

# First horizontal bar chart
fig, ax = plt.subplots(figsize=(10, 6))  # width=6 inches, height=6 inches
ax.set_title('Fuel Economy', fontsize=16)
ax.set_xlabel('Fuel economy (L/km/tonnes)')
ax.barh(xlst_flipped, ylst_flipped, color=['tab:orange', 'tab:blue'])
plt.tight_layout()
plt.show()

# Second horizontal bar chart
fig2, ax2 = plt.subplots(figsize=(6, 6))  # width=6 inches, height=6 inches
ax2.set_title('Fuel Economy', fontsize=16)
ax2.set_xlabel('Fuel economy (L/km/tonnes)')
ax2.barh(x2lst_flipped, y2lst_flipped, color=['tab:green', 'tab:orange', 'tab:blue'])
fig2.patch.set_alpha(0.0)  # Set figure background to transparent
ax2.patch.set_alpha(0.0)  # Set axes background to transparent
plt.tight_layout()
plt.xlim(0, 0.2)
plt.show()
