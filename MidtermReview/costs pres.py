import numpy as np
import matplotlib.pyplot as plt

ylst = np.array([0.46, 0.5, 0.4, 0.0])
xlst = np.array(['C17', 'C5', 'C130', 'WAVE'])

# Flip order
ylst_flipped = ylst[::-1]
xlst_flipped = xlst[::-1]

fig = plt.figure()
ax = fig.add_subplot()
ax.set_xlabel('Costs ($/tonnes/km)')
ax.set_title('Operational Costs', fontsize=16)
fig.patch.set_alpha(0.0)  # Set figure background to transparent
ax.patch.set_alpha(0.0)  # Set axes background to transparent
ax.barh(xlst_flipped, ylst_flipped, color=['tab:green', 'tab:red', 'tab:orange', 'tab:blue'])
plt.tight_layout()
plt.show()
