import numpy as np
import matplotlib.pyplot as plt

ylst = np.array([0.25, 0.23, 1.46])
xlst = np.array(['C5', 'C17', 'C130'])

# Flip order
ylst_flipped = ylst[::-1]
xlst_flipped = xlst[::-1]

fig = plt.figure()
ax = fig.add_subplot()
ax.set_xlabel('Costs ($/tonnes/km)')
ax.set_title('Operational Costs', fontsize=16)
ax.barh(xlst_flipped, ylst_flipped, color=['orange', 'blue', 'red'])
plt.tight_layout()
plt.show()
