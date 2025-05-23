import numpy as np
import matplotlib.pyplot as plt

ylst = np.array([0.14, 0.163, 0.384])
xlst = np.array(['C5', 'C17', 'C130'])
y2lst = np.append(ylst, 0.070)
x2lst = np.append(xlst, 'Design 10')

# Reverse the arrays for flipped order
ylst_flipped = ylst[::-1]
xlst_flipped = xlst[::-1]
y2lst_flipped = y2lst[::-1]
x2lst_flipped = x2lst[::-1]

# First horizontal bar chart
fig = plt.figure()
ax = fig.add_subplot()
ax.set_title('Fuel Economy', fontsize=16)
ax.set_xlabel('Fuel economy (L/km/tonnes)')
ax.barh(xlst_flipped, ylst_flipped, color=['orange', 'blue', 'red'])
plt.tight_layout()
plt.show()

# Second horizontal bar chart
fig2 = plt.figure()
ax2 = fig2.add_subplot()
ax2.set_title('Fuel Economy', fontsize=16)
ax2.set_xlabel('Fuel economy (L/km/tonnes)')
ax2.barh(x2lst_flipped, y2lst_flipped, color=['green', 'orange', 'blue', 'red'])
plt.tight_layout()
plt.show()
