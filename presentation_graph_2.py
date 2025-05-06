import matplotlib.pyplot as plt
import numpy as np

masses = 1*np.array([457629, 446683, 436041, 425694, 415627, 405841, 396319, 387055, 378039])
print(masses)
range = [2800, 2700, 2600, 2500, 2400, 2300, 2200, 2100, 2000]

plt.plot(range, masses, marker='o', label='Fuel Economy')
plt.plot([2800, 2000,], [500000, 500000], 'r--', label='Fuel Economy Limit')
plt.xlabel('Range [nmi]')
plt.ylabel('Fuel Economy [L/mt/km]')
plt.title('Fuel Economy vs Range')
plt.grid()
plt.legend()
plt.show()

