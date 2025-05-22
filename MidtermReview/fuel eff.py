import numpy as np
import matplotlib.pyplot as plt


ylst=np.array([0.14,0.163,0.384])
xlst=np.array(['C5','C17','C130'])
y2lst=np.append(ylst,0.070)
x2lst=np.append(xlst,'Design 10')


fig=plt.figure()
ax=fig.add_subplot()
ax.set_title('Fuel Economy')
ax.set_ylabel('Fuel economy (L/km/tonnes)')

ax.bar(xlst,ylst,color=['red','blue','orange'])
plt.show()

fig2=plt.figure()
ax2=fig2.add_subplot()
ax2.set_ylabel('Fuel economy (L/km/tonnes)')
ax2.set_title('Fuel Economy')
ax2.bar(x2lst,y2lst,color=['red','blue','orange','green'])
plt.show()