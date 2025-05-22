import numpy as np
import matplotlib.pyplot as plt


ylst=np.array([0.25,0.23,1.46])
xlst=np.array(['C5','C17','C130'])



fig=plt.figure()
ax=fig.add_subplot()
ax.set_ylabel('Costs ($/km/tonnes)')
ax.set_title('Operational Costs')
ax.bar(xlst,ylst,color=['green','purple','blue'])
plt.show()
