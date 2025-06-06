import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def interpolate_airfoil(points, num_interp=300, plot=False):
    # List/array of (x, y) coordinate tuples is input
    points = np.array(points)

    # Parametric index based on cumulative arc length (for smooth spacing)
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    t = np.insert(np.cumsum(distances), 0, 0)
    t /= t[-1]  # normalize to [0, 1]

    cs_x = CubicSpline(t, points[:, 0])
    cs_y = CubicSpline(t, points[:, 1])

    t_interp = np.linspace(0, 1, num_interp)
    x_interp = cs_x(t_interp)
    y_interp = cs_y(t_interp)

    interpolated = np.column_stack((x_interp, y_interp))

    if plot:
        fig=plt.figure(figsize=(10, 4))
        ax=plt.subplot()
        ax.set_xlim(-0.1,1.1)
        ax.set_ylim(-0.5,0.5)
        for i in range(len(points[:,0])-1):
            
            ax.plot(points[:, 0][i:i+2], points[:, 1][i:i+2], 'b-', label='Original Points')
            #ax.plot(points[:, 0][i+1], points[:, 1][i+1], 'o-', label='Original Points')
            #plt.plot(x_interp, y_interp, '-', label='Interpolated Airfoil')
            # plt.axis('equal')
            # plt.grid(True)
            # plt.legend()
            # plt.title("Smooth Airfoil Interpolation (Order Preserved)")
            # plt.show()
            plt.pause(0.5)
        plt.show()
    return interpolated

glenn_points_2 = airfoil_points = [
    (1.0000, 0.0082),
    (0.9500, 0.0265),
    (0.9000, 0.0423),
    (0.8000, 0.0709),
    (0.7000, 0.0947),
    (0.6000, 0.1128),
    (0.5000, 0.1289),
    (0.4000, 0.1377),
    (0.3000, 0.1423),
    (0.2000, 0.1383),
    (0.1500, 0.1280),
    (0.1000, 0.1116),
    (0.0750, 0.0995),
    (0.0500, 0.0829),
    (0.0250, 0.0597),
    (0.0125, 0.0440),
    (0.0000, 0.0193),
    (0.0125, 0.0083),
    (0.0250, 0.0043),
    (0.0500, 0.0006),
    (0.0750, 0.0000),
    (0.1000, 0.0000),
    (0.1500, 0.0005),
    (0.2000, 0.0036),
    (0.3000, 0.0110),
    (0.4000, 0.0114),
    (0.5000, 0.0092),
    (0.6000, 0.0035),
    (0.7000, 0.0007),
    (0.8000, 0.0000),
    (0.9000, 0.0017),
    (0.9500, 0.0035),
    (1.0000, 0.0082)
]
glenn_points_21 = [(0.0000, 0.0886),
    (0.0125, 0.1221),
    (0.0250, 0.1381),
    (0.0500, 0.1598),
    (0.0750, 0.1765),
    (0.1000, 0.1892),
    (0.1500, 0.2072),
    (0.2000, 0.2168),
    (0.3000, 0.2213),
    (0.4000, 0.2113),
    (0.5000, 0.1920),
    (0.6000, 0.1664),
    (0.7000, 0.1335),
    (0.8000, 0.0943),
    (0.9000, 0.0500),
    (0.9500, 0.0257),
    (1.0000, 0.0000),
    (0.9500, 0.0000),
    (0.9000, 0.0000),
    (0.8000, 0.0000),
    (0.7000, 0.0000),
    (0.6000, 0.0000),
    (0.5000, 0.0000),
    (0.4000, 0.0000),
    (0.3000, 0.0000),
    (0.2000, 0.0028),
    (0.1500, 0.0075),
    (0.1000, 0.0172),
    (0.0750, 0.0235),
    (0.0500, 0.0333),
    (0.0250, 0.0479),
    (0.0125, 0.0603),
    (0.0000, 0.0886)
]

for i in range(len(interpolate_airfoil(glenn_points_21, num_interp=200, plot=True))):
    print(*interpolate_airfoil(glenn_points_21, num_interp=200, plot=False)[i])
# print(*interpolate_airfoil(glenn_points, num_interp=200, plot=True))