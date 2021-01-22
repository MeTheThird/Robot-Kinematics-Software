import numpy as np
import matplotlib.pyplot as plt

numDeg = 360
degrees = np.linspace(0, 2*np.pi, numDeg)
r = 1
x, y = [], []
fig, ax = plt.subplots()
ax.axis('equal')
ax.set(xlim=(-2,2), ylim=(-2,2))

for t in degrees:
    x.append(r*np.cos(t))
    y.append(r*np.sin(t))
    ax.plot(x, y, 'r')
    plt.pause(0.001)
plt.show()
