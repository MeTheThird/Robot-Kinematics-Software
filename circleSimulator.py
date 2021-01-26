import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

initDeg = 0
endDeg = 360
degStep = 1
r = 1
angles = np.linspace(0, 2*np.pi, int((endDeg-initDeg+1) / degStep))

leaveTrail = True
realTimeStep = 1000
timeMult = 1
timeStep = int(realTimeStep / timeMult)

xdata, ydata = [], []
fig, ax = plt.subplots()
ln, = ax.plot([], [], 'ro')
ax.set_aspect('equal', adjustable='box')

def init():
    ax.set_xlim(-1.5*r, 1.5*r)
    ax.set_ylim(-1.5*r, 1.5*r)
    return ln,

def drawCircle(t):
    if leaveTrail:
        xdata.append(r*np.cos(t))
        ydata.append(r*np.sin(t))
        ln.set_data(xdata, ydata)
    else:
        ln.set_data(r*np.cos(t), r*np.sin(t))
    return ln,


draw = FuncAnimation(fig, drawCircle, frames=angles, init_func=init, interval=timeStep,
                     repeat=False, blit=False)

plt.show()
