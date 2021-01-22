import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# the x and y positions are given in order and represent a polygon
xPositions = [12, 1, -6, 9]
yPositions = [5, -3, 2, 6]
assert len(xPositions) == len(yPositions)

numSteps = 360
times = np.linspace(0, numSteps, numSteps+1)

leaveTrail = True
realTimeStep = 1000
timeMult = 300
timeStep = int(realTimeStep / timeMult)

xdata, ydata = [], []
fig, ax = plt.subplots()
ln, = ax.plot([], [], 'ro')
ax.set_aspect('equal', adjustable='box')
axesMult = 1.2

def init():
    xBound = max(abs(min(xPositions)), abs(max(xPositions)))
    yBound = max(abs(min(yPositions)), abs(max(yPositions)))
    bound = max(xBound, yBound)
    ax.set_xlim(-axesMult*bound, axesMult*bound)
    ax.set_ylim(-axesMult*bound, axesMult*bound)

    return ln,

def drawPoly(frame):
    ind = int(frame/(numSteps/len(xPositions))) % len(xPositions)
    nextInd = 0 if ind==len(xPositions)-1 else ind+1
    t = (frame % int(numSteps/len(xPositions))) / (numSteps/len(xPositions))

    rise = float(yPositions[nextInd])-yPositions[ind]
    run = float(xPositions[nextInd])-xPositions[ind]
    xPos = xPositions[ind] + run*t
    yPos = yPositions[ind] + rise*t

    if leaveTrail:
        xdata.append(xPos)
        ydata.append(yPos)
        ln.set_data(xdata, ydata)
    else:
        ln.set_data(xPos, yPos)
    return ln,


draw = FuncAnimation(fig, drawPoly, frames=times, init_func=init, interval=timeStep, repeat=False,
                     blit=False)

plt.show()
