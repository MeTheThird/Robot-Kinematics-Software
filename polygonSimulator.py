import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# the x and y positions are given in order and represent a polygon
xPositions = [12, -9, 3, 10]
yPositions = [-6, -2, 5, -2]
assert len(xPositions) == len(yPositions)
assert len(xPositions) > 1

distances = [0 for _ in range(len(xPositions))]
distpfx = [0 for _ in range(len(xPositions))]
for i in range(len(xPositions)):
    nextInd = 0 if i==len(xPositions)-1 else i+1
    distances[i] = math.sqrt((xPositions[nextInd]-xPositions[i])**2
                             + (yPositions[nextInd]-yPositions[i])**2)
    distpfx[i] = distances[i] if i==0 else distances[i]+distpfx[i-1]

# print(distances)
# print(distpfx)

numSteps = 360
times = np.linspace(0, numSteps, numSteps+1)

leaveTrail = True
realTimeStep = 1000
timeMult = 1
# calc timestep based on numSteps and distances to make it go at (realTimeStep/timeMult) per unit
timeStep = int((distpfx[len(xPositions)-1] / numSteps) * (realTimeStep / timeMult))
# print("timeStep:", timeStep)
assert timeStep > 0

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
    currDist = (frame/numSteps) * distpfx[len(xPositions)-1]
    ind = 0
    while currDist > distpfx[ind]:
        ind += 1
    nextInd = 0 if ind==len(xPositions)-1 else ind+1
    if ind > 0:
        t = (currDist-distpfx[ind-1]) / (distpfx[ind]-distpfx[ind-1])
    else:
        t = currDist / distpfx[ind]
    # print("frame:", frame, "t:", t, "ind:", ind, "nextInd:", nextInd)
    # print("prevX:", xPositions[ind], "prevY:", yPositions[ind])
    # print("nextX:", xPositions[nextInd], "nextY:", yPositions[nextInd])

    rise = yPositions[nextInd]-yPositions[ind]
    run = xPositions[nextInd]-xPositions[ind]
    xPos = xPositions[ind] + run*t
    yPos = yPositions[ind] + rise*t
    # print("rise:", rise, "run:", run)
    # print("xPos:", xPos, "yPos:", yPos, "\n")

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
