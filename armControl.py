import time
from adafruit_servokit import ServoKit
import numpy as np
import sympy as sp

# link lengths
L1 = 105
L2 = 175

kit = ServoKit(channels=16, address=65)

# assuming we're using channels 0-4
# assuming the channels are connected in the following order:
# shoulderRotation, shoulderJoint, elbowJoint, wristRotation, gripperJoint
channels = [0, 1, 2, 3, 4]
pulseWidthRanges = [(450, 2500), (575, 2450), (600, 2550), (440, 2475), (444, 2556)]
angleExtremes = [(0, 180), (0, 180), (0, 180), (0, 180), (62, 118)]

for i in range(len(channels)):
    kit.servo[channels[i]].set_pulse_width_range(pulseWidthRanges[i][0], pulseWidthRanges[i][1])

def move(thetaArr):
    assert len(thetaArr) == len(channels)

    for i in range(len(channels)):
        assert angleExtremes[i][0] <= thetaArr[i] and thetaArr[i] <= angleExtremes[i][1]
        kit.servo[channels[i]].angle = thetaArr[i]

def angleSolve(x, y, alphaGuess, betaGuess):
    alpha = sp.Symbol('alpha')
    beta = sp.Symbol('beta')

    f1 = -L2**2 + L1**2 + x**2 + y**2 - 2*L1*(sp.sqrt(x**2+y**2))*sp.cos(alpha)
    f2 = L2**2 + L1**2 - x**2 - y**2 - 2*L1*L2*sp.cos(beta)

    return sp.nsolve((f1,f2), (alpha,beta), (alphaGuess,betaGuess))

x1 = 190.0
y1 = -80.0
x2 = 140.0
y2 = 150.0

# calculate alpha and beta
initAngles = angleSolve(x1, y1, np.pi/6, np.pi/2)
finalAngles = angleSolve(x2, y2, np.pi/4, np.pi/2)

assert len(initAngles) == len(finalAngles)
for i in range(len(initAngles)):
    initAngles[i] = initAngles[i] * 180/np.pi
    finalAngles[i] = finalAngles[i] * 180/np.pi

print("initAngles: ", initAngles)
print("finalAngles: ", finalAngles)


move([90, 90, 90, 90, 70])
time.sleep(1)

# picking up sharpie position
initAngleSteps = np.linspace(float(90.0), float(initAngles[0] - abs(np.arctan2(y1,x1))*180/np.pi), 100)
for i in range(len(initAngleSteps)):
    move([42, initAngleSteps[i], initAngles[1] - 90, 90, 70])
    time.sleep(0.01)
move([42, initAngleSteps[-1], initAngles[1] - 90, 90, 115])
time.sleep(2)

# dropping sharpie position
finalAngleSteps = [np.linspace(float(45.0), float(150.0), 100),
                   np.linspace(float(initAngleSteps[-1]), float(finalAngles[0] + abs(np.arctan2(y2,x2))*180/np.pi), 100),
                   np.linspace(float(initAngles[1] - 90), float(finalAngles[1] - 90), 100)]
for i in range(len(finalAngleSteps[0])):
    move([finalAngleSteps[0][i], finalAngleSteps[1][i], finalAngleSteps[2][i], 90, 115])
    time.sleep(0.01)

time.sleep(5)
openClawSlow = np.linspace(float(115.0), float(70.0), 100)
for i in range(len(openClawSlow)):
    move([150, finalAngles[0] + abs(np.arctan2(y2,x2))*180/np.pi, finalAngles[1] - 90, 90, openClawSlow[i]])
time.sleep(1.5)
move([90, 90, 90, 90, 90])















