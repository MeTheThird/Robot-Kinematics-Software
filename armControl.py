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

def move(thetaArr, sleeping):
    assert len(thetaArr) == len(channels)

    for i in range(len(channels)):
        assert angleExtremes[i][0] <= thetaArr[i] and thetaArr[i] <= angleExtremes[i][1]
        kit.servo[channels[i]].angle = thetaArr[i]
        if sleeping and i > 2:
            time.sleep(3)

def angleSolve(x, y, alphaGuess, betaGuess):
    alpha = sp.Symbol('alpha')
    beta = sp.Symbol('beta')

    f1 = -L2**2 + L1**2 + x**2 + y**2 - 2*L1*(sp.sqrt(x**2+y**2))*sp.cos(alpha)
    f2 = L2**2 + L1**2 - x**2 - y**2 - 2*L1*L2*sp.cos(beta)

    return sp.nsolve((f1,f2), (alpha,beta), (alphaGuess,betaGuess))

x1 = 190.0
y1 = -80.0
x2 = 130.0
y2 = 180.0

# calculate alpha and beta
initAngles = angleSolve(x1, y1, np.pi/6, np.pi/2)
finalAngles = angleSolve(x2, y2, np.pi/2, np.pi/2)

assert len(initAngles) == len(finalAngles)
for i in range(len(initAngles)):
    initAngles[i] = initAngles[i] * 180/np.pi
    finalAngles[i] = finalAngles[i] * 180/np.pi

print("initAngles: ", initAngles)
print("finalAngles: ", finalAngles)


move([90, 90, 90, 90, 90], False)
time.sleep(1)

# picking up sharpie position
initAngleSteps = np.linspace(float(90.0), float(initAngles[0] - abs(np.arctan2(y1,x1))*180/np.pi), 100)
for i in range(len(initAngleSteps)):
    move([45, initAngleSteps[i], initAngles[1] - 90, 90, 90], False)
    time.sleep(0.01)
move([45, initAngleSteps[-1], initAngles[1] - 90, 90, 115], True)

# dropping sharpie position
move([150, finalAngles[0] + abs(np.arctan2(y2,x2))*180/np.pi, finalAngles[1] - 90, 90, 70], True)
move([90, 90, 90, 90, 90], True)















