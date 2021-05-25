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

# positions
x1 = 190.0
y1 = -80.0
x2 = 140.0
y2 = 150.0

# calculate alpha and beta
firstAngles = angleSolve(x1, y1, np.pi/6, np.pi/2)
finalAngles = angleSolve(x2, y2, np.pi/4, np.pi/2)

assert len(firstAngles) == len(finalAngles)
for i in range(len(firstAngles)):
    firstAngles[i] = firstAngles[i] * 180/np.pi
    finalAngles[i] = finalAngles[i] * 180/np.pi

print("firstAngles: ", firstAngles)
print("finalAngles: ", finalAngles)

# constants
INIT_OPEN = [90, 90, 90, 90, 70]
INIT_NEUTRAL = [90, 90, 90, 90, 90]
INIT_CLOSED = [90, 90, 90, 90, 115]

SHOULDER_ROTATION1 = 42
SHOULDER_ROTATION2 = 150

GAMMA1 = abs(np.arctan2(y1,x1))*180/np.pi
GAMMA2 = abs(np.arctan2(y2,x2))*180/np.pi
ELBOW_CORRECTION = -90

NUM_STEPS = 100

SLEEP_INIT = 1
SLEEP_LINSPACE = 0.01
SLEEP_PICKED_UP = 2
SLEEP_DROPPING = 5
SLEEP_FINAL = 1.5



move(INIT_OPEN)
time.sleep(SLEEP_INIT)

# picking up sharpie position
firstAngleSteps = np.linspace(float(INIT_OPEN[1]), float(firstAngles[0] - GAMMA1), NUM_STEPS)
for i in range(NUM_STEPS):
    move([SHOULDER_ROTATION1, firstAngleSteps[i], firstAngles[1] + ELBOW_CORRECTION, INIT_OPEN[3], INIT_OPEN[4]])
    time.sleep(SLEEP_LINSPACE)

move([SHOULDER_ROTATION1, firstAngleSteps[-1], firstAngles[1] + ELBOW_CORRECTION, INIT_CLOSED[3], INIT_CLOSED[4]])
time.sleep(SLEEP_PICKED_UP)

# dropping sharpie position
finalAngleSteps = [np.linspace(float(SHOULDER_ROTATION1), float(SHOULDER_ROTATION2), NUM_STEPS),
                   np.linspace(float(firstAngleSteps[-1]), float(finalAngles[0] + GAMMA2), NUM_STEPS),
                   np.linspace(float(firstAngles[1] + ELBOW_CORRECTION), float(finalAngles[1] + ELBOW_CORRECTION), NUM_STEPS)]
for i in range(NUM_STEPS):
    move([finalAngleSteps[0][i], finalAngleSteps[1][i], finalAngleSteps[2][i], INIT_CLOSED[3], INIT_CLOSED[4]])
    time.sleep(SLEEP_LINSPACE)

time.sleep(SLEEP_DROPPING)
openClawSlow = np.linspace(float(INIT_CLOSED[4]), float(INIT_OPEN[4]), NUM_STEPS)
for i in range(len(openClawSlow)):
    move([SHOULDER_ROTATION2, finalAngles[0] + GAMMA2, finalAngles[1] + ELBOW_CORRECTION, INIT_OPEN[3], openClawSlow[i]])

time.sleep(SLEEP_FINAL)
move(INIT_NEUTRAL)















