import time
from adafruit_servokit import ServoKit
import numpy as np

# servo angle min and max -- will be determined via calibration
ANGLE_MIN = 0
ANGLE_MAX = 180

kit = ServoKit(channels=16)

# assuming we're using channels 0-4
# assuming the channels are connected in the following order:
# shoulderRotation, shoulderJoint, elbowJoint, wristRotation, gripperJoint
channels = [0, 1, 2, 3, 4]
pulseWidthRanges = [(450, 2500), (575, 2450), (600, 2550), (440, 2475), (444, 2556)]
# elbowJoint 180 is actually 180, but 10 is really 0 deg
# angleExtremes = [(), (10, 173), (10, 180), (0, 180), (62, 118)]
angleExtremes = [(0, 180), (0, 180), (0, 180), (0, 180), (62, 118)]


for i in range(len(channels)):
    # will be determined via calibration
    kit.servo[i].set_pulse_width_range(pulseWidthRanges[i][0], pulseWidthRanges[i][1])

# given input theta vector in degrees
thetaArr = [50, 30, 20, 15, 40]
assert len(thetaArr) == len(channels)

for i in range(len(channels)):
    assert angleExtremes[i][0] <= thetaArr[i] and thetaArr[i] <= angleExtremes[i][1]
    kit.servo[i].angle = thetaArr[i]
    time.sleep(2)