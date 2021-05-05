import time
from adafruit_servokit import ServoKit
import numpy as np

# servo angle min and max -- will be determined via calibration
ANGLE_MIN = 0
ANGLE_MAX = 180

kit = ServoKit(channels=16)

# assuming we're using channels 0-4
channels = [0, 1, 2, 3, 4]
for i in range(len(channels)):
    # will be determined via calibration
    kit.servo[i].set_pulse_width_range(444, 2556)

# given input theta vector in degrees
thetaArr = [50, 30, 20, 15, 40]
assert len(thetaArr) == len(channels)

for i in range(len(channels)):
    assert thetaArr[i] <= ANGLE_MAX and thetaArr[i] >= ANGLE_MIN
    kit.servo[i].angle = thetaArr[i]
    time.sleep(2)