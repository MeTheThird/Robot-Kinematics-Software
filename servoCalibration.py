# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit
# import RPi.GPIO as GPIO

# PWMOEN = 7
# GPIO.setup(PWMOEN, GPIO.OUT)

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16, address=65)

# channel being tested
CHNL = 0

kit.servo[CHNL].set_pulse_width_range(450, 2500)

# kit.servo[4].angle = 90
# GPIO.output(PWMOEN, 0) # enable outputs of PCA9685
kit.servo[CHNL].angle = 0
time.sleep(5)
# GPIO.output(PWMOEN, 1) # disable outputs of PCA9685
kit.servo[CHNL].angle = 180
time.sleep(5)