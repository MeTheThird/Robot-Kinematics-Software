# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16, address=65)

# channel being tested
CHNL = 0

kit.servo[CHNL].set_pulse_width_range(444, 2556)

kit.servo[4].angle = 90

kit.servo[CHNL].angle = 0
time.sleep(4)
kit.servo[CHNL].angle = 180
time.sleep(4)