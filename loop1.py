#  ___   ___  ___  _   _  ___   ___   ____ ___  ____
# / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
#| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
# \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
#                  (____/
# Osoyoo Model-Pi L298N DC motor driver programming guide
# tutorial url: https://osoyoo.com/2020/03/01/python-programming-tutorial-model-pi-l298n-motor-driver-for-raspberry-pi/

import RPi.GPIO as GPIO #control motor board through GPIO pins
import time #set delay time to control moving distance

#PWM on pin 32 = 12; PWM on pin 33 = 35
#pin values below (12) will change to pca channels
#If IN1Rear=True and IN2Rear=False right motor move forward, If IN1Rear=False,IN2Rear=True right motor move backward,in other cases right motor stop
IN1RR = 16 #GPIO23 to IN1 Rear-right wheel direction
IN2RR = 18 #GPIO24 to IN2 Rear-right wheel direction

#If IN3Rear=True and IN3Rear=False left motor move forward, If IN3Rear=False,IN4Rear=True left motor move backward,in other cases left motor stop
IN3RL= 13 #GPIO27 to IN3 Rear-left wheel direction
IN4RL= 15 #GPIO22 to IN4 Rear-left wheel direction

#ENA/ENB are PWM(analog) signal pin which control the speed of right/left motor through GPIO ChangeDutyCycle(speed) function
ENARR = 12 #GPIO18 to ENA PWM SPEED of rear right motor
ENBRL = 33 #GPIO13 to ENB PWM SPEED of rear left motor

#If IN1Front=True and IN2Front=False right motor move forward, If IN1Front=False,IN2Front=True right motor move backward,in other cases right motor stop
IN1FR = 40 #GPIO21 to IN1 Front Model X right wheel direction
IN2FR = 38 #GPIO20 to IN2 Front Model X right wheel direction

#If IN3Front=True and IN3Front=False left motor move forward, If IN3Front=False,IN4Front=True left motor move backward,in other cases left motor stop
IN3FL = 36 #GPIO16 to IN3 Front Model X left wheel direction
IN4FL = 31 #GPIO12 to IN4 Front Model X left wheel direction MOVED from 32

#ENA/ENB are PWM(analog) signal pin which control the speed of right/left motor through GPIO ChangeDutyCycle(speed) function
ENAFR = 32 #GPIO12 to ENA PWM SPEED of front right motor MOVED from pin 1
ENBFL = 35 #GPIO19 to ENB PWM SPEED of front left motor MOVED from pin 17

GPIO.setmode(GPIO.BOARD)

#equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

#for 0 to 100, % speed as integer, to use for PWM
#full range 0xFFFF, but PCS9685 ignores last Hex digit as only 12 bit resolution)
def getPWMPer(value):
  return int(valmap(value, 0, 100, 0, 100)) #last was 0xFFF, change value for PCA

# for IN1, IN2, define 1  and 0 settings
high = True    #0xFFFF ie 1 was True     #change value for PCA
low  = False   #0           was False    #change value for PCA

class Wheel:
  def __init__(self, name, enCh,in1Ch,in2Ch):
    self.name =name #for debug
    #change following five lines for PCA
    self.en = enCh
    self.in1 = in1Ch
    self.in2 = in2Ch
    GPIO.setup(self.en, GPIO.OUT) #EN  wheel 'speed', actually sets power
    GPIO.setup(self.in1, GPIO.OUT) #IN1, IN3 wheel direction control 1
    GPIO.setup(self.in2, GPIO.OUT) #IN2, IN4 wheel direction control 2
    self.Speed = GPIO.PWM(enCh,1000)
    self.Speed.start(0)

    # print(type(self.en), type(self.in1), type(self.in2), type(self.Speed))

    #If IN1=True  and IN2=False motor moves forward,
    #If IN1=False and IN2=True  motor moves backward
    #in both other cases motor will stop/brake
    #right: ENA, IN1, IN2
    #left:  ENB, IN3, IN4 - IN1=IN4 and IN2=IN3 but motor is reversed, so swap

    #print("created Wheel "+str(self.name)+" at "+str(enCh)+" "+str(in1Ch)+" "+str(in2Ch)) #debug

  def move(self,speed):
   #change following three lines for PCA
  	GPIO.output(self.in1, high if speed > 0 else low)
  	GPIO.output(self.in2, low if speed > 0 else high)
  	self.Speed.ChangeDutyCycle(speed) if speed > 0 else self.Speed.ChangeDutyCycle(-speed)
  	#print("move "+self.name+" @ "+str(speed)) #debug
  	#positive speed forward, negative speed reverse/back; 0 coast, 100% = 4095

  def brake(self):
    #change following two lines for PCA
  	GPIO.output(self.in1, low)
  	GPIO.output(self.in2, low)
  	#print("brake "+self.name) #debug
  	#electric braking effect, should stop movement

#end of Wheel class

#Set up Wheel instances with connections, ch 0 is left end,
#leaving one pin per quad for future
rl = Wheel("rl", ENBRL, IN3RL, IN4RL) #Rear-left wheel
rr = Wheel("rr", ENARR, IN1RR, IN2RR) #Rear-right wheel
fl = Wheel("fl", ENBFL, IN3FL, IN4FL) #Front-left wheel
fr = Wheel("fr", ENAFR, IN1FR, IN2FR) #Front-right wheel

#Movement control examples
#rear right motor moving forward was def rr_ahead(speed): ... now rr.move(speed)
#rear right motor moving back    was def rr_back(speed): ...  now rr.move(-speed)

def stop_car():
    rl.brake()
    rr.brake()
    fl.brake()
    fr.brake()
    #brakes all 4 wheels

def coast_car():
    rl.move(0)
    rr.move(0)
    fl.move(0)
    fr.move(0)

def go_ahead(speed):
    rl.move(speed)
    rr.move(speed)
    fl.move(speed)
    fr.move(speed)

def go_back(speed):
    rr.move(-speed)
    rl.move(-speed)
    fr.move(-speed)
    fl.move(-speed)

#making right turn on spot (tank turn)
def turn_right(speed):
    rl.move(speed)
    rr.move(-speed)
    fl.move(speed)
    fr.move(-speed)

#make left turn on spot (tank turn)
def turn_left(speed):
    rr.move(speed)
    rl.move(-speed)
    fr.move(speed)
    fl.move(-speed)

# parallel left shift (crab left)
def shift_left(speed):
    fr.move(speed)
    rr.move(-speed)
    rl.move(speed)
    fl.move(-speed)

# parallel right shift (crab right)
def shift_right(speed):
    fr.move(-speed)
    rr.move(speed)
    rl.move(-speed)
    fl.move(speed)

#diagonal forward and right @45
def upper_right(speed):
    rr.move(speed)
    fl.move(speed)

#diagonal back and left @45
def lower_left(speed):
    rr.move(-speed)
    fl.move(-speed)

#diagonal forward and left @45
def upper_left(speed):
    fr.move(speed)
    rl.move(speed)

#diagonal back and rightt @45
def lower_right(speed):
    fr.move(-speed)
    rl.move(-speed)

def main():
    RSPEED = 87
    SPEED_BUFFER = 2.1
    COAST_BUFFER = 0.5
    TURN_BUFFER = 6
    RADIUS = 18
    WIDTH = 4
    TURN_SPEED_RATIO = ((RADIUS + WIDTH) / (RADIUS - WIDTH)) * 1.44
    TURN_SPEED_MULT = 20
    print("TURN_SPEED_RATIO: " + str(TURN_SPEED_RATIO))

    time.sleep(2)
    for _ in range(2):
        print("going straight")
        fr.move(RSPEED)
        rr.move(RSPEED)
        fl.move(100)
        rl.move(100)
        time.sleep(SPEED_BUFFER)
        coast_car()
        time.sleep(COAST_BUFFER)

        print("turning left")
        fr.move(TURN_SPEED_RATIO * TURN_SPEED_MULT)
        rr.move(TURN_SPEED_RATIO * TURN_SPEED_MULT)
        fl.move(TURN_SPEED_MULT)
        rl.move(TURN_SPEED_MULT)
        time.sleep(TURN_BUFFER)
        coast_car()
        time.sleep(COAST_BUFFER)

    print("going straight")
    fr.move(RSPEED)
    rr.move(RSPEED)
    fl.move(100)
    rl.move(100)
    time.sleep(SPEED_BUFFER)
    coast_car()
    time.sleep(COAST_BUFFER)

def destroy():
    print("Done")
    GPIO.cleanup()

try:
    main()
    destroy()
except KeyboardInterrupt:
    destroy()