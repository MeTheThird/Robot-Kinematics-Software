import time # used to set delay time to control moving distance
import math # used for pi
# set up Raspberry Pi
import RPi.GPIO as GPIO # control through GPIO pins not BCM

# set up PC9685 osoyoo/AdaFruit
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685

# pins
PWMOEN = 7 # PCA9685 OEn pin

# front controller
ENAFR = 0
IN1FR = 1
IN2FR = 2

IN3FL = 5
IN4FL = 6
ENBFL = 4

# rear controller
ENARR = 8
IN1RR = 9
IN2RR = 10

IN3RL = 13
IN4RL = 14
ENBRL = 12

# create i2c bus interface to access PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60
GPIO.setup(PWMOEN, GPIO.OUT)

#equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

#for 0 to 100, % speed as integer, to use for PWM
#full range 0xFFFF, but PCS9685 ignores last Hex digit as only 12 bit resolution)
def getPWMPer(value):
    return int(valmap(value, 0, 100, 0, 0xFFFF))

# for IN1, IN2, define 1  and 0 settings
high = 0xFFFF # 1 was True
low  = 0      # 0 was False

class Wheel:
    def __init__(self, name, enCh,in1Ch,in2Ch):
        self.name = name #for debug
        self.en  = pca.channels[enCh]  #EN  wheel 'speed', actually sets power
        self.in1 = pca.channels[in1Ch] #IN1, IN3 wheel direction control 1
        self.in2 = pca.channels[in2Ch] #IN2, IN4 wheel direction control 2
        #If IN1=True  and IN2=False motor moves forward,
        #If IN1=False and IN2=True  motor moves backward
        #in both other cases motor will stop/brake
        #right: ENA, IN1, IN2
        #left:  ENB, IN3, IN4 - IN1=IN4 and IN2=IN3 but motor is reversed, so swap

        #print("created Wheel "+str(self.name)+" at "+str(enCh)+" "+str(in1Ch)+" "+str(in2Ch)) #debug

    def move(self,speed):
        self.in1.duty_cycle = high if speed > 0 else low
        self.in2.duty_cycle = low if speed > 0 else high
        self.en.duty_cycle  = getPWMPer(speed) if speed > 0 else getPWMPer(-speed)
        #print("move "+self.name+" @ "+str(speed)) #debug
        #positive speed forward, negative speed reverse/back; 0 coast, 100% = 4095

    def brake(self):
        self.in1.duty_cycle = low
        self.in2.duty_cycle = low
        #print("brake "+self.name) #debug
        #electric braking effect, should stop movement

#end of Wheel class

#Set up Wheel instances with connections, ch 0 is left end,
#leaving one pin per quad for future
rl = Wheel("rl", ENBRL, IN3RL, IN4RL) #Rear-left wheel
rr = Wheel("rr", ENARR, IN1RR, IN2RR) #Rear-right wheel
fl = Wheel("fl", ENBFL, IN3FL, IN4FL) #Front-left wheel
fr = Wheel("fr", ENAFR, IN1FR, IN2FR) #Front-right wheel

def coast_car():
    fr.move(0)
    fl.move(0)
    rl.move(0)
    rr.move(0)

def stop_car():
    fr.brake()
    fl.brake()
    rl.brake()
    rr.brake()

def get_fr_speed(v_tx, v_ty, w, a, b):
    return -v_tx + v_ty + w*(a+b)

def get_fl_speed(v_tx, v_ty, w, a, b):
    return v_tx + v_ty - w*(a+b)

def get_rl_speed(v_tx, v_ty, w, a, b):
    return -v_tx + v_ty - w*(a+b)

def get_rr_speed(v_tx, v_ty, w, a, b):
    return v_tx + v_ty + w*(a+b)

# the scaling function needs to multiply every speed by 100/maxspeed, then pass over the array and
# int cast any float over 100 (could also assert that the int casted number equals 100)
def main():
    GPIO.output(PWMOEN, 0) # enable outputs of PCA9685

    # speed ratios at which each wheel turns at the same rate
    # determined by testing via encoder outputs
    # necessary beacuse each wheel motor has a slightly different speed
    FR_RATIO = 0.65825
    RR_RATIO = 0.75501
    FL_RATIO = 0.92369
    RL_RATIO = 1.00000

    # time buffer parameters
    STRAIGHTAWAY_BUFFER = 2.1
    COAST_BUFFER = 0.5

    # constants
    RADIUS = 65
    WIDTH = 4 # distance in inches in the x direction between the center of the car and each wheel
    LENGTH = 4 # distance in inches in the y direction between the center of the car and each wheel
    ENDTIME = 2*math.pi
    ANG_BUFFER1 = 0.085
    ANG_BUFFER2 = 0.6


    # buffer time
    time.sleep(2)


    print("going forward")

    fr.move(FR_RATIO*87)
    rr.move(RR_RATIO*87)
    fl.move(FL_RATIO*100)
    rl.move(RL_RATIO*100)
    time.sleep(STRAIGHTAWAY_BUFFER)


    print("semicircle left")

    init_time = time.time()
    prevMod = -1
    elapsed_time = 0
    while (elapsed_time < ENDTIME):
        while True:
            elapsed_time = time.time() - init_time
            if int(elapsed_time*pca.frequency) % pca.frequency != prevMod:
                prevMod = int(elapsed_time*pca.frequency) % pca.frequency
                break

        print("elapsed time: " + str(elapsed_time))

        v_tx = -RADIUS * math.sin(elapsed_time / 2)
        v_ty = RADIUS * math.cos(elapsed_time / 2)

        print("v_tx: " + str(v_tx) + ", v_ty: " + str(v_ty) + "\n")

        if elapsed_time < ENDTIME / 2:
            fr_speed = get_fr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER1, WIDTH, LENGTH) * FR_RATIO
            rr_speed = get_rr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER1, WIDTH, LENGTH) * RR_RATIO
            fl_speed = get_fl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER1, WIDTH, LENGTH) * FL_RATIO
            rl_speed = get_rl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER1, WIDTH, LENGTH) * RL_RATIO
        else:
            fr_speed = get_fr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER2, WIDTH, LENGTH) * FR_RATIO
            rr_speed = get_rr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER2, WIDTH, LENGTH) * RR_RATIO
            fl_speed = get_fl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER2, WIDTH, LENGTH) * FL_RATIO
            rl_speed = get_rl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER2, WIDTH, LENGTH) * RL_RATIO

        fr.move(fr_speed)
        rr.move(rr_speed)
        fl.move(fl_speed)
        rl.move(rl_speed)

        print("fr: " + str(fr_speed))
        print("rr: " + str(rr_speed))
        print("fl: " + str(fl_speed))
        print("rl: " + str(rl_speed))


    print("going backward")

    fr.move(-FR_RATIO*87)
    rr.move(-RR_RATIO*87)
    fl.move(-FL_RATIO*100)
    rl.move(-RL_RATIO*100)
    time.sleep(STRAIGHTAWAY_BUFFER)


    print("semicircle right")

    ANG_BUFFER3 = 0.23
    ANG_BUFFER4 = 0.81

    init_time = time.time()
    prevMod = -1
    elapsed_time = 0
    while (elapsed_time < ENDTIME):
        while True:
            elapsed_time = time.time() - init_time
            if int(elapsed_time*pca.frequency) % pca.frequency != prevMod:
                prevMod = int(elapsed_time*pca.frequency) % pca.frequency
                break

        print("elapsed time: " + str(elapsed_time))

        v_tx = RADIUS * math.sin(elapsed_time / 2)
        v_ty = -RADIUS * math.cos(elapsed_time / 2)

        print("v_tx: " + str(v_tx) + ", v_ty: " + str(v_ty) + "\n")

        if elapsed_time < ENDTIME / 2:
            fr_speed = get_fr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER3, WIDTH, LENGTH) * FR_RATIO
            rr_speed = get_rr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER3, WIDTH, LENGTH) * RR_RATIO
            fl_speed = get_fl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER3, WIDTH, LENGTH) * FL_RATIO
            rl_speed = get_rl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER3, WIDTH, LENGTH) * RL_RATIO
        else:
            fr_speed = get_fr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER4, WIDTH, LENGTH) * FR_RATIO
            rr_speed = get_rr_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER4, WIDTH, LENGTH) * RR_RATIO
            fl_speed = get_fl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER4, WIDTH, LENGTH) * FL_RATIO
            rl_speed = get_rl_speed(v_tx, v_ty, (ENDTIME/2 - elapsed_time) * ANG_BUFFER4, WIDTH, LENGTH) * RL_RATIO

        fr.move(fr_speed)
        rr.move(rr_speed)
        fl.move(fl_speed)
        rl.move(rl_speed)

        print("fr: " + str(fr_speed))
        print("rr: " + str(rr_speed))
        print("fl: " + str(fl_speed))
        print("rl: " + str(rl_speed))


    print("going forward")

    fr.move(FR_RATIO*87)
    rr.move(RR_RATIO*87)
    fl.move(FL_RATIO*100)
    rl.move(RL_RATIO*100)
    time.sleep(STRAIGHTAWAY_BUFFER-0.63)

    GPIO.output(PWMOEN, 1) # disable outputs of PCA9685

def destroy():
    print("Done")
    stop_car()
    GPIO.cleanup()

try:
    main()
    destroy()
except KeyboardInterrupt:
    print("KeyboardInterrupt")
    destroy()
except ValueError:
    print("ValueError")
    destroy()