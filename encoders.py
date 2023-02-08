import RPi.GPIO as GPIO
import time
from adafruit_motorkit import MotorKit

#Define GPIO PINS
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN , pull_ip_down= GPIO.PUD_UP)

stateLast =GPIO.input(17)
num_of_rotation =0
num_of_state =0
stateCountTotal = 0


Cir = 207 #circumference of the wheel(in mm)
states_Per_Rotation = 40 #number of steps per rotation
Distance_per_step = Cir / states_Per_Rotation


kit = Motorkit()
kit.motor1.throttle =0.3


try:
    while True:
        stateCurrent = GPIO.input(17)
        if stateCurrent != stateLast:
            stateLast = stateCurrent
            num_of_state +=1
            stateCountTotal +=1


        if num_of_state == states_Per_Rotation :
            num_of_rotation +=1
            num_of_state =0    

        distance = Distance_per_step * stateCountTotal
        print("distance" , distance)

except KeyboardInterrupt:
    kit.motor1.throttle = 0
    GPIO.cleanup()