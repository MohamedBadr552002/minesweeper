#!/usr/bin/env python3
import rospy
from control.msg import Joy
from control.msg import Motor
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
import time
import busio
from board import SCL, SDA


class signal_tx_rx():
    '''
    this class is responsible for take the data from GUI and mapped the value to the requied range and send them to topic (motor__signal) to be taken from Servo driver 
    '''
    def __init__(self):
        self.pwms = [0,0,0,0]
        self.pub = rospy.Publisher('motor_signal', Motor, queue_size=10)
        self.max_signal = 100
        self.min_signal = -100

    def map(self, x,y):
        if abs(y) > abs(x):
            
            if y > 0:
                for i in self.pwms:
                    self.pwms[i] = self.max_signal * y
            else:
                for i in self.pwms:
                    self.pwms[i] = self.min_signal * y
        else:
            if x > 0:
                self.pwms[0] = self.max_signal * x
                self.pwms[1] = self.max_signal * x
                self.pwms[2] = self.min_signal * x
                self.pwms[3] = self.min_signal * x
            else:
                self.pwms[0] = self.min_signal * x
                self.pwms[1] = self.min_signal * x
                self.pwms[2] = self.max_signal * x
                self.pwms[3] = self.max_signal * x

    def callback(self , data):
        # Befor you send data you need to mapped ranges
        self.map(data.Joy_right.y,data.Joy_right.x)

        motors = Motor()

        motors.pwm0 = self.pwms[0]
        motors.pwm1 = self.pwms[1]
        motors.pwm2 = self.pwms[2]
        motors.pwm3 = self.pwms[3]

        self.pub.publish(motors)


    def run(self):
        rospy.init_node('Motors_controller', anonymous = True)
        rospy.Rate(1)
        # Create a subscriber node to take data from GUI 
        self.subscriber = rospy.Subscriber('GUI', Joy, self.callback)
        rospy.spin()


class PWM_Driver():
    """
    this class used to send data from PI to PWM Driver 
    """

    def __init__(self):
        # initiate I2C bus
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.driver = PCA9685(self.i2c_bus)
        self.driver.frequency = 50
        self.signals = [0,0,0,0]
        # zeroing PWM signal
        for i in len(self.signals):
            self.driver.chanels[i].duty_cycle  = self.value_to_duty_cycle(0)  


    def value_to_duty_cycle(self, value, input_min=-100, input_max=100, output_min=0, output_max=4095):
        """
        Converts an input value within the range -100 to 100 to a duty cycle value for the PCA9685.

        :param value: The input value to be converted, expected within the range -100 to 100.
        :param input_min: The minimum value of the input range (default is -100).
        :param input_max: The maximum value of the input range (default is 100).
        :param output_min: The minimum value of the output range (default is 0).
        :param output_max: The maximum value of the output range (default is 4095).
        :return: The converted duty cycle value.
        """
        # Ensure the input value is within the specified range
        value = max(input_min, min(value, input_max))
    
        # Calculate the duty cycle using linear interpolation
        duty_cycle = output_min + (output_max - output_min) * (value - input_min) / (input_max - input_min)
        return int(duty_cycle)
    

    def send_PWM_signal(self):
        
        #todo apply smoothing here...............

        for i , value in enumerate(self.signals):
            self.driver.chanels[i].duty_cycle = self.value_to_duty_cycle(min(100 , max(value , -100))) # make sure the sent signal within the range


    def callback(self, data):
        self.signals = [data.pwm0 , data.pwm1 , data.pwm2 ,data.pwm3]
        self.send_PWM_signal()


    def run(self):
        rospy.init_node("PWM_Node", anonymous=True)
        rospy.Subscriber("motor_signal", Motor, self.callback)
        rospy.spin()

    def shutdown(self):
        self.signals = [0,0,0,0]
        self.send_PWM_signal()





if __name__ == "__main__":
    try:
        # Create instance from signal_tx_rx to recieve and transmit data from GUI to PWM driver
        DATA = signal_tx_rx()
        DATA.run()
        DRIVER = PWM_Driver()
        DRIVER.run()
    except rospy.ROSInterruptException:
        DRIVER.shutdown()
