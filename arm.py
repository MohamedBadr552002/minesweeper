import rospy
from control.msg import Joy
from rover import PWM_Driver, DRIVER


class ARM_CONTROL():
    """
    this class take the Servo driver channels connected to motors
    """
    def __init__(self, ch1,dir1,ch2,dir2) -> None:
        self.pwm =[0,0]
        self.direction =[0,0]
        self.motors = [ch1,dir1,ch2,dir2]

        self.reset()


    def reset(self):
        # inital value
        for i in self.motors:
            DRIVER.driver.chanels[i].duty_cycle = self.ConverToDutyCycle(0)

    def ConverToDutyCycle(self,value) -> int:
        value = value * 100 
        return int((value /100) * 0xFFFF )


    def map(self,x,y):
        if abs(x) > 0:
            #Define the rotation direction
            if x > 0:
                self.direction[0] = self.ConverToDutyCycle(100) 
            else:
                self.direction[0] = self.ConverToDutyCycle(0)

            #define PWM signal
                self.pwm[0] = self.ConverToDutyCycle(x)

        elif abs(y) > 0:  # second motor 
            #Define the rotation direction
            if y > 0:
                self.direction[1] = self.ConverToDutyCycle(100) 
            else:
                self.direction[1] = self.ConverToDutyCycle(0)

            #define PWM signal
                self.pwm[1] = self.ConverToDutyCycle(y)

    
    def callback(self, data):
        self.map(data.Joy_left.x,data.Joy_left.y)

        #write on PWM servo drivere channels 
        # motor 1
        DRIVER.driver.chanels[self.motors[0]].duty_cycle = self.pwm[0]
        DRIVER.driver. chanels[self.motors[1]].duty_cycle = self.direction[0]

        #motor 2
        DRIVER.driver.chanels[self.motors[2]].duty_cycle = self.pwm[1]
        DRIVER.driver.chanels[self.motors[3]].duty_cycle = self.direction[1]



    def run(self):
        rospy.init_node('arm_controller', anonymous = True)
        rospy.subscriber('arm',Joy,self.callback)

        rospy.spin()




if __name__ == '__main__':
    try:
        robot_arm = ARM_CONTROL(4,5,6,7)
    except rospy.ROSInterruptException:
       print("ARM is out of service !!!!!")
