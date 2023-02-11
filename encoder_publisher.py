import rospy ,random
from std_msgs.msg import Int64

raduis = 15

def Encoder():
    global raduis
    encoder = rospy.Publisher('Distance', Int64 , queue_size=10)
    rospy.init_node('ENCODER', anonymous=False)
    rate = rospy.Rate(1)

    # The string to be published on the topic
    
    while not rospy.is_shutdown():
            num_of_rotation = random.randrange(0,100)
            distance = num_of_rotation * (2*3.14* raduis)
            msg = Int64()
            msg.data = int(distance)
            #int64(encoder_distance = distance)
            rospy.loginfo(msg.data)
            encoder.publish(msg.data)
            rate.sleep()


if __name__ == '__main__':
    try:
        Encoder()
        
    except rospy.ROSInterruptException:
        pass
