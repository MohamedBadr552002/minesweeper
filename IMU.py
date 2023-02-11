import rospy
from std_msgs.msg import Int64


# Topic callback function.
def distanceListenerCallback(data):
    rospy.loginfo('%s', data.data)


def IMU():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'stringListener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imu', anonymous=False)

    rospy.Subscriber('Distance', Int64, distanceListenerCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    

if __name__ == '__main__':
    IMU()
