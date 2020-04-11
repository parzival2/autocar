#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from ds4_driver.msg import Status
from sensor_msgs.msg import Joy

class DualshockTeleop(object):
    def __init__(self):
        # Initialize node
        rosNode = rospy.init_node("dualshock_keyop")
        # Publisher
        self.ackermannPub = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=10)
        # Subscriber
        self.joySub = rospy.Subscriber("joy", Joy, self.onJoyMessageReceived, queue_size=1)
        self.getParameters()
        rospy.spin()

    def getParameters(self):
        # Min and max speed
        self.minSpeed = rospy.get_param("/minspeed")
        self.maxSpeed = rospy.get_param("/maxspeed")
        # Min and max steering angles
        self.minSteer = rospy.get_param("/minsteer")
        self.maxSteer = rospy.get_param("/maxsteer")

    def onJoyMessageReceived(self, joy):
        ackermannMsg = AckermannDrive()
        ackermannMsg.speed = joy.axes[3] * self.maxSpeed
        speedFactor = self.maxSteer
        ackermannMsg.steering_angle = joy.axes[0] * speedFactor
        self.ackermannPub.publish(ackermannMsg)


if __name__ == '__main__':
    dualShockObject = DualshockTeleop()


