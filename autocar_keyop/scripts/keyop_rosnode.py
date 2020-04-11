from PyQt5.QtCore import QThread, QTimer
import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import JointState

class RosThreaddedNode(QThread):
    def __init__(self):
        super(RosThreaddedNode, self).__init__()
        # Initialize the ros node
        self.rosNode = rospy.init_node("autocar_keyop")
        # Publisher
        self.ackermannPub = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=10)
        self.currentAckermannMsg = AckermannDrive()
        # self.jointStateSub = rospy.Subscriber("joint_states", JointState, self.onJointStateReceived)
        # Joint states
        self.currentSpeed = 0.0
        self.currentSteerAngle = 0.0
        # The scale value that decided how much the velocity will be increased on each keypress
        self.speedScale = 0.1
        self.angleScale = 0.01
        # Collect parameters for parameter server
        self.getParameters()

    def getParameters(self):
        # Min and max speed
        self.minSpeed = rospy.get_param("/minspeed")
        self.maxSpeed = rospy.get_param("/maxspeed")
        # Min and max steering angles
        self.minSteer = rospy.get_param("/minsteer")
        self.maxSteer = rospy.get_param("/maxsteer")

    def resetSpeed(self):
        rospy.loginfo("Resetting speed")
        self.currentSpeed = 0.
        self.currentAckermannMsg.speed = self.currentSpeed
        # self.publishAckermannMessage()

    def resetSteer(self):
        self.currentSteerAngle = 0.
        self.currentAckermannMsg.steering_angle = self.currentSteerAngle
        self.publishAckermannMessage()

    def run(self):
        rospy.spin()

    def keyUpPressed(self):
        speed = self.currentSpeed + self.speedScale
        self.currentSpeed = max(min(speed, self.maxSpeed), self.minSpeed)
        self.currentAckermannMsg.speed = self.currentSpeed
        self.publishAckermannMessage()

    def keyLeftPressed(self):
        currentSpeed = self.currentSpeed
        if(not currentSpeed > 0):
            currentSpeed = 1
        steer = self.currentSteerAngle + self.angleScale * (1/currentSpeed)
        self.currentSteerAngle = max(min(steer, self.maxSteer), self.minSteer)
        self.currentAckermannMsg.steering_angle = self.currentSteerAngle
        self.publishAckermannMessage()

    def keyRightPressed(self):
        currentSpeed = self.currentSpeed
        if (not currentSpeed > 0):
            currentSpeed = 1
        steer = self.currentSteerAngle - self.angleScale * (1/currentSpeed)
        self.currentSteerAngle = max(min(steer, self.maxSteer), self.minSteer)
        self.currentAckermannMsg.steering_angle = self.currentSteerAngle
        self.publishAckermannMessage()

    def publishAckermannMessage(self):
        rospy.loginfo("Current speed : %f", self.currentAckermannMsg.speed)
        rospy.loginfo("Current steer : %f", self.currentAckermannMsg.steering_angle)
        self.ackermannPub.publish(self.currentAckermannMsg)

