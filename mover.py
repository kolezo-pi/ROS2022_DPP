#!/usr/bin/python3



from math import sqrt, pow, pi
import time
import struct

import serial
import rospy
#import rospkg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import UInt16
from std_msgs.msg import Int16

class Mover:
    speedBooster = 2
    moveVelocity = 0.3
    rotateVelocity = 0.3
    isOn = True

    def __callbackOdometry(self, msg):
        self.odom = msg

    def __init__(self):
        rospy.init_node("mainController_node")
        
        self.pubServoHand = rospy.Publisher("/servoHand", UInt16, queue_size=10)
        self.pubServoChange = rospy.Publisher("/servoChange", UInt16, queue_size=10)
        self.pubServoDown = rospy.Publisher("/servoDown", UInt16, queue_size=10)
        self.pubServoCamera = rospy.Publisher("/servoCamera", UInt16, queue_size=10)
        self.pubMotorCable = rospy.Publisher("/motorCable", Int16, queue_size=10)

        self.pub_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/odom_pose2d", Pose2D, self.__callbackOdometry)
        vel = Twist()
        self.pub_velocity.publish(vel)
        rospy.sleep(0.2)
        rospy.loginfo("INITED SUCCESS!")
        
    def __getDistance(self, startPose, currentPose):
        ans = sqrt(pow(startPose.x - currentPose.x, 2) + pow(startPose.y - currentPose.y, 2))
        print(ans)
        return ans
    def __movePIDregulator(self, error):
        speed = self.moveVelocity

        if error <= 0.15:
            speed = self.moveVelocity * error * 10
            if speed > self.moveVelocity:
                speed = self.moveVelocity
            elif speed < 0.03:
                speed = 0.03
        return speed

    def __rotatePIDregulator(self, error):
        speed = self.rotateVelocity
        if error <= 0.261799:
            speed = self.rotateVelocity * error * 10
            if speed > self.rotateVelocity:
                speed = self.rotateVelocity
            elif speed < 0.03:
                speed = 0.03
        return speed

    def servoHand(self, angle):
        if  0 <= angle <= 180:
            dataPublish = UInt16()
            dataPublish.data = angle
            self.pubServoHand.publish(dataPublish)

    def servoChange(self, angle):
        if  0 <= angle <= 180:
            dataPublish = UInt16()
            dataPublish.data = angle
            self.pubServoChange.publish(dataPublish)

    def servoDown(self, angle):
        if  0 <= angle <= 180:
            dataPublish = UInt16()
            dataPublish.data = angle
            self.pubServoDown.publish(dataPublish)

    def servoCamera(self, angle):
        if  0 <= angle <= 180:
            dataPublish = UInt16()
            dataPublish.data = angle
            self.pubServoCamera.publish(dataPublish)
    
    def motorCableForward(self, speed):
        if  speed <= 255:
            dataPublish = Int16()
            dataPublish.data = speed
            self.pubMotorCable.publish(dataPublish)

    def motorCableBackward(self, speed):
        if  speed <= 255:
            dataPublish = Int16()
            dataPublish.data = -speed
            self.pubMotorCable.publish(dataPublish)

    def forward(self, distance):
        startPose = self.odom
        currentPose = startPose
        cmd_vel = Twist()
        
        #cmd_vel.linear.x = self.moveVelocity
        #self.pub_velocity.publish(cmd_vel)
        lastDistance = -1
        lastSpeed = -1
        while self.__getDistance(startPose, currentPose) < distance and self.isOn:
            if lastDistance != self.__getDistance(startPose, currentPose):
                lastDistance = self.__getDistance(startPose, currentPose)
                if lastSpeed != self.__movePIDregulator(distance-lastDistance):
                    lastSpeed = self.__movePIDregulator(distance-lastDistance)
                    cmd_vel.linear.x = lastSpeed
                    self.pub_velocity.publish(cmd_vel)
            currentPose = self.odom
        
        cmd_vel.linear.x = 0
        self.pub_velocity.publish(cmd_vel)

        print("STOP")
    
    def forwardManual(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.moveVelocity
        self.pub_velocity.publish(cmd_vel)
        time.sleep(0.1)
        self.lastCall = time.time()
        cmd_vel.linear.x = 0
        self.pub_velocity.publish(cmd_vel)



    def backward(self, distance):
        startPose = self.odom
        currentPose = startPose
        cmd_vel = Twist()
        
        #cmd_vel.linear.x = self.moveVelocity
        #self.pub_velocity.publish(cmd_vel)
        lastDistance = -1
        lastSpeed = -1
        while self.__getDistance(startPose, currentPose) < distance and self.isOn:
            if lastDistance != self.__getDistance(startPose, currentPose):
                lastDistance = self.__getDistance(startPose, currentPose)
                if lastSpeed != self.__movePIDregulator(distance-lastDistance):
                    lastSpeed = self.__movePIDregulator(distance-lastDistance)
                    cmd_vel.linear.x = -lastSpeed
                    self.pub_velocity.publish(cmd_vel)
            currentPose = self.odom
        
        cmd_vel.linear.x = 0
        self.pub_velocity.publish(cmd_vel)

    def backwardManual(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = -self.moveVelocity
        self.pub_velocity.publish(cmd_vel)
        time.sleep(0.1)
        self.lastCall = time.time()
        cmd_vel.linear.x = 0
        self.pub_velocity.publish(cmd_vel)

    def __getAngle(self, startPose, currentPose, vector):
        if startPose.theta > 0 and currentPose.theta < 0:
            delta =  abs(2*pi + vector*(currentPose.theta - startPose.theta))
        else:
            delta =  abs(vector*(currentPose.theta - startPose.theta))
        print("DELTA:", delta*180/pi)
        return delta

    def left(self, deltaAngle):
        deltaAngle *= pi/180
        startPose = self.odom
        currentPose = startPose
        cmd_vel = Twist()
        
        #cmd_vel.linear.x = self.moveVelocity
        #self.pub_velocity.publish(cmd_vel)
        lastAngle = -1
        lastSpeed = -1
        while self.__getAngle(startPose, currentPose, 1) < deltaAngle and self.isOn:
            if lastAngle != self.__getAngle(startPose, currentPose, 1):
                lastAngle = self.__getAngle(startPose, currentPose, 1)
                if lastSpeed != self.__rotatePIDregulator(deltaAngle-lastAngle):
                    lastSpeed = self.__rotatePIDregulator(deltaAngle-lastAngle)
                    cmd_vel.angular.z = lastSpeed
                    self.pub_velocity.publish(cmd_vel)
            currentPose = self.odom
        
        cmd_vel.angular.z = 0
        self.pub_velocity.publish(cmd_vel)

    # def left(self, deltaAngle):
    #     deltaAngle *= pi/180
    #     rospy.loginfo(deltaAngle)
    #     startPose = self.odom
    #     currentPose = startPose
    #     cmd_vel = Twist()

    #     cmd_vel.angular.z = self.rotateVelocity
    #     self.pub_velocity.publish(cmd_vel)

    #     while self.__getAngle(startPose, currentPose, 1) < deltaAngle:
    #         currentPose = self.odom

    #     cmd_vel.angular.z = 0.0
    #     self.pub_velocity.publish(cmd_vel)

    def leftManual(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = self.moveVelocity
        self.pub_velocity.publish(cmd_vel)
        time.sleep(0.1)
        self.lastCall = time.time()
        cmd_vel.angular.z = 0
        self.pub_velocity.publish(cmd_vel)

    def right(self, deltaAngle):
        deltaAngle *= pi/180
        startPose = self.odom
        currentPose = startPose
        cmd_vel = Twist()
        
        #cmd_vel.linear.x = self.moveVelocity
        #self.pub_velocity.publish(cmd_vel)
        lastAngle = -1
        lastSpeed = -1
        while self.__getAngle(startPose, currentPose, 1) < deltaAngle and self.isOn:
            if lastAngle != self.__getAngle(startPose, currentPose, 1):
                lastAngle = self.__getAngle(startPose, currentPose, 1)
                if lastSpeed != self.__rotatePIDregulator(deltaAngle-lastAngle):
                    lastSpeed = self.__rotatePIDregulator(deltaAngle-lastAngle)
                    cmd_vel.angular.z = -lastSpeed
                    self.pub_velocity.publish(cmd_vel)
            currentPose = self.odom
        
        cmd_vel.angular.z = 0
        self.pub_velocity.publish(cmd_vel)
    # def right(self, deltaAngle):
    #     deltaAngle *= pi/180
    #     rospy.loginfo(deltaAngle)
    #     startPose = self.odom
    #     currentPose = startPose
    #     cmd_vel = Twist()

    #     cmd_vel.angular.z = -self.rotateVelocity
    #     self.pub_velocity.publish(cmd_vel)

    #     while self.__getAngle(startPose, currentPose, -1) < deltaAngle:
    #         currentPose = self.odom

    #     cmd_vel.angular.z = 0.0
    #     self.pub_velocity.publish(cmd_vel)

    def rightManual(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = -self.moveVelocity
        self.pub_velocity.publish(cmd_vel)
        time.sleep(0.1)
        self.lastCall = time.time()
        cmd_vel.angular.z = 0
        self.pub_velocity.publish(cmd_vel)


robot = Mover()

ser = serial.Serial(port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', baudrate = 19200)
ser.reset_input_buffer()
ser.reset_output_buffer()

def selectCommand(classMover, cmd):

    if   cmd[0] == int("00", 16):
        print("STOP")
        classMover.isOn = False

    elif cmd[0] == int("0B", 16):
        print("RESUME")
        classMover.isOn = True

    elif cmd[0] == int("0C", 16):
        if   cmd [1] == int("01", 16):
            print("MOVE_VELOCITY")
            classMover.moveVelocity = cmd[2]/100

        elif cmd [1] == int("10", 16):
            print("ROTATE_VELOCITY")
            classMover.rotateVelocity = cmd[2]/100

    elif cmd[0] == int("01", 16):
        if cmd[2] != int("FF", 16):
            print("FORWARD")
            _S = cmd[1]+cmd[2]/100
            classMover.forward(_S)
        else:
            print("FORWARD MANUAL")
            classMover.forwardManual()

    elif cmd[0] == int("B1", 16):
        print("BOOST FORWARD")
        classMover.moveVelocity *= classMover.speedBooster
        _S = cmd[1]+cmd[2]/100
        classMover.forward(_S)
        classMover.moveVelocity /= classMover.speedBooster

    elif cmd[0] == int("02", 16):
        if cmd[2] != int("FF", 16):
            print("BACKWARD")
            _S = cmd[1]+cmd[2]/100
            classMover.backward(_S)
        else:
            print("BACKWARD MANUAL")
            classMover.backwardManual()

    elif cmd[0] == int("B2", 16):
        print("BOOST BACKWARD")
        classMover.moveVelocity *= classMover.speedBooster
        _S = cmd[1]+cmd[2]/100
        classMover.forward(_S)
        classMover.moveVelocity /= classMover.speedBooster

    elif cmd[0] == int("03", 16):
        if   cmd [1] == int("01", 16):
            if cmd[2] != int("FF", 16):
                print("RIGHT")
                classMover.right(cmd[2])
            else:
                print("MANUAL RIGHT")
                classMover.rightManual()

        elif cmd [1] == int("10", 16):
            if cmd[2] != int("FF", 16):
                print("LEFT")
                classMover.left(cmd[2])
            else:
                print("MANUAL LEFT")
                classMover.leftManual()

    elif cmd[0] == int("A1", 16):
        classMover.servoHand(cmd[2])
    
    elif cmd[0] == int("A2", 16):
        classMover.servoChange(cmd[2])
    
    elif cmd[0] == int("A3", 16):
        classMover.servoDown(cmd[2])
    
    elif cmd[0] == int("A5", 16):
        classMover.servoCamera(cmd[2])
    
    elif cmd[0] == int("A4", 16):
        if   cmd[1] == int("01", 16):
            classMover.motorCableForward(cmd[2])

        elif cmd[1] == int("10", 16):
            classMover.motorCableBackward(cmd[2])

    else:
        print("UNKNOWN COMMAND:", cmd)

#print(f"SEC: {time.time()}")

while True:
    if ser.in_waiting > 0:
        inWait = ser.in_waiting
        rospy.sleep(0.1)

        while inWait != ser.in_waiting:
            inWait = ser.in_waiting
            rospy.sleep(0.1)
        #raw = ser.read(size=ser.in_waiting)
        #print(raw)
        data = list(struct.unpack(">"+ str(ser.in_waiting) +"B", ser.read(size=ser.in_waiting)))

        if len(data) % 3 == 0:
            cmd = [(data[x*3], data[x*3+1], data[x*3+2]) for x in range(len(data)//3)]
            print("PACKET:", cmd)
            for i in cmd:
                selectCommand(robot, i)
                rospy.sleep(0.5)
                print(i)
        else:
            print("INCORRECT DATA!")


#ROS_MASTER_URI=http://192.168.50.112:11311 rqt