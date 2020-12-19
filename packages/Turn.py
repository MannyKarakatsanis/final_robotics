#!/usr/bin/env python3

import rospy
import random
from time import sleep
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class MoveRobot:
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.TurnOmega = 4
        self.TurnSpeed = 0.5
        self.TurnSteps = 62

    # Stops the robot from moving
    def stop(self):
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        moveMsg.v = 0
        moveMsg.omega = 0
        
        self.pub.publish(moveMsg)
    
    # turns robot for a given parameter of Omega
    def turn(self,pOmega):
        counter = 0
        
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        moveMsg.v = self.TurnSpeed
        moveMsg.omega = pOmega
        
        while not rospy.is_shutdown():
            self.pub.publish(moveMsg)
            counter = counter + 1
            sleep(0.01)
            if counter > self.TurnSteps:
                duckieRobot.stop()
                sleep(1)
                break   
    
    # turns the robot to the right       
    def turnRight(self):
        duckieRobot.turn(-self.TurnOmega)
    
    # turns the robot to the left    
    def turnLeft(self):
        duckieRobot.turn(self.TurnOmega)        

if __name__ == '__main__':
    try:
        duckieRobot = MoveRobot()
        rospy.init_node('MoveRobot', anonymous=True)
        duckieRobot.turnLeft()
        duckieRobot.turnRight()

 
    except rospy.ROSInterruptException:
        rospy.logwarn("error encountered")
