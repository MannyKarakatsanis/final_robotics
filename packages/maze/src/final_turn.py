#!/usr/bin/env python3                                                                                                                                                                                                                                                            



import rospy
from std_msgs.msg import *
from duckietown_msgs.msg import Twist2DStamped



class DriveSquare:
  def __init__(self):
    self.pub = rospy.Publisher("/duckietesla/car_cmd_switch_node/cmd",Twist2DStamped,queue_size=10)

  def duckie_stop(self):
    while not rospy.is_shutdown():
      message = Twist2DStamped()
      message.v = 0.0
      message.omega = 0.0
      self.pub.publish(message)

  def duckie_drive(self):
    while not rospy.is_shutdown():
     # message = Twist2DStamped()                                                                                                                                                                                                                                                 
      #message.v = 0.0                                                                                                                                                                                                                                                            
      #message.omega = -5                                                                                                                                                                                                                                                         
      #self.pub.publish(message)                                                                                                                                                                                                                                                  
      #rospy.logwarn("First turn")                                                                                                                                                                                                                                                
      #rospy.sleep(0.8)                                                                                                                                                                                                                                                           
      message = Twist2DStamped()
      message.v = 0.0
      message.omega = 0.0
      self.pub.publish(message)
      rospy.sleep(2)
      message = Twist2DStamped()
      message.v = 0.5
      message.omega = 7
      self.pub.publish(message)
      rospy.logwarn("Second turn")
      rospy.sleep(1.3)



  def stop(event):
    rospy.signal_shutdown("Stop Driving")

if __name__ == '__main__':
  sqr = DriveSquare()
  rospy.init_node('lab2_square',anonymous=True)
  #rospy.on_shutdown(sqr.duckie_stop)                                                                                                                                                                                                                                             
 # rospy.Timer(rospy.Duration(24),stop)                                                                                                                                                                                                                                           
  sqr.duckie_drive()
  rospy.spin()
