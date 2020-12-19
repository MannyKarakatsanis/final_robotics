#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState, BoolStamped
from pid_control import PID
from time import sleep

class Lab3:
    def __init__(self):
        # slow
        self.controller = PID(kp_d=3.6, ki_d=0.19, kd_d=0, kp_phi=2.8, ki_phi=0.01, kd_phi=0)
        # fast
        # self.controller = PID(kp_d=7.0, ki_d=0.25, kd_d=0, kp_phi=4.5, ki_phi=0.02, kd_phi=0)

        rospy.Subscriber("/maze/output/error", LanePose, self.call_controller)

        self.pub = rospy.Publisher("/bib/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.previous_second = None

        self.TurnOmega = 5
        self.TurnSpeed = 0.4
        self.TurnSteps = 60





    def call_controller(self, pose_values):
    
        direction = pose_values.header.frame_id        
    
        if direction == "left":
            self.turnLeft()
            direction == "straight"
        elif direction == "right":
            self.turnRight()
            direction == "straight"

        elif direction == "straight":
        # rospy.logwarn("Emmanuel Karakatsanis LANE FOLLOWING CODE")


            # finding dt
            current_second = rospy.Time.now().to_sec()
            dt = None
            if self.previous_second is not None:
                dt = (current_second - self.previous_second)

            # getting info from the pose
            d = pose_values.d
            phi = pose_values.phi

            # finding error: desired - actual = error
            d_error = 0.0 - d
            phi_error = 0.0 - phi

            # limiting the error maximum
            if d_error > 0.15:
                # rospy.logwarn(f"thresholded d_error before: {d_error}")
                d_error = 0.15
                # rospy.logwarn(f"thresholded d_error after: {d_error}")
            
            if d_error < -0.15:
                # rospy.logwarn(f"thresholded d_error before: {d_error}")
                d_error = -0.15
                # rospy.logwarn(f"thresholded d_error after: {d_error}")
            
            if phi_error > 0.15:
                # rospy.logwarn(f"thresholded phi_error before: {phi_error}")
                phi_error = 0.15
                # rospy.logwarn(f"thresholded d_error after: {phi_error}")
            
            if phi_error < -0.15:
                # rospy.logwarn(f"thresholded d_error before: {phi_error}")
                phi_error = -0.15

            # rospy.logwarn("Emmanuel Karakatsanis LANE FOLLOWING CODE")

            # get the angular velocity
            
            omega = self.controller.calculate_PID(d_error, phi_error, dt)
            v = 0.04
        # for fast use
        # v = 0.19
        
            # controlling the output
            control_duckiebot = Twist2DStamped()
            control_duckiebot.v = v
            control_duckiebot.omega = omega
            self.pub.publish(control_duckiebot)

            # self.previous_second = current_second

    # Stops the robot from moving
    def stop(self):
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        moveMsg.v = 0
        moveMsg.omega = 0
        
        self.pub.publish(moveMsg)
    
    # turns robot for a given parameter of Omega
    def turn(self,pOmega):
        timer = 0
        
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        # moveMsg.v = self.TurnSpeed
        # moveMsg.omega = pOmega
        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # t.drive(timer)
            rate.sleep()
                    # 8 worked wellish
            if timer < 5:

                moveMsg.v = 0.3
                moveMsg.omega = pOmega


                # self.my_msg.header = msg_wheels_cmd.header

                self.pub.publish(moveMsg)
            else: 
                moveMsg.v = 0.0
                moveMsg.omega = 0.0
                self.pub.publish(moveMsg)
                break
            # rospy.logwarn(f"Timer: {timer}")
            timer += 1

    
    # turns the robot to the right       
    def turnRight(self):
        self.turn(-self.TurnOmega)
    
    # turns the robot to the left    
    def turnLeft(self):
        self.turn(self.TurnOmega)    



if __name__ == "__main__":
    rospy.init_node('pid', anonymous=True)    
    Lab3()

    rospy.spin()
