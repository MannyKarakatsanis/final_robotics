#!/usr/bin/env python3
import rospy

class PID:
    def __init__(self, kp_d, ki_d, kd_d, kp_phi, ki_phi, kd_phi):

        self.kp_d = kp_d
        self.ki_d = ki_d
        self.kd_d = kd_d

        self.kp_phi = kp_phi
        self.ki_phi = ki_phi
        self.kd_phi = kd_phi

        self.d_integral = 0.0
        self.db_d_integral = 0.0

        self.phi_integral = 0.0
        self.db_phi_integral = 0.0

        self.d_derivative = 0.0
        self.phi_derivative = 0.0
        
        self.previous_d_error = 0.0
        self.previous_phi_error = 0.0


    def calculate_PID(self, d_error, phi_error, dt):
        
        # integrals
        if dt is not None: 
            self.d_integral += (d_error*dt)
            self.phi_integral += (phi_error*dt)


        # derivitives
        if dt is not None: 
            self.d_derivative = (d_error - self.previous_d_error) / dt
            self.phi_derivative = (phi_error - self.previous_phi_error) / dt

        omega = (self.kp_d*d_error) + (self.kp_phi*phi_error) + (self.ki_d*self.d_integral) + (self.ki_phi*self.phi_integral) + (self.kd_d*self.d_derivative) + (self.kd_phi*self.phi_integral)

        # rospy.logwarn("-------------------------")

        self.previous_d_error = d_error
        self.previous_phi_error = phi_error



        return omega
    def change_gains(self, kp, ki, kd):
        self.kp_d = kp_d
        self.ki_d = ki_d
        self.kd_d = kd_d

        self.kp_phi = kp_phi
        self.ki_phi = ki_phi
        self.kd_phi = kd_phi