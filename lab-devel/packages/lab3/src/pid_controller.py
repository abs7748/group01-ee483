#!/usr/bin/env python3
import sys
import rospy
import cv2
import matplotlib
import numpy as np
import os
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose, Twist2DStamped

class PID:
    def __init__(self):
        

        self.phi = [0,0,0,0,0]


        self.car_cmd = Twist2DStamped()

        self.Kp = 5
        self.Ki = 0.01
        self.Kd = 0.1
        self.v = 0.0

        

        # Error terms
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.get_time()



        # ROS setup
        rospy.Subscriber("/ee483mm01/lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub_control = rospy.Publisher("/ee483mm01/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.pub_phi = rospy.Publisher("/filtered_phi", Float32, queue_size=10)
        


    def callback(self, msg):
        
        self.phi.append(msg.phi)
        del self.phi[0]
        average = 0
        sum = 0
        
        for i in range(len(self.phi)): 
            sum += self.phi[i] 
        average = sum/len(self.phi)
        self.pub_phi.publish(average)


        if rospy.has_param("Kp"):
            self.Kp = rospy.get_param("Kp")
        
        if rospy.has_param("Ki"):
            self.Ki = rospy.get_param("Ki")

        if rospy.has_param("Kd"):
            self.Kd = rospy.get_param("Kd")

        if rospy.has_param("/v"):
            self.v = rospy.get_param("/v")
       

        try:
            error = -average
            current_time = rospy.get_time()
            dt = current_time - self.prev_time if self.prev_time else 0.01

            # PID calculations
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

            controller_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

            # Publish control input
            #self.pub_control.publish(Float32(control))

            # Update previous values
            self.prev_error = error
            self.prev_time = current_time

            self.car_cmd.v = self.v
            self.car_cmd.omega = controller_output
            self.pub_control.publish(self.car_cmd)

        except Exception as e:
            rospy.logerr(f"callback error: {e}")



if __name__ == '__main__':

    try:
        rospy.init_node('PID', anonymous=True)
        PID()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass