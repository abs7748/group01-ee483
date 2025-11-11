#!/usr/bin/env python3

import rospy
import os
import math
from duckietown_msgs.msg import WheelsCmdStamped # Import the message for the wheel comm



class Driver():#CHANGE CLASSNAME to the name of your class
    def __init__(self):
        
        #rospy.Subscriber("lane_follow", , self.image_callback, queue_size=1, buff_size=10000000)
        #rospy.Subscriber("direction", , self.image_callback, queue_size=1, buff_size=10000000)
        self.pub = rospy.Publisher('/ee483mm01/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.cmd_pub = WheelsCmdStamped()

        #initialize states
        #self.state = 0
        #self.count = 0
        #self.state_started = rospy.Time.now()


        #durations
        #self.forward_time = rospy.Duration(3)
        #self.turn_time = rospy.Duration((math.pi/2)/1)

    def move_straight(self,speed =0.4, duration=2.5):
        self.cmd_pub.vel_right = speed
        self.cmd_pub.vel_left = speed + .2
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
            self.pub.publish(self.cmd_pub)
            rospy.sleep(.001)

        self.stop()

    def move_straight_slow(self,duration=.5,speed =0.2):
        self.cmd_pub.vel_right = speed
        self.cmd_pub.vel_left = speed + .2
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
            self.pub.publish(self.cmd_pub)
            rospy.sleep(.001)
    


    def turn(self,direction, duration=0.8):
        self.move_straight_slow(2,0.2)
        self.stop(2)
        if direction == 'left':
            self.cmd_pub.vel_right = 0.2
            self.cmd_pub.vel_left = -0.2
        if direction == 'right':
            self.cmd_pub.vel_right = -0.2
            self.cmd_pub.vel_left = 0.2
        
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
            self.pub.publish(self.cmd_pub)
            rospy.sleep(0.001)
        self.stop(2)
        self.move_straight_slow(1.75,0.2)
        self.stop(2)
        

    def stop(self,duration = 5):
        self.cmd_pub.vel_right = 0
        self.cmd_pub.vel_left = 0
        self.pub.publish(self.cmd_pub)
        rospy.sleep(duration)
    
    def go(self, side_length = 2.0):
        self.move_straight_slow(.1)
        self.move_straight(.4,.1)
        rospy.loginfo(f"turning")
        self.stop()
        self.turn('left')


    def talk(self): #defines method

        now = rospy.Time.now()
        elapsed = now-self.state_started

        #new message object we will added the turtle's movements
        if self.state == 2: #stop 
            self.cmd_pub.vel_right = 0
            self.cmd_pub.vel_left = 0

        elif self.state == 0: #move forward
            self.cmd_pub.vel_right = 0.5
            self.cmd_pub.vel_left = 0.5
            
            if elapsed >= self.forward_time:
                self.count += 1 
                self.state = 2
                self.state_started = now


        
         # publishes the Twist msg to the topic
        self.cmd_pub.header.stamp = rospy.Time.now()
        self.pub.publish(self.cmd_pub)
        
        rospy.loginfo(f"State = {self.state}, CouWnt = {self.count}, Elapsed={elapsed.to_sec():.2f}s")
        
        

if __name__ == "__main__": ## The main function which will be called when your python sc
# Initialize the node
    try:
        rospy.init_node('driving')
        drive = Driver() # Create obj of the Driver class
        rospy.sleep(3) # Delay to wait enough time for the code to run
        drive.go()
        #rate = rospy.Rate(20)
        # Keep the line above - you might be able to reduce the delay a bit,
        #while not rospy.is_shutdown(): # Run ros forever - you can change
        #    drive.talk()
        #    rate.sleep()
# this as well instead of running forever
        #    drive.drive() # calling your node function
    except rospy.ROSInterruptException:
        pass