#!/usr/bin/env python3

import rospy
import os
import math


class MasterControl: 
    def __init__(self):
        self.current_state = 'go'
        self.initial_turn = 'left'
        self.all_turns = [turn1,turn2,turn3]
        self.count = 0

        rospy.Subscriber("nav_condition", string, self.nav_callback, queue_size=1, buff_size=10000000)
        self.pub_direction = rospy.Publisher("direction", msg, queue_size=10)

        self.pub_direction.Publish(self.initial_turn)

    def nav_callback(self,msg):
        if msg == "lane detected":
            rospy.sleep(10)
            self.turn = self.all_turns[self.count]
            self.count += 1
            




