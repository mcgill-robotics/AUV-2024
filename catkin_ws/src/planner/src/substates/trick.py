#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64

class Trick(smach.State):
    def __init__(self, control, trick_type):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.trick_type = trick_type

    def execute(self,ud):
        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, -2))
        self.control.rotateEuler((0,0,None))
        
        if self.trick_type == "roll":
            return self.execute_roll()
        elif self.trick_type == "pitch":
            return self.execute_pitch()
        elif self.trick_type == "yaw":
            return self.execute_yaw()

    
    def execute_roll(self):
        print("Starting roll trick")
        try:
            self.control.rotateEuler((120.0, 0, 0))
            self.control.rotateEuler((240.0, 0, 0))
            self.control.rotateEuler((0.0, 0, 0))

            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Roll trick interrupted by user.")
            return 'failure'    
    
    def execute_pitch(self):
        print("Starting pitch trick")
        try:
            self.control.rotateEuler((0, 120.0, 0))
            self.control.rotateEuler((0, 240.0, 0))
            self.control.rotateEuler((0, 0.0, 0))
            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Pitch trick interrupted by user.")
            return 'failure'   
    
    def execute_yaw(self):
        print("Starting yaw trick")
        try:
            self.control.rotateEuler((0,0,120.0))
            self.control.rotateEuler((0,0,240.0))
            self.control.rotateEuler((0,0,0.0))

            print("Completed")
            return 'success'

        except KeyboardInterrupt:
            print("Yaw trick interrupted by user.")
            return 'failure'   