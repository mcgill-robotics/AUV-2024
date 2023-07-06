#!/usr/bin/env python3

from in_place_search import InPlaceSearch
import rospy
import smach
from .utility.vision import *



class NavigateOctagon(smach.State):
    def __init__(self, control, state, mapping, octagon_class):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.octagon_class = octagon_class

    def execute(self, ud):
        print("Starting octagon navigation.") 
        auv_current_position = (self.state.x, self.state.y)
       
        octagon_obj = self.mapping.getClosestObject(self.octagon_class, (auv_current_position[0], auv_current_position[1]))
        
        if octagon_obj is None:
            print("No octagon in object map! Failed.")
            return 'failure'
        
        print("Moving up to avoid the buoy.")
        self.control.move((auv_current_position[0], auv_current_position[1], -1))
        print("Moving to the center of the octagon.")
        self.control.move((octagon_obj[1], octagon_obj[2], -1))
        print("Surfacing.")
        self.control.kill()

        print("Successfully navigated through the octagon.")
        return 'success'
    


class GoToOctagon(smach.State):
    def __init__(self,search_point, control):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.search_point = search_point

    def execute(self, ud):
        self.control.move((self.search_point[0], self.search_point[1], -1))
        return 'success'