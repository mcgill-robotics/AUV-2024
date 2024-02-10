#!/usr/bin/env python3
  
import rospy
import rostest
import unittest
import numpy as np
from object_detection_utils import measure_angle

@TODO - Record bounding box data
@TODO - Initialize the bounding box variable from the recorded data
class measure_angle_test(unittest.TestCase):
    """ Given a bounding box, returns the angle of the object in degrees (only for front cam) """
    def __init__(self):
        self.bbox = None 
        self.tolerance_degrees = 5

    def test__Gate30Degrees(self):
        target_angle = 30
        estimated_angle = measure_angle(bbox)
        self.assertTrue(abs(estimated_angle - target_angle) < self.tolerance_degrees)
    
    def test__GateMinus60Degrees(self):
        target_angle = -60
        estimated_angle = measure_angle(bbox)
        self.assertTrue(abs(estimated_angle - target_angle) < self.tolerance_degrees)

    def test__Buoy45Degrees(self):
        target_angle = 45
        estimated_angle = measure_angle(bbox)
        self.assertTrue(abs(estimated_angle - target_angle) < self.tolerance_degrees)
    
    def test__BuoyMinus45Degrees(self):
        target_angle = -45
        estimated_angle = measure_angle(bbox)
        self.assertTrue(abs(estimated_angle - target_angle) < self.tolerance_degrees)
        

if __name__ == '__main__':
    # rospy.init_node("measure_angle_test") - Already initialized in object_detection_utils.py
    rostest.rosrun("vision", 'measure_angle_test', measure_angle_test)