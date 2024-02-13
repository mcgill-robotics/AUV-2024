#!/usr/bin/env python3
  
import rospy
import rostest
import unittest
import numpy as np
from object_detection_utils import transformLocalToGlobal

# @TODO - Record img data, bbox data 
class visualize_bbox_test(unittest.TestCase):
    """ 
        Given an image, class name, and a bounding box, draws the bounding box rectangle and label name onto the image
        params: img, bbox, class_name, thickness, fontSize
    """
    def test__Vector1(self):
        pass
        

if __name__ == '__main__':
    # rospy.init_node("visualize_bbox_test") - Already initialized in object_detection_utils.py
    rostest.rosrun("vision", 'visualize_bbox_test', visualize_bbox_test)