#!/usr/bin/env python3
  
import rospy
import rostest
import unittest
import numpy as np
from object_detection_utils import transformLocalToGlobal

@TODO - Come up with some examples + calculate the expected results by hand
class transform_local_to_global(unittest.TestCase):
    """ 
        Given a vector relative to the auv, turns local measurements to global
        params: lx, ly, lz, camera_id, yaw_offset=0
    """
    def test__Vector1(self):
        local_vector = [1, 1, 1]
        real_global_vector = [1, 1, 1]
        estimated_global_vector = transformLocalToGlobal(local_vector[0], local_vector[1], local_vector[2], camera_id=0, yaw_offset=0)
        self.assertTrue(np.allclose(estimated_global_vector, real_global_vector))

    def test__Vector2(self):
        local_vector = [1, 1, 1]
        real_global_vector = [1, 1, 1]
        estimated_global_vector = transformLocalToGlobal(local_vector[0], local_vector[1], local_vector[2], camera_id=0, yaw_offset=0)
        self.assertTrue(np.allclose(estimated_global_vector, real_global_vector))
    
    def test__Vector3(self):
        local_vector = [1, 1, 1]
        real_global_vector = [1, 1, 1]
        estimated_global_vector = transformLocalToGlobal(local_vector[0], local_vector[1], local_vector[2], camera_id=0, yaw_offset=0)
        self.assertTrue(np.allclose(estimated_global_vector, real_global_vector))

    def test__Vector4(self):
        local_vector = [1, 1, 1]
        real_global_vector = [1, 1, 1]
        estimated_global_vector = transformLocalToGlobal(local_vector[0], local_vector[1], local_vector[2], camera_id=0, yaw_offset=0)
        self.assertTrue(np.allclose(estimated_global_vector, real_global_vector))
        

if __name__ == '__main__':
    # rospy.init_node("transform_local_to_global") - Already initialized in object_detection_utils.py
    rostest.rosrun("vision", 'transform_local_to_global', transform_local_to_global)