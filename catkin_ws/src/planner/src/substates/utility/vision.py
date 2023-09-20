#!/usr/bin/env python3

import rospy
import math
from auv_msgs.msg import VisionObjectArray

class ObjectMapper:
    def __init__(self):
        #replace with map subscriber in future
        self.map = []
        self.obj_sub = rospy.Subscriber('vision/object_map', VisionObjectArray, self.mapUpdateCb)

    def mapUpdateCb(self,msg):

        for obj in msg.array:
            new_map_obj = []
            new_map_obj.append(obj.label)
            new_map_obj.append(obj.x)
            new_map_obj.append(obj.y)
            new_map_obj.append(obj.z)
            if obj.theta_z == -1234.5: new_map_obj.append(None)
            else: new_map_obj.append(obj.theta_z)
            if obj.extra_field == -1234.5: new_map_obj.append(None)
            else: new_map_obj.append(obj.extra_field)
            self.map.append(new_map_obj)

    def getClass(self,cls=None):
        objs_with_class = []
        for obj in self.map:
            if obj[0] == cls or cls is None:
                objs_with_class.append(obj)
        return objs_with_class

    def getClosestObject(self,pos,cls=None):
        closest_object_dist=999999
        closest_object = None
        for obj in self.map:
            if obj[0] == cls or cls is None:
                dist = math.sqrt((obj[1]-pos[0])**2 + (obj[2]-pos[1])**2)
                if dist < closest_object_dist:
                    closest_object = obj
                    closest_object_dist = dist
        return closest_object

    def updateObject(self, obj):
        obj = self.getClosestObject(pos=(obj[1], obj[2]), cls=obj[0])
