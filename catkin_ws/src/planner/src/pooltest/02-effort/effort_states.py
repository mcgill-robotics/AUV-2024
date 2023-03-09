#!/usr/bin/env python3

import rospy
import smach

from geometry_msgs.msg import Wrench, Vector3, Vector3
from std_msgs.msg import Float64

class EffortMotion(smach.State):
    def __init__(self, wrench, duration=2.0):
        super().__init__(outcomes=['done'])
        self.effort = wrench
        self.duration = duration
        self.pub = rospy.Publisher('effort', Wrench, queue_size=50)
    
    def execute(self, ud):
        timer = rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.sleep(self.duration)

        # this is a hack to stop publishing
        timer.shutdown()
        return 'done'

    def update(self, _):
        self.pub.publish(self.effort) 

class Rotate(smach.State):
    def __init__(self,effort=10.0):
        super().__init__(outcomes=['done'])
        force = Vector3(x=0.0, y=0.0, z=0.0)
        torque = Vector3(x=0.0, y=0.0, z=effort)
        self.effort = Wrench(force=force, torque=torque)
        self.theta_z = 0
        self.sub = rospy.Subscriber("state_theta_z",Float64,self.set_theta_z)
        self.pub = rospy.Publisher('effort', Wrench, queue_size=50)

    def set_theta_z(self,data):
        self.theta_z = data.data

    def execute(self,ud):
        start = self.theta_z
        force = Vector3(x=0.0, y=0.0, z=0.0)
        torque = Vector3(x=0.0, y=0.0, z=0.0)
        zero = Wrench(force=force,torque=torque)
        DELTA = 2
        rate = rospy.Rate(60)
        while(abs(self.theta_z - start) < 180 - DELTA):
            self.pub.publish(self.effort)
            rate.sleep()
        
        self.pub.publish(zero)
        return 'done'

class Surge(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Vector3(x=effort, y=0.0, z=0.0)
        torque = Vector3(x=0.0, y=0.0, z=0.0)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)


class Sway(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Vector3(x=0.0, y=effort, z=0.0)
        torque = Vector3(x=0.0, y=0.0, z=0.0)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)


class Heave(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Vector3(x=0.0, y=0.0, z=effort)
        torque = Vector3(x=0.0, y=0.0, z=0.0)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)


class Roll(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Vector3(x=0.0, y=0.0, z=0.0)
        torque = Vector3(x=effort, y=0.0, z=0.0)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)


class Pitch(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Vector3(x=0.0, y=0.0, z=0.0)
        torque = Vector3(x=0.0, y=effort, z=0.0)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)


class Yaw(EffortMotion):
    def __init__(self, effort=10.0, duration=10.0):
        force = Vector3(x=0.0, y=0.0, z=0.0)
        torque = Vector3(x=0.0, y=0.0, z=effort)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)


class Pause(EffortMotion):

    def __init__(self, duration=2.0):
        force = Vector3(x=0.0, y=0.0, z=0.0)
        torque = Vector3(x=0.0, y=0.0, z=0.0)
        wrench = Wrench(force=force, torque=torque)
        super().__init__(wrench, duration)
