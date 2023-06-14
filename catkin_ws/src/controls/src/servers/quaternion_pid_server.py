#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from servers.base_server import BaseServer
import actionlib
from auv_msgs.msg import QuaternionAction
import numpy as np
import quaternion


class QuaternionServer():

    def __init__(self):
        self.establish_pid_publishers()
        self.establish_state_subscribers()
        self.server = actionlib.SimpleActionServer('quaternion_server', QuaternionAction, execute_cb= self.callback, auto_start = False)
        
        self.goal = None
        self.body_quaternion = None
        self.position = None
        pose_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        self.Kp = [0, 0, 0, 1]
        self.Ki = [0, 0, 0, 1]
        self.Kd = [0, 0, 0, 1]
        self.error = [0, 0, 0, 1]
        self.integral_error = [0, 0, 0, 1]
        self.dt = 0
        self.derivative_error = [0, 0, 0, 1]
        
        self.control_effort_pub = rospy.Publisher('[something]', Float64, queue_size=50)
        

    def pose_callback(self, data):
        self.position = [data.position.x, data.position.y, data.position.z]
        self.body_quaternion = np.quaternion(self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z)
    
    def cancel(self):
        self.set_pids(self.pose)
    
    def callback(self, goal):
        if(goal.displace):
            displaced_position,displace_quaternion = self.get_goal_after_displace(goal.pose)

            self.set_pids(displaced_position,displace_quaternion)
        else:
            goal_position = []
            goal_position.append(goal.pose.position.x)
            goal_position.append(goal.pose.position.y)
            goal_position.append(goal.pose.position.z)
            goal_quaternion = np.quaternion(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z)
            self.set_pids(goal_position,goal_quaternion)
        
    def get_goal_after_displace(self, goal_pose):
        new_goal = Pose()
        goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        displacement_quat = np.quaternion(goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z)
        new_goal_quat = self.body_quaternion * displacement_quat
        return goal_position,new_goal_quat

    # Felipe's code
    def convertDestination(self,eulers):
        theta_x , theta_y, theta_z = eulers
        self.q2 = tf.transformations.quaternion_from_euler(theta_x, theta_y, theta_z)         
    
    def calculateError(self, q1, q2):
        q1_inv = [q1[0], q1[1], q1[2], q1[3]]
        error = tf.transformations.quaternion_multiply(q1_inv, q2)
        return error
    
    def calculateDerivativeError(self, error):
        pass
    
    def calculateIntegralError(self, error):
        pass
        
    def controlEffort(self):
        # Calculate error values
        error = self.calculateError(self.body_quaternion, self.goal)
        derivative_error = self.calculateIntegralError(error)
        integral_error = self.calculateInternionalError(error)
        
        # Calculate proportional term
        proportional = self.Kp * error
        # Calculte integral term
        integration = self.Ki * (integral_error + error * self.dt)
        # Calculate derivative term
        derivative = self.Kd * derivative_error 
        
        control_effort = proportional + integration + derivative 
        
        self.control_effort_pub.publish(control_effort)
        