#!/usr/bin/env python3

import rospy
import smach

from sub_states import *
from sub_states.utility import *

def descend(depth=-2.0):
    descended = False
    def done():
        global descended
        descended = True
    controller.moveDelta((0, 0, depth), done)
    while not descended: rospy.sleep(0.1)

def endMission(msg="Shutting down mission planner."):
    print(msg)
    controller.preemptCurrentAction()
    controller.velocity((0,0,0))
    controller.angularVelocity((0,0,0))

def testRotationsMission():
    descend(depth=-2.0)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('test_submerged_rotations', test_submerged_rotations.TestSubmergedRotations(hold_time = 5.0), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished rotation test mission.")

def laneMarkerGridSearchMission():
    descend(depth=-2.0)
    sm = smach.StateMachine(outcomes=['success', 'failure']) 
    with sm:
        smach.StateMachine.add('gridsearch', grid_search.GridSearch(target_class=0, timeout=60, min_consecutive_detections=5), 
                transitions={'success': 'navigateLaneMarker', 'failure':'failure'})
        smach.StateMachine.add('navigateLaneMarker', navigate_lane_marker.NavigateLaneMarker(), 
                transitions={'success': 'success', 'failure':'failure'})
    res = sm.execute()
    endMission("Finished lane marker grid search mission. Result: {}".format(res))

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    rospy.on_shutdown(endMission)

    # ----- UNCOMMENT BELOW TO RUN MISSION(S) -----

    testRotationsMission()
    #laneMarkerGridSearchMission()
