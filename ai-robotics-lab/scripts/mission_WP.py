#!/usr/bin/env python
import time
import json
import rospy
import smach
import smach_ros

from move_to.msg import MoveToAction, MoveToGoal
from ai_robotics_lab.srv import FFSolve, FFSolveRequest, FFSolveResponse

from utils.mux import ActivateWP, ActivateStop, ActivateFollow
import mdp_robot.mdp_robot as model

WPs = [11,21,35]

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'], output_keys=['index'])

    def execute(self, userdata):
        userdata.index = 0
        # Wait 5s before (so we can see we go through this state in the GUI)
        time.sleep(5)
        return 'start'

class ChooseWP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to', 'follow'],
                             input_keys=['index'],
                             output_keys=['index', 'waypoint'])

    def execute(self, userdata):
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(2)
        if userdata.index < len(WPs):
            goal = MoveToGoal()
            wp = WPs[userdata.index]
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = model.index2xyroom(wp)
            userdata.waypoint = goal
            userdata.index = userdata.index + 1
            return 'go_to'
        else:
            return 'follow'

class MoveToWP(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self,'move_to',
                                             MoveToAction,
                                             goal_key='waypoint')

    def execute(self, userdata):
        ActivateWP()
        outcome = smach_ros.SimpleActionState.execute(self, userdata)
        ActivateStop()
        return outcome

class FollowingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        ActivateFollow()
        # Follow during 10s
        time.sleep(10)
        ActivateStop()
        return 'finished'

def main():
    rospy.init_node('mission_WP')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:

        smach.StateMachine.add('INIT', InitState(),
                               transitions={'start': 'CHOOSE_WP'})

        smach.StateMachine.add('CHOOSE_WP', ChooseWP(),
                               transitions={'go_to': 'MOVE_TO',
                                            'follow': 'FOLLOWING'})

        smach.StateMachine.add('MOVE_TO', MoveToWP(),
                               {'succeeded': 'CHOOSE_WP', 'aborted': 'aborted'})

        smach.StateMachine.add('FOLLOWING', FollowingState(),
                               transitions={'finished':'INIT'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    time.sleep(5)

    sis.stop()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
