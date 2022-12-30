#!/usr/bin/env python
import time
import json
import rospy
import smach
import smach_ros



from smach_ros import ServiceState

from move_to.msg import MoveToAction, MoveToGoal
from ai_robotics_lab.srv import FFSolve, FFSolveRequest, FFSolveResponse
from std_msgs.msg import String, Bool

from utils.mux import ActivateWP, ActivateStop, ActivateFollow
import mdp_robot.mdp_robot as model

WP = [('navigate', 11),('navigate', 35), ('take_picture', )]

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
        smach.State.__init__(self, outcomes=['go_to', 'follow', 'take_picture'],
                             input_keys=['index'],
                             output_keys=['index', 'waypoint'])

    def execute(self, userdata):
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(2)
        if userdata.index < len(WP):
		wp = WP[userdata.index]
		userdata.index = userdata.index + 1
		if wp[0] == 'navigate':	
		    goal = MoveToGoal()
		    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = model.index2xyroom(wp[1])
		    userdata.waypoint = goal
	  	    return 'go_to'
		elif wp[0] == 'take_picture':
		    return 'take_picture'
	else:
		return 'follow'

class TakePicture(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self,
					topic = 'robot/CameraMain',
					msg_type = Bool,
					cond_cb = self.callback,
					max_checks = 1
					)

    def callback(self, ud, msg):
        return msg.data

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
    rospy.init_node('my_mission_WP')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:


        smach.StateMachine.add('INIT', InitState(),
                               transitions={'start': 'FF'})


        smach.StateMachine.add('CHOOSE_WP', ChooseWP(),
                               transitions={'go_to': 'MOVE_TO',
                                            'follow': 'FOLLOWING',
					    'take_picture': 'TAKE_PICTURE'})

        smach.StateMachine.add('TAKE_PICTURE', TakePicture(),
                               transitions= {'valid': 'CHOOSE_WP', 'invalid': 'aborted'})

        smach.StateMachine.add('MOVE_TO', MoveToWP(),
                              transitions= {'succeeded': 'CHOOSE_WP', 'aborted': 'aborted'})

        smach.StateMachine.add('FOLLOWING', FollowingState(),
                               transitions={'finished':'INIT'})

	smach.StateMachine.add('FF', 
				ServiceState('ff_wrapper',
				      FFSolve,
				      request = FFSolveRequest('/home/etdisc/Bureau/catkin_ws/src/ai-robotics-lab/pddl/my_nav_domain.pddl', '/home/etdisc/Bureau/catkin_ws/src/ai-robotics-lab/pddl/my_nav_pb.pddl')
					
					    ),
				transitions={'succeeded': 'CHOOSE_WP'}
	                      )

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
