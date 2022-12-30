#!/usr/bin/env python
import time
import json
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from std_msgs.msg import Bool
from move_to.msg import MoveToAction, MoveToGoal
from ai_robotics_lab.srv import FFSolve, FFSolveRequest, FFSolveResponse
from utils.states import GenerateProblem #ReplanningState
from utils.mux import ActivateWP, ActivateStop, ActivateFollow
import mdp_robot.mdp_robot as model

WPs = [('navigate', 28),
	('navigate',35)]

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'], output_keys=['index'])

    def execute(self, userdata):
        userdata.index = 0
        # Wait 5s before (so we can see we go through this state in the GUI)
        time.sleep(5)
        return 'start'

class CallFF(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'], output_keys=['index'])

    def execute(self, userdata):
        userdata.index = 0
        return 'start'

class ChooseWP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to','follow','picture'],
                             input_keys=['index'],
                             output_keys=['index', 'waypoint'])

    def execute(self, userdata):
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(2)
        if userdata.index < len(WPs):
		if WPs[userdata.index][0] == "navigate" :
		    	goal = MoveToGoal()
		    	wp = WPs[userdata.index][1]
		    	goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = model.index2xyroom(wp)
		    	userdata.waypoint = goal
		    	userdata.index = userdata.index + 1
		    	return 'go_to'
		elif WPs[userdata.index][0] == "take_picture" :
			userdata.index = userdata.index + 1
			return 'picture'
        else:
            return 'follow'

class TakePicture(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self,
					topic='/robot/CameraMain',
                             		msg_type=Bool,
					cond_cb=self.callback,
					max_checks=1)

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
        # Follow during 100s
        time.sleep(100)
        ActivateStop()
        return 'finished'




def main():
    rospy.init_node('my_mission')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:

	def ff_result_cb(userdata, ff_result):


		print('\n')
		print('\n\n')
		result = json.loads(ff_result.solution)

		global WPs	
		test = []
		for item in result:
			test.append((item[1][0].lower(),int(item[1][3][3:])) if item[1][3][0] != 'O' else (item[1][0].lower(),))
			

		WPs = test
		print(WPs)


		return 'succeeded'

        smach.StateMachine.add('INIT', InitState(),
                               transitions={'start': 'GENERATE_PROB'})

        smach.StateMachine.add('CHOOSE_WP', ChooseWP(),
                               transitions={'go_to': 'MOVE_TO',
                                            'follow': 'FOLLOWING',
					    'picture': 'TAKE_PICTURE'})

        smach.StateMachine.add('GENERATE_PROB', GenerateProblem(),
                               transitions={'generated': 'SOLVE_FF'})

    	smach.StateMachine.add('REPLANNING', ReplanningState(),
                               transitions={'replanned': 'SOLVE_FF'})

        smach.StateMachine.add('MOVE_TO', MoveToWP(),
                               transitions={'succeeded': 'CHOOSE_WP',
					    'aborted': 'aborted'})

        smach.StateMachine.add('FOLLOWING', FollowingState(),
                               transitions={'finished':'INIT'})

	    smach.StateMachine.add('TAKE_PICTURE', TakePicture(),
                               transitions={'valid': 'CHOOSE_WP', 'invalid': 'GENERATE_PROB'})

	    smach.StateMachine.add('SOLVE_FF',
		                       ServiceState('solve',
		                                    FFSolve,
		                                    request = FFSolveRequest('/home/etdisc/Bureau/catkin_ws/src/ai-robotics-lab/pddl/my_nav_domain.pddl','/home/etdisc/Bureau/catkin_ws/src/ai-robotics-lab/pddl/my_nav_problem.pddl'),
						    response_cb = ff_result_cb,
						    output_keys = ['solution']),
		                       transitions={'succeeded':'CHOOSE_WP'})


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
