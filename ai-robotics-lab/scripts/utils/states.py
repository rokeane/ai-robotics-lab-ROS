import rospy
import smach
from geometry_msgs.msg import PoseStamped

from .model import *
from .pddl import generate_problem, write_problem, ZONE_MAP, PDDL_FOLDER, OOIs

class GenerateProblem(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['generated'], output_keys=['index'])
        self.subscriber = rospy.Subscriber("/robot/pose", PoseStamped, self.pose_callback)
        self.position = 0
    
    def pose_callback(self, msg):
        self.position = int(xyroom2index(msg.pose.position.x,
                                            msg.pose.position.y))

    def execute(self, userdata):
        userdata.index = 0
        pb = generate_problem('navpb', ZONE_MAP, OOIs, self.position)
        write_problem(PDDL_FOLDER + "my_nav_problem.pddl", pb)
        return 'generated'
'''
class ReplanningState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['replanned'], output_keys=['index'])
        self.subscriber = rospy.Subscriber("/robot/pose", PoseStamped, self.pose_callback)


    
    def pose_callback(self, msg):
        self.position = int(xyroom2index(msg.pose.position.x,
                                            msg.pose.position.y))
	

    def execute(self, userdata):
        userdata.index = 0
	print('Position of robot = ', self.position)
	new_OOIs = {}



	for obj, pos in OOIs.items():
	  if pos != self.position:
	    new_OOIs[obj] = pos
			

        pb = generate_problem('navpb', ZONE_MAP, new_OOIs, self.position)
        write_problem(PDDL_FOLDER + "my_nav_problem.pddl", pb)
        return 'replanned'

'''
