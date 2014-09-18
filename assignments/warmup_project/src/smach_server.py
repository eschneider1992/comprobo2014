#!/usr/bin/env python

# Written by Eric Schneider for the CompRobo warmup project
# 9/14/2014
# 
# Stuff about how it works...
# 
# In the code below I couldn't get the input_keys methods to apprpriately
# pass a class object. The sm_robot class was either not passed into the
# states, or it was passed as a constant. I left the code in there for
# documentation and in case I could get it working later


import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from copy import deepcopy

from neato_classes import *
neato = NeatoFollower()

# Find is currently unused because the robot will go forward and align w/ wall
# define state Find
# class Find(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['preempt-avoid', 'finished'], input_keys=['find_robot'])
#         self.result = ''

#     def execute(self, userdata):
#         global nea
#         rospy.loginfo('Executing state FIND')

#         if neato.movement_detected:
#             self.result = 'preempt-avoid'
#         else:
#             self.result = 'finished'
        
#         rospy.loginfo("FIND state is returning %s\n", self.result)
#         return self.result


# define state Follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'finished'], input_keys=['follow_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state FOLLOW')
        
        cmd_align = Vector3()
        cmd_approach = Vector3()
        wall = Twist()
        r = rospy.Rate(10)

        while neato.wall_detected and not rospy.is_shutdown():
            if neato.movement_detected:
                self.result = 'preempt-avoid'
                break
            wall = deepcopy(neato.closest_wall)

            cmd_align.x = cos(wall.angular.z)
            cmd_align.y = sin(wall.angular.z)
            error = vector_mag(wall.linear) - neato.goal_distance
            cmd_approach = vector_multiply(create_unit_vector(wall.linear),
                                            error)
            # rospy.loginfo("cmd_align: \n%s", cmd_align)
            # rospy.loginfo("cmd_approach: \n%s", cmd_approach)

            neato.command_motors(vector_add(cmd_align, cmd_approach))
            r.sleep()

        self.result = 'finished'
        rospy.loginfo("FOLLOW state is returning %s\n", self.result)
        neato.stop()
        return self.result


# define state Wary
class Wary(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alarm', 'false-alarm'], input_keys=['wary_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state WARY')

        # TODO: Put some sort of timeout
        if neato.movement_detected:
            self.result = 'alarm'
        else:
            self.result = 'false-alarm'

        rospy.loginfo("WARY state is returning %s\n", self.result)
        return self.result


# define state Avoid
class Avoid(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'],
                             input_keys=['avoid_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state AVOID')

        self.result = 'finished'
        rospy.loginfo("AVOID state is returning %s\n", self.result)
        return self.result


# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'preempt-follow',
                             'finished'], input_keys=['idle_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state IDLE')

        rospy.logdebug('neato.movement_detected: %s', neato.movement_detected)
        rospy.logdebug('neato.wall_detected: %s\n', neato.wall_detected)
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            if neato.movement_detected:
                self.result = 'preempt-avoid'
                break
            elif neato.wall_detected:
                self.result = 'preempt-follow'
                break
            else:
                pass
                v = Vector3()
                neato.command_motors(v)
                r.sleep()

        rospy.loginfo("IDLE state is returning %s\n", self.result)
        return self.result

# main
def main():
    # Unnecessary b/c NeatoFollower() calls init_node
    # rospy.init_node('warmup_state_machine')
    # rospy.sleep(0.5)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm-finished']) #,
                            # input_keys=['sm_robot'])
    try:
        sm.userdata.sm_robot = NeatoFollower()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error! %s", e)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                                transitions={'preempt-avoid':'AVOID',
                                            'preempt-follow':'FOLLOW',
                                            # 'finished':'WARY'},
                                            'finished':'sm-finished'},
                                remapping={'idle_robot':'sm_robot'})
        smach.StateMachine.add('WARY', Wary(), 
                                transitions={'alarm':'AVOID',
                                            'false-alarm':'IDLE'},
                                remapping={'wary_robot':'sm_robot'})
        smach.StateMachine.add('AVOID', Avoid(), 
                               transitions={'finished':'IDLE'},
                               remapping={'avoid_robot':'sm_robot'})
        # smach.StateMachine.add('FIND', Find(), 
        #                        transitions={'preempt-avoid':'AVOID', 
        #                                     'finished':'FOLLOW'},
        #                         remapping={'find_robot':'sm_robot'})
        smach.StateMachine.add('FOLLOW', Follow(), 
                               transitions={'preempt-avoid':'AVOID', 
                                            'finished':'IDLE'},
                                remapping={'follow_robot':'sm_robot'})

    # Add introspection
    # sis = smach_ros.IntrospectionServer('view_smach', sm, '/SM_ROOT')
    # sis.start() 

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for script to end (cntl-c)
    rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    main()