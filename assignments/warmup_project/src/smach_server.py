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

from neato_classes import NeatoFollower
neato = NeatoFollower()


# define state Find
class Find(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'finished'], input_keys=['find_robot'])
        self.result = ''

    def execute(self, userdata):
        global nea
        rospy.loginfo('Executing state FIND')

        if neato.movement_detected:
            self.result = 'preempt-avoid'
        else:
            self.result = 'finished'
        
        rospy.loginfo("FIND state is returning %s\n", self.result)
        return self.result


# define state Follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'finished'], input_keys=['follow_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state FOLLOW')

        if neato.movement_detected:
            self.result = 'preempt-avoid'
        else:
            self.result = 'finished'
        
        rospy.loginfo("FOLLOW state is returning %s\n", self.result)
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
        smach.State.__init__(self, outcomes=['finished'], input_keys=['avoid_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state AVOID')

        neato.movement_detected = False
        
        self.result = 'finished'
        rospy.loginfo("AVOID state is returning %s\n", self.result)
        return self.result


# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'preempt-follow', 'finished'], input_keys=['idle_robot'])
        self.result = ''

    def execute(self, userdata):
        global neato
        rospy.loginfo('Executing state IDLE')

        rospy.sleep(1.0)

        if neato.movement_detected:
            self.result = 'preempt-avoid'
        elif neato.wall_detected:
            self.result = 'preempt-follow'
        else:
            # pass
            self.result = 'finished'
        rospy.loginfo("IDLE state is returning %s\n", self.result)
        return self.result

# main
def main():
    rospy.init_node('warmup_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm-finished']) #,
                            # input_keys=['sm_robot'])
    sm.userdata.sm_robot = NeatoFollower()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                                transitions={'preempt-avoid':'AVOID',
                                            'preempt-follow':'FIND',
                                            'finished':'WARY'},
                                            # 'finished':'sm-finished'},
                                remapping={'idle_robot':'sm_robot'})
        smach.StateMachine.add('WARY', Wary(), 
                                transitions={'alarm':'AVOID',
                                            'false-alarm':'IDLE'},
                                remapping={'wary_robot':'sm_robot'})
        smach.StateMachine.add('AVOID', Avoid(), 
                               transitions={'finished':'IDLE'},
                               remapping={'avoid_robot':'sm_robot'})
        smach.StateMachine.add('FIND', Find(), 
                               transitions={'preempt-avoid':'AVOID', 
                                            'finished':'FOLLOW'},
                                remapping={'find_robot':'sm_robot'})
        smach.StateMachine.add('FOLLOW', Follow(), 
                               transitions={'preempt-avoid':'AVOID', 
                                            'finished':'IDLE'},
                                remapping={'follow_robot':'sm_robot'})

    # Add introspection
    sis = smach_ros.IntrospectionServer('view_smach', sm, '/SM_ROOT')
    sis.start() 

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for script to end (cntl-c)
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()