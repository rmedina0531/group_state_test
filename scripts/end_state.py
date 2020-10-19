#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Int16, String
import time
import threading

class End_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])
        self.sub = rospy.Subscriber('number_count', Int16, self.callback)
        self.pub = rospy.Publisher('data_recorder', String, queue_size=1)
        
        self.mutex = threading.Lock()

        self.elapsed_time = 0
        self.data = None
        self.start_time = time.time()


    def callback(self, data):
        self.pub.publish(str(data))
        self.mutex.acquire()
        self.elapsed_time = 0
        self.start_time = time.time()
        self.mutex.release()

    def execute(self, userdata):
        while(True):
            self.mutex.acquire()
            self.elapsed_time = time.time() - self.start_time
            self.mutex.release()
            if self.elapsed_time > 1:
                break

        self.pub.publish("Publisher Silent")
        exit()



if __name__ == '__main__':
    rospy.init_node('test_state_machine')

    #create a SMACH state machine
    sm = smach.StateMachine(outcomes=['completed'])

    #open the container
    with sm:
        smach.StateMachine.add('my_state', End_State(), 
                                transitions={'end':'completed'})

    outcome = sm.execute()
