#! /usr/bin/env python

import rospy

import actionlib

import robodk_postprocessors.msg

class GenerateProgramAction(object):
    # create messages that are used to publish feedback/result
    _feedback = robodk_postprocessors.msg.GenerateProgramFeedback()
    _result = robodk_postprocessors.msg.GenerateProgramResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, robodk_postprocessors.msg.GenerateProgramAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing!' % (self._action_name))

        # check that preempt has not been requested by the client
        #if self._as.is_preempt_requested():
        #    rospy.loginfo('%s: Preempted' % self._action_name)
        #    self._as.set_preempted()
        #    success = False
        #    break

        # publish the feedback
        #self._as.publish_feedback(self._feedback)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('robodk_postprocessors')
    server = GenerateProgramAction(rospy.get_name())
    rospy.spin()
