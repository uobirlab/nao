#!/usr/bin/env python


#
# ROS node to execute behaviours on the Nao that were installed by Choreographe
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib

roslib.load_manifest('nao_components')
import rospy
import sys

from nao_driver import *

#import rospy.rostime
#from rospy.rostime import Duration

import actionlib
from actionlib_msgs.msg import GoalStatus
import nao_components.msg
from nao_components.msg import BehaviorGoal, BehaviorAction

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class BehaviorManager(NaoNode):
    def __init__(self):
        NaoNode.__init__(self)

        # ROS initialization:
        rospy.init_node('behavior_manager')
        #naoqi init
        self.connectNaoQi()    


        #self.poseLibrary = dict()
        #self.readInPoses()
        #self.poseServer = actionlib.SimpleActionServer("body_pose", BodyPoseAction,
        #                                               execute_cb=self.executeBodyPose,
        #                                               auto_start=False)
        
        self.behaviorServer = actionlib.SimpleActionServer("behavior_action", BehaviorAction, 
                                                           execute_cb=self.executeBehavior,
                                                           auto_start=False) 
        self.behaviorServer.start()



    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.behaviorProxy = self.getProxy("ALBehaviorManager")
        if self.behaviorProxy is None:
            rospy.signal_shutdown("Unable to find ALBehaviorManager")
            
            
    def executeBehavior(self, goal):
        rospy.loginfo('Executing behavior: %s',goal.behavior_name)
        
        if not self.behaviorProxy.isBehaviorRunning(goal.behavior_name):
            rospy.logwarn("Behavior \"%s\" is not installed on the robot",goal.behavior_name)
            self.behaviorServer.set_aborted()
        else:
            #put execution code here
            rospy.loginfo('Done')
            self.behaviorServer.set_succeeded()


if __name__ == '__main__':

    manager = BehaviorManager()
    rospy.spin()
    exit(0)
