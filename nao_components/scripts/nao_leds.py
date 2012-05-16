#!/usr/bin/env python

#
# ROS node to control Nao's LEDs
# This code is based on nao_walker.py although all errors are my own.
# 
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
roslib.load_manifest('nao_driver')
import rospy

from nao_driver import *


class NaoLEDs(NaoNode):

    def __init__(self): 
	NaoNode.__init__(self)
    
        # ROS initialization:
        rospy.init_node('nao_LEDs')
        
        self.connectNaoQi()
    
        # last: ROS subscriptions (after all vars are initialized)
        #rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)
        
        # ROS services (blocking functions)
        #self.cmdPoseSrv = rospy.Service("cmd_pose_srv", CmdPoseService, self.handleTargetPoseService)
    
        rospy.loginfo("nao_LEDs initialized")
    
    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.ledsProxy = self.getProxy("ALLeds")
        if self.ledsProxy is None:
            exit(1)
                            
    # def handleCmdVel(self, data):
    #     rospy.logdebug("Walk cmd_vel: %f %f %f, frequency %f", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
    #     if data.linear.x != 0 or data.linear.y != 0 or data.angular.z != 0:
    #         self.gotoStartWalkPose()        
    #     try:        
    #         eps = 1e-3 # maybe 0,0,0 is a special command in motionProxy...
    #         if abs(data.linear.x)<eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
    #            self.motionProxy.setWalkTargetVelocity(0,0,0,0.5)
    #         self.motionProxy.setWalkTargetVelocity(data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
    #     except RuntimeError,e:
    #         # We have to assume there's no NaoQI running anymore => exit!
    #         rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
    #         rospy.signal_shutdown("No NaoQI available anymore")
            
            

    # def handleCmdVelService(self, req):
    #     self.handleCmdVel(req.twist)
    #     return CmdVelServiceResponse()
        


if __name__ == '__main__':
    LEDs = NaoLEDs()
    rospy.loginfo("nao_LEDs running...")
    rospy.spin()
    rospy.loginfo("nao_LEDs stopping...")
    # any shutdown stuff here
    rospy.loginfo("nao_LEDs stopped.")
    exit(0)
