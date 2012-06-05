#!/usr/bin/env python

# SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/nao_robot/nao_driver/scripts/nao_controller.py $
# SVN $Id: nao_controller.py 2245 2011-11-28 16:25:56Z hornunga@informatik.uni-freiburg.de $


#
# ROS node to provide joint angle control to Nao by wrapping NaoQI
# This code is currently compatible to NaoQI version 1.6 or newer (latest 
# tested: 1.10)
#
# Copyright 2011 Armin Hornung, University of Freiburg
# http://www.ros.org/wiki/nao
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

roslib.load_manifest('nao_demo')
import rospy

 
from nao_driver import * 

import math
from math import fabs

class JointReader(NaoNode):
    def __init__(self): 
        NaoNode.__init__(self)
    
        # ROS initialization:
        rospy.init_node('joint_reader')

        self.pip="192.168.0.6"

        self.connectNaoQi()
        
        # get number of available joints, used to distinguish between H21 and H25 later
        #self.availableJoints = len(self.motionProxy.getJointNames("Body"))


        #print self.motionProxy.getAngles("Body", True)
        print self.motionProxy.getJointNames("LArm")
        print self.motionProxy.getAngles(["LArm"], True)    
        print self.motionProxy.getJointNames("RArm")
        print self.motionProxy.getAngles(["RArm"], True)    
        #print self.motionProxy.getAngles(["LHipPitch","LKneePitch","LAnklePitch"], False)    



    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.getProxy("ALMotion")
        if self.motionProxy is None:
            exit(1)
            
            


if __name__ == '__main__':

    controller = JointReader()
    exit(0)
