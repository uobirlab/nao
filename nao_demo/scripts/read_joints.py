#!/usr/bin/env python

# Prints out joint angles

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
        #print self.motionProxy.getJointNames("LArm")
        #print self.motionProxy.getAngles(["LArm"], True)    
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
