#!/usr/bin/env python

import roslib

roslib.load_manifest('nao_demo')
import rospy

 
from nao_driver import * 

from std_srvs.srv import *
from nao_msgs.msg import *

import actionlib
from actionlib_msgs.msg import *

# naoqi
import motion
import almath

class TinyDancer(NaoNode):
    def __init__(self): 
        NaoNode.__init__(self)
    
        # ROS initialization:
        rospy.init_node('tiny_dancer')

        self.connectNaoQi()
        
        rospy.wait_for_service('body_stiffness/disable')
        self.stiffnessDisableClient = rospy.ServiceProxy('body_stiffness/disable', Empty)
        
        rospy.wait_for_service('body_stiffness/enable')
        self.stiffnessEnableClient = rospy.ServiceProxy('body_stiffness/enable', Empty)
        
        self.bodyPoseClient = actionlib.SimpleActionClient('body_pose', nao_msgs.msg.BodyPoseAction)
        self.bodyPoseClient.wait_for_server()
    
    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.getProxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    def toPose(self,pose_name):
        rospy.logdebug("going to pose %s",pose_name)
        goal = nao_msgs.msg.BodyPoseGoal(pose_name=pose_name)
        self.bodyPoseClient.send_goal(goal)
        self.bodyPoseClient.wait_for_result()
        return self.bodyPoseClient.get_state()
        
    def torsoBob(self, dy, dz, duration, block):
        ''' double bob, assumes everything starts zeroed and returns to that state '''
        # cartesian control, from http://www.aldebaran-robotics.com/documentation/naoqi/motion/control-cartesian.html#control-cartesian
        space      = motion.SPACE_NAO # relative to average of feet position, x is forward rhs
        axisMask   = almath.AXIS_MASK_ALL   # full control
        isAbsolute = False

        # Lower the Torso and move to the side
        effector   = "Torso"
        path       = [[0.0, -dy, -dz, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, dy, -dz, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        
        times      =  [duration * 0.25, duration * 0.5, duration * 0.75, duration]

        if block:
            self.motionProxy.positionInterpolation(effector, space, path,
                                                   axisMask, times, isAbsolute)
        else:
            self.motionProxy.post.positionInterpolation(effector, space, path,
                                                        axisMask, times, isAbsolute)
       

    def headSweep(self, duration, block):
        ''' over robot's left shoulder, down in middle '''
        names  = ["HeadYaw","HeadPitch"]
        angleLists  = [[0.7, 0.0, -0.7, 0.0], [ 0.0, 0.3, 0.0, 0.0]]
        times      =  [duration * 0.25, duration * 0.5, duration * 0.75, duration]
        timeLists   = [times]*2
        isAbsolute  = True

        if block:
            self.motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        else:
            self.motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)

    def headBob(self, diff, duration, block):
        ''' delta then back '''
        names  = ["HeadPitch"]
        angleLists  = [diff, 0.0]
        timeLists   =  [duration * 0.5, duration]
        isAbsolute  = True

        if block:
            self.motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        else:
            self.motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)


    def torsoBobDefault(self, duration):
        dy         = 0.07
        dz         = 0.03
        self.torsoBob(dy,dz,duration)
        
    def demo(self):

        rospy.loginfo("you, me, dancing")

        # start from init pose
        self.stiffnessEnableClient.call()        

        state = self.toPose('init')
        if state != GoalStatus.SUCCEEDED:
            ROS_ERROR("could not init")
            return
            
        #for i in range(10):
         #   self.torsoBob(0,0.04,4)


        # head sweep + bob, head needs work
        duration = 4
        for i in range(5):
            self.headBob(0.3,duration/2, True)
            self.headBob(-0.3,duration/2, True)
            #self.headSweep(duration, False)
            #self.torsoBob(0.04, 0.05, duration, True)

        # arm init position straight out
        # alternating arms
        
        # clap
        # head bob
        # head sweep improvements
        


        #rospy.loginfo("are we there yet?")
        #rospy.sleep(15.0)

        # return to init just to be safe
        state = self.toPose('init')

        # end in a crouch with stiffness off
        state = self.toPose('crouch')
        if state != GoalStatus.SUCCEEDED:
            ROS_ERROR("could not crouch")
        else:
            self.stiffnessDisableClient.call()

        
                


if __name__ == '__main__':

    controller = TinyDancer()
    controller.demo()
    exit(0)
