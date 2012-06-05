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


    lArm = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll']
    rArm = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']

    bothArms = lArm + rArm

    # "clap"
    lArmIn   = [0.17,  -0.65,  0.30, -0.6]
    rArmIn   = [0.17,   0.65, -0.30,  0.6]
    
    # straight out in front
    lArmOut  = [0.11, -0.08,  0.30, -0.08]
    rArmOut  = [0.11,  0.08, -0.30,  0.08]

    # down
    lArmDown = [0.6,  0.1,  0.1, -0.6]
    rArmDown = [0.6, -0.1, -0.1,  0.6]

    # up
    lArmUp   = [-0.6,  0.1,  0.1, -0.6]
    rArmUp   = [-0.6, -0.1, -0.1,  0.6]


    head = ["HeadYaw", "HeadPitch"]

    headLeft         = [ 0.7,  0.0]
    headMiddleUp     = [ 0.0, -0.3]
    headMiddleMiddle = [ 0.0,  0.0]
    headMiddleDown   = [ 0.0,  0.3]
    headRight        = [-0.7,  0.0]


    lowerHalf = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 
                 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']

    
    torsoInit      = [ 0.0,  0.0, -0.4,  0.7, -0.4,  0.0, 
                       0.0,  0.0, -0.4,  0.7, -0.4,  0.0]

    torsoBobMiddle = [ 0.0,  0.0, -0.7,  1.3, -0.6,  0.0, 
                       0.0,  0.0, -0.7,  1.3, -0.6,  0.0]

    torsoBobLeft   = [ 0.0, -0.3, -0.7,  1.2, -0.6,  0.3, 
                       0.0, -0.3, -0.7,  1.2, -0.6,  0.3]

    torsoBobRight  = [ 0.0,  0.3, -0.7,  1.2, -0.6, -0.3, 
                       0.0,  0.3, -0.7,  1.2, -0.6, -0.3] 

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
        
    def torsoBob(self, dy, dz, duration, block=True):
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
       

    def headSweep(self, duration, block=True):
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

    def headBob(self, diff, duration, block=True):
        ''' delta then back '''
        names  = ["HeadPitch"]
        angleLists  = [diff, 0.0]
        timeLists   =  [duration * 0.5, duration]
        isAbsolute  = True

        if block:
            self.motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        else:
            self.motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)



    def armsOut(self, duration, block=True):
        names = self.lArm + self.rArm
        jointPositions = [self.lArmOut + self.rArmOut] 
        self.multiMove(names,jointPositions,duration,block)

    def armsDown(self, duration, block=True):
        names = self.lArm + self.rArm
        jointPositions = [self.lArmDown + self.rArmDown] 
        self.multiMove(names,jointPositions,duration,block)

    def armsUp(self, duration, block=True):
        names = self.lArm + self.rArm
        jointPositions = [self.lArmUp + self.rArmUp] 
        self.multiMove(names,jointPositions,duration,block)



    def interleave(self, l1):
        return [ [ l1[j][i] for j in range(len(l1)) ] for i in range(len(l1[0])) ]


    def clap(self, duration, block=True):
        names = self.lArm + self.rArm
        jointPositions = [self.lArmIn + self.rArmIn, self.lArmOut + self.rArmOut] 
        self.multiMove(names,jointPositions,duration,block)


    def multiMoveLoop(self, names, jointPositionsOnce, singleDuration, times, block=True):
        jointPositions = jointPositionsOnce*times
        self.multiMove(names,jointPositions,times * singleDuration,block)


    def clapLoop(self, times, singleDuration, block=True):
        names = self.lArm + self.rArm
        jointPositionsOnce = [self.lArmIn + self.rArmIn, self.lArmOut + self.rArmOut] 
        self.multiMoveLoop(names,jointPositionsOnce,singleDuration,times, block)



    def multiMove(self, namesList, jointPositions, duration, block=True):
        ''' move though given joint positions at regular intervals '''
        angleLists = self.interleave(jointPositions)
        timeLists   = [[((i+1.0)*(duration/len(jointPositions))) for i in range(len(jointPositions))]] * len(namesList)
        isAbsolute  = True

        if block:
            self.motionProxy.angleInterpolation(namesList, angleLists, timeLists, isAbsolute)
        else:
            self.motionProxy.post.angleInterpolation(namesList, angleLists, timeLists, isAbsolute)


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

        # 2 seconds a bar
        barLength = 2.0

        self.armsOut(barLength,False)

        # start by bobbing up and down for 4 bars

        # a single bob is a bar
        singleMiddleBob = [self.torsoBobMiddle, 
                           self.torsoInit]

        self.multiMoveLoop(self.lowerHalf, singleMiddleBob, barLength, 4)


        #add a head bob at half that speed

        # head is slower, so do it over 2 bars
        singleMiddleBobWithHead = [self.headMiddleMiddle + self.torsoBobMiddle, 
                                   self.headLeft         + self.torsoInit,
                                   self.headMiddleMiddle + self.torsoBobMiddle, 
                                   self.headRight        + self.torsoInit]

        headAndBody             = self.head + self.lowerHalf

        self.multiMoveLoop(headAndBody, singleMiddleBobWithHead, barLength * 2, 2)


        # one bob
        self.multiMoveLoop(self.lowerHalf, singleMiddleBob, barLength, 1)


        # bob left arm air
        singleMiddleBobLeftArmUp = [self.lArmUp + self.torsoBobMiddle, 
                                    self.lArmUp + self.torsoInit]
        leftArmAndBody           = self.lArm + self.lowerHalf
        self.multiMoveLoop(leftArmAndBody, singleMiddleBobLeftArmUp, barLength, 1)

        # one bob
        self.multiMoveLoop(self.lowerHalf, singleMiddleBob, barLength, 1)


        # bob rightarmDone
        singleMiddleBobRightArmDown = [self.rArmDown + self.torsoBobMiddle, 
                                       self.rArmDown + self.torsoInit]
        rightArmAndBody             = self.rArm + self.lowerHalf
        self.multiMoveLoop(rightArmAndBody, singleMiddleBobRightArmDown, barLength , 1)


        # bob with head again
        self.multiMoveLoop(headAndBody, singleMiddleBobWithHead, barLength * 2, 2)


        # now hand jive
        bobWithHandJive = [self.lArmDown + self.rArmUp   + self.torsoBobMiddle, 
                           self.lArmDown + self.rArmUp   + self.torsoInit,
                           self.lArmUp   + self.rArmDown + self.torsoBobMiddle, 
                           self.lArmUp   + self.rArmDown + self.torsoInit]
        
        armsAndBody     = self.lArm + self.rArm + self.lowerHalf

        self.multiMoveLoop(armsAndBody, bobWithHandJive, barLength * 2, 2)
        

        #stand and clap
        singleClap = [self.lArmIn + self.rArmIn,
                      self.lArmOut + self.rArmOut]
                      
        self.multiMoveLoop(self.bothArms, singleClap, barLength, 4)


        # bob clap jive
        bobWithHandJiveClap = [self.lArmIn   + self.rArmIn   + self.torsoBobMiddle, 
                               self.lArmDown + self.rArmUp   + self.torsoInit,
                               self.lArmIn   + self.rArmIn   + self.torsoBobMiddle, 
                               self.lArmUp   + self.rArmDown + self.torsoInit]
        
        self.multiMoveLoop(armsAndBody, bobWithHandJiveClap, barLength * 2, 4)



    def bobDemo(self):


        
        duration = 2.0
        times = 4
    
        jointPositionsOnce = [self.torsoBobMiddle, 
                              self.torsoInit]

        self.multiMoveLoop(self.lowerHalf, jointPositionsOnce, duration, times)


        duration = 6.0

        jointPositionsOnce = [self.torsoBobMiddle, 
                              self.torsoInit,
                              self.torsoBobLeft, 
                              self.torsoInit,
                              self.torsoBobRight, 
                              self.torsoInit]

        self.multiMoveLoop(self.lowerHalf, jointPositionsOnce, duration, times)



    def armsAndHeadDemo(self):

        duration = 2.0
        times = 2

        self.armsOut(duration, True)
        #self.clapLoop(4,duration,True)

        
        # 4 times with arms
        jointPositionsOnce = [self.lArmUp + self.rArmDown, 
                              self.lArmDown + self.rArmUp] 
        self.multiMoveLoop(self.bothArms, jointPositionsOnce, duration, times)

        # add head
        headAndArms = self.head + self.bothArms
        jointPositionsOnce = [self.headLeft  + self.lArmUp   + self.rArmDown, 
                              self.headLeft  + self.lArmDown + self.rArmUp, 
                              self.headRight + self.lArmUp   + self.rArmDown, 
                              self.headRight + self.lArmDown + self.rArmUp] 
        self.multiMoveLoop(headAndArms, jointPositionsOnce, duration*2, times)



        #self.armsDown(duration, True)
        #self.armsUp(duration, True)

        #for i in range(5):
        #    self.headBob(0.3,duration/2, True)
        #    self.headBob(-0.3,duration/2, True)
        #for i in range(5):
            #self.headSweep(duration, False)
            #self.torsoBob(0.04, 0.05, duration, True)

        # arm init position straight out
        # alternating arms
        
        # clap
        # head sweep improvements
        


        #rospy.loginfo("are we there yet?")
        #rospy.sleep(15.0)

        # return to init just to be safe
  #      state = self.toPose('init')

        # end in a crouch with stiffness off
        #state = self.toPose('crouch')
        #if state != GoalStatus.SUCCEEDED:
        #    ROS_ERROR("could not crouch")
        #else:
        #    self.stiffnessDisableClient.call()

        
                


if __name__ == '__main__':

    controller = TinyDancer()
    controller.demo()
    exit(0)
