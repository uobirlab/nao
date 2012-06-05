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
roslib.load_manifest('nao_components')
import rospy

from nao_driver import *

from nao_components.msg import LEDs

class NaoLEDs(NaoNode):


    def __init__(self): 
	NaoNode.__init__(self)

        # set up to match constants in LEDs.msg
        self.ledGroups = ["EarLeds","FaceLeds","BrainLeds","ChestLeds","FeetLeds","AllLeds"]
    
        # ROS initialization:
        rospy.init_node('nao_LEDs')        
        self.connectNaoQi()    
        rospy.Subscriber("leds", LEDs, self.handleLEDState, queue_size=10)        
        rospy.loginfo("nao_LEDs initialized")
    
    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.ledsProxy = self.getProxy("ALLeds")
        if self.ledsProxy is None:
            exit(1)

        # print(self.ledsProxy.listGroups())
        # print(self.ledsProxy. getMethodList ())
         
    def validRGBValue(self,val):
        return val >= 0 and val <= 255
                   
    def handleLEDState(self, data):
        #rospy.loginfo("handleLEDState led: %d, rgb: %d %d %d", data.led, data.red, data.green, data.blue);

        #check for valid values
        if data.led < 0 or data.led >= len(self.ledGroups):
            rospy.logwarn("Out of range LED constant (0 - %d), ignoring: %d", (len(self.ledGroups) - 1), data.led)
            return

        if not self.validRGBValue(data.red):
            rospy.logwarn("out of range (0 - 255) red value, ignoring: %d", data.red)
            return

        if not self.validRGBValue(data.green):
            rospy.logwarn("out of range (0 - 255) green value, ignoring: %d", data.green)
            return

        if not self.validRGBValue(data.blue):
            rospy.logwarn("out of range (0 - 255) blue value, ignoring: %d", data.blue)
            return

        try:                    

            if data.led == data.ledBothEyes or data.led == data.ledChest:

                #  RGB to hex: 0x00RRGGBB.
                rgb = data.blue
                rgb = rgb | (data.green << 8)
                rgb = rgb | (data.red << 16)
            
                self.ledsProxy.fadeRGB(self.ledGroups[data.led], rgb, data.duration.to_sec())

            elif data.led == data.ledBothEars or data.led == data.ledHead:

                intensity = data.blue / 255.0
                
                self.ledsProxy.fade(self.ledGroups[data.led], intensity, data.duration.to_sec())

        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")


if __name__ == '__main__':
    LEDs = NaoLEDs()
    rospy.loginfo("nao_LEDs running...")
    rospy.spin()
    rospy.loginfo("nao_LEDs stopping...")
    # any shutdown stuff here
    rospy.loginfo("nao_LEDs stopped.")
    exit(0)
