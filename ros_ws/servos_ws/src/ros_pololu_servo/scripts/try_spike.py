#! /usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'mandeep'

#import roslib; roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from ros_pololu_servo.msg import *
#from trajectory_msgs.msg import JointTrajectory
#from trajectory_msgs.msg import JointTrajectoryPoint
#from ros_pololu_servo.msg import MotorCommand

class pololuCommander:
    def __init__(self):
        self.TimeOut=120.0
        self.pub=rospy.Publisher("pololu/command",MotorCommand,queue_size=10)
        pub_rate= 10
	#enter commands to be executed
	spikespeed=-1.0
	speed=-0.40
	steer=1.0

	spiketime=0.5
	runtime=3.0
	braketime=2.0

	totaltime=spiketime+runtime+braketime
        r = rospy.Rate(pub_rate)
	start = rospy.get_rostime()
        # Set up command for steering wheel
        mtr=MotorCommand()
        mtr.joint_name="1"
        mtr.position=steer                  # position > 0 is left, < 0 is right. should be between -0.590088 and 0.806176 degrees.
        mtr.speed=0.0                     #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
        mtr.acceleration=0.0

	while not rospy.is_shutdown():
            curtime = (rospy.get_rostime()-start).to_sec()
            if curtime >totaltime:
		# Command for steering wheel
                mtr.joint_name="1"
                mtr.position=0.0    # position > 0 is left, < 0 is right. In radians.
                mtr.speed=0.0         #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                mtr.acceleration=0.0
		self.pub.publish(mtr)
                r.sleep()

		# Command for traction motor
                mtr.joint_name="0"
                mtr.position=0.0      # position > 0 is backward, < 0 is forward. 
                mtr.speed=0.0         #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                mtr.acceleration=0.0
		self.pub.publish(mtr)
                rospy.signal_shutdown("Done")
		break
            
            try:
                rospy.loginfo("building commands for steering_wheel.")
                if curtime < totaltime:
                    # Command for steering wheel
                    mtr.joint_name="1"
                    mtr.position=steer   # position > 0 is left, < 0 is right. In radians.
                    mtr.speed=0.0         #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                    mtr.acceleration=0.0
            except IndexError:
                rospy.signal_shutdown("Done")
                break
            self.pub.publish(mtr)
            rospy.loginfo("steering_wheel commands published.")
            r.sleep()

            try:
                rospy.loginfo("building commands for traction_motor.")
                if curtime < spiketime:
                    # Command for traction motor
                    mtr.joint_name="0"
                    mtr.position=spikespeed   # position > 0 is backward, < 0 is forward. 
                    mtr.speed=0.0        #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                    mtr.acceleration=0.0          
                elif curtime < spiketime+runtime:
                    # Command for traction motor
                    mtr.joint_name="0"
                    mtr.position=speed   # position > 0 is backward, < 0 is forward. 
                    mtr.speed=0.0        #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                    mtr.acceleration=0.0
		elif curtime < totaltime:
                    # Command for traction motor
                    mtr.joint_name="0"
                    mtr.position=3.0   # position > 0 is backward, < 0 is forward. 
                    mtr.speed=0.0       #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                    mtr.acceleration=0.0
                else:
                    # Command for traction motor
                    mtr.joint_name="0"
                    mtr.position=0.0      # position > 0 is backward, < 0 is forward. 
                    mtr.speed=0.0         #/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
                    mtr.acceleration=0.0
            except IndexError:
                rospy.signal_shutdown("Done")
                break
            self.pub.publish(mtr)
            rospy.loginfo("traction_motor commands published.")




            print 'Now time =', curtime ,'\n'
            r.sleep()


if __name__ == '__main__':
  rospy.init_node('pololu_action_server')
  server = pololuCommander()
  rospy.spin()
