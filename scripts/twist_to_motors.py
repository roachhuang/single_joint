#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack


    Copyright (C) 2012 Jon Stephan.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
# import roslib
from std_msgs.msg import Float64
# from rospy_tutorials.msg import Floats
# from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

# mapping -1, 1 (m/s) to -200, 200
from numpy import interp

#############################################################
#############################################################
class TwistToMotors(object):
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        # what func to call when ctrl-c is pressed
        rospy.on_shutdown(self.shutdown)

        self.L = rospy.get_param("wheel_separation", 0.2)
        self.R = rospy.get_param("wheel_radius", 0.035)
        # 20ms
        self.rate = rospy.get_param("rate", 50)

        # self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float64, queue_size=10)
        # pub to h/w interface. the topic name must match with yaml and launch files.
        self.pub_rmotor = rospy.Publisher(            
            '/single_joint_actuator/joint1_velocity_controller/command', Float64, queue_size=10)
        self.pub_lmotor = rospy.Publisher('/single_joint_actuator/lwheel/command', Float64, queue_size=10)

        # rospy.Subscriber('twist', Twist, self.twistCallback)
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)

        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = self.left_mapped = 0
        self.right = self.right_mapped = 0
        self.motor_speed = {}

    #############################################################
    def spin(self):
    #############################################################

        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        ###### main loop ######
        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    #############################################################
    def spinOnce(self):
    #############################################################

        # dx = (l + r) / 2
        # dr = (r - l) / w

        self.right = 1.0 * self.dx + self.dr * self.L / 2
        self.left = 1.0 * self.dx - self.dr * self.L / 2
        # rospy.loginfo("publishing: (%5.2f, %5.2f)", self.left, self.right)
        
        # mapping makes tunning pid coefficents easier
        self.right_mapped = interp(self.right, [-1, 1], [-200, 200])
        self.left_mapped  = interp(self.left, [-1, 1], [-200, 200])

        if(self.left_mapped < 0 and self.right_mapped > 0):
            self.motor_speed = [49, self.right_mapped]
        elif(self.left_mapped > 0 and self.right_mapped < 0):	  
            self.motor_speed = [self.left_mapped, 49]
        else:
            self.motor_speed = [self.left_mapped, self.right_mapped]

        self.pub_rmotor.publish(self.motor_speed[0])
        self.pub_lmotor.publish(self.motor_speed[1])

        # self.pub_lmotor.publish(self.left_mapped)
        # self.pub_rmotor.publish(self.right_mapped)
            
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
    def shutdown(self):        
        rospy.loginfo("Stop roachBot")
        self.motor_speed.data = {0}
        self.pub_motors.publish(self.motor_speed)
        # makes sure robot receive the stop cmd prior to shutting down
        self.rospy.sleep(1)

#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass





