#!/usr/bin/env python
# import roslib; roslib.load_manifest('YOUR_PACKAGE_NAME_HERE')
import rospy
from std_msgs.msg import    Float32
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

class Kinematics(object):
    def __init__(self):
        self.L = rospy.get_param("wheel_separation", 0.2)   
        self.R = rospy.get_param("wheel_radius", 0.035)
        self.rate = rospy.get_param("rate", 10)
        self.efforts = [0,0]
        rospy.init_node("twist_to_motors", anonymous=False)
        # send vl and vr to ros controller as setpoint command
        self.pub_efforts = rospy.Publisher('/single_joint_actuator/joint1_velocity_controller', Float32, queue_size=1)      
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)
        #self.nodename = rospy.get_name()
        #rospy.loginfo("%s started" % self.nodename)
        # 10hz = 0.1s
        self.rate = rospy.Rate(self.rate)     
        self.last_time = rospy.Time.now()
        self.using_cmd_vel = False    

    def cmd_vel_cb(self, msg):
        #rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x,
        #msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x,
        #msg.angular.y, msg.angular.z))
        self.using_cmd_vel = True
        self.last_time = rospy.Time.now()

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities
        # into motor commands
        self.v = msg.linear.x
        # the robot car doesn't move in y direction
        # y = msg.linear.y
        self.w = msg.angular.z
        # vr
        self.efforts[0] = (2.0 * self.v + self.w * self.L) / (2.0 * self.R)
        # vl
        self.efforts[1] = (2.0 * self.v - self.w * self.L) / (2.0 * self.R)   

    def main(self):
        #self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        #self.left = 0
        #self.right = 0
            
        while not rospy.is_shutdown():
            if self.using_cmd_vel and rospy.Time.now().to_sec() - self.last_time.to_sec() >= 1.0:
                # stop motor if no cmd_vel in 1sec
                self.efforts = [0,0]    
                self.using_cmd_vel = False
            self.pub_efforts.publish(self.efforts[0])            
            self.rate.sleep()

if __name__ == '__main__':    
    #try:
        effort_cmd = Kinematics()
        effort_cmd.main()
    #except rospy.ROSInterruptException:
    #    pass


