#!/usr/bin/env python
# encoding: utf8
import rospy
from sensor_msgs.msg import JointState
# from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32, Float64
# <packagename>.srv
from diff_drive.srv import joint_state, joint_stateResponse
# from Float_array.srv import Floats_array, Float_arrayResponse,
# Float_arrayRequest

class Wheel_state(object):
    def __init__(self):
        rospy.init_node("wheel_states")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.pos0 = self.pos1 = 0
        self.vel = [0,0]
        self.rate = rospy.get_param("~rate", 5)

        # sub from pid_velocity.py and create a service for h/w interface to call
        # note that joint1 maps to rwheel, cmd[0], vel[0]
        rospy.Subscriber("/rwheel_vel", Float32, self.joint1_callback)
        rospy.Subscriber("/lwheel_vel", Float32, self.joint2_callback)
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
          

    def joint1_callback(self, msg):
        # self.pos[0]=msg.data[0]
        # self.vel[0]=msg.data[1]
        # self.pos0=msg.data[0]
        self.vel[0] = msg.data

    def joint2_callback(self, msg):
        # self.pos[0]=msg.data[0]
        # self.vel[0]=msg.data[1]
        # self.pos1=msg.data[0]
        self.vel[1] = msg.data

        # print "Pos: ", pos, "vel: ", vel
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", msg)
    """
    def joint2_callback(self, msg):
        self.pos[1]=msg.data[0]
        self.vel[1]=msg.data[1]
    """

    # for h/w interface to call server service.
    def my_server(self, req):
        # global pos, vel
        res = joint_stateResponse()
        res.pos1 = self.pos0
        res.vel1 = self.vel[0]
        res.pos2 = self.pos1
        res.vel2 = self.vel[1]
        res.success = True
        return res

    def PubJointState(self):
        rate = rospy.Rate(5)   # 5hz

        msg = JointState()
        msg.header.frame_id = 'base_link'
        msg.name = ['joint1', 'joint2']
        while not rospy.is_shutdown():
            msg.velocity = self.vel
            msg.position = []
            msg.effort = []
            rospy.loginfo("pub!")
            self.pub_joint_state.publish(msg)
            rate.sleep()
            rospy.spin()
     #############################################################

    def spin(self):
    #############################################################
        r = rospy.Rate(self.rate)
        #idle = rospy.Rate(5)
        # then = rospy.Time.now()
        ###### main loop  ######
        while not rospy.is_shutdown():           
            self.spinOnce()
            r.sleep()
        #idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################        
        msg = JointState()   
        msg.header.frame_id = 'base_link'              
        msg.name.append('joint1')   # right wheel
        msg.name.append('joint2')       
        msg.velocity.append(self.vel[0])
        msg.velocity.append(self.vel[1])
        msg.position = [0,0]
        #msg.effort = []  
        msg.header.stamp = rospy.Time.now()
        self.pub_joint_state.publish(msg)    

    def main(self): 
        # joint_state is the file name of srv; client is read method in h/w
        # interface
        rospy.Service('read_joint_state', joint_state, self.my_server)
        rospy.loginfo('joint state service is available for h/w interface')
        # spin() simply keeps python from exiting until this node is stopped 
        rospy.spin()        
"""
pos=[0,0]
vel=[0,0]

# msg from arduino coz we subscribe /joint_states_from_arduino
def my_callback(msg):
    global pos, vel  
    pos[0]=msg.data[0]
    vel[0]=msg.data[1]

    # pos[1]=msg[1].data[0]
    # vel[1]=msg[1].data[1]
    # print "Pos: ", pos, "vel: ", vel
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

# for h/w interface to call server service.
def my_server(req):
    global pos, vel
    res = joint_stateResponse() 
    res.pos1=pos[0]
    res.vel1=vel[0]
    res.pos2=pos[1]
    res.vel2=vel[1]
    res.success = True
    return res

def main():
    rospy.init_node('subscriber_py', anonymous=True) #initialzing the node with name "subscriber_py"

    rospy.Subscriber("/joint_states_from_arduino", Floats, my_callback, queue_size=10)
 
    # joint_state is the file name of srv; client is read method in h/w interface
    rospy.Service('read_joint_state', joint_state, my_server)
    rospy.loginfo('joint state service is available for h/w interface')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() 
"""
if __name__ == '__main__':
    wheel_state = Wheel_state()
    # wheel_state.main()   
    wheel_state.spin()
