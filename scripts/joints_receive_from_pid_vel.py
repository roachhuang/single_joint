#!/usr/bin/env python
#encoding: utf8
import rospy
#from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
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
        self.vel0 = self.vel1 = 0       
        
        # sub from pid_velocity.py and create a service for h/w interface to
        # call
        rospy.Subscriber("/rwheel_vel", Float32, self.joint1_callback, queue_size=10)
        rospy.Subscriber("/lwheel_vel", Float32, self.joint2_callback, queue_size=10)

    def joint1_callback(self, msg):        
        #self.pos[0]=msg.data[0]
        #self.vel[0]=msg.data[1]
        #self.pos0=msg.data[0]
        self.vel0 = msg.data

    def joint2_callback(self, msg):        
        #self.pos[0]=msg.data[0]
        #self.vel[0]=msg.data[1]
        #self.pos1=msg.data[0]
        self.vel1 = msg.data
       
        #print "Pos: ", pos, "vel: ", vel
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
    """             
    def joint2_callback(self, msg):        
        self.pos[1]=msg.data[0]
        self.vel[1]=msg.data[1]
    """

    # for h/w interface to call server service.
    def my_server(self, req):
        global pos, vel
        res = joint_stateResponse() 
        res.pos1 = self.pos0
        res.vel1 = self.vel0
        res.pos2 = self.pos1
        res.vel2 = self.vel1
        res.success = True
        return res

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
    #print "Pos: ", pos, "vel: ", vel
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

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
    wheel_state.main()