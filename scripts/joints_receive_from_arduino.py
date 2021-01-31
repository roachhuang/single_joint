#!/usr/bin/env python
#encoding: utf8
import rospy
from rospy_tutorials.msg import Floats

# from sensor_msgs.msg import JointState
# <packagename>.srv
from diff_drive.srv import joint_state, joint_stateResponse
# from Float_array.srv import Floats_array, Float_arrayResponse, Float_arrayRequest
pos=0
vel=0

# msg from arduino coz we subscribe /joint_states_from_arduino
def my_callback(msg):
    global pos, vel  
    pos=msg.data[0]
    vel=msg.data[1]
    #print "Pos: ", pos, "vel: ", vel
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)

# for h/w interface to call server service.
def my_server(req):
    global pos, vel
    res = joint_stateResponse() 
    res.pos=pos
    res.vel=vel
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

if __name__ == '__main__':
    main()