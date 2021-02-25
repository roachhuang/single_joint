#ifndef ROACHBOT_CLASS_H
#define ROACHBOT_CLASS_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

// #include <rospy_tutorials/Floats.h>
// #include <diff_drive/joint_state.h>
#include <angles/angles.h>

// not that ros_control RobotHW needs velocity in rad/s
class MyRobot : public hardware_interface::RobotHW
{
public:
	// MyRobot(ros::NodeHandle& nh);
	MyRobot(ros::NodeHandle* nh);	
	void read(ros::Time time, ros::Duration period);
	void write(ros::Time time, ros::Duration period);

private:
	// helper functions (member methods)
	void initializeHardwareInterface();
    void initializeSubscribers();
    void initializePublishers();
	// encorder ticks
	void lwheel_cb(const std_msgs::Int32& msg_holder); 
	void rwheel_cb(const std_msgs::Int32& msg_holder); 

	// private data only available to member functions of this class
	ros::NodeHandle nh_;	// we will need this. to pass btw constructor and main
	ros::Publisher vr_pub;
	ros::Publisher vl_pub;
	ros::Subscriber lwheel_sub;
	ros::Subscriber rwheel_sub;

	// hardware_interface::JointStateInterface gives read access to all joint values 
	// without conflicting with other controllers.
	hardware_interface::JointStateInterface jnt_state_interface;
	// hardware_interface::PositionJointInterface inherits from 
	// hardware_interface::JointCommandInterface and is used for reading and writing
	// joint positions. Because this interface reserves the joints for write access,
	// conflicts with other controllers writing to the same joints might occure.
	// To only read joint positions, avoid conflicts using 
	// hardware_interface::JointStateInterface.
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::EffortJointInterface effort_joint_interface;
	// Data member array to store the controller commands which are sent to the 
	// robot's resources (joints, actuators)
	double cmd[2];

	// Data member arrays to store the state of the robot's resources (joints, sensors)
	double pos[2];
	double vel[2];
	double eff[2];
	// __int32 is synonymous with type int. The __int64 type is synonymous with type long long.
	int	encoder_ticks[2];
};	// a class definition requires a semicolon at the end of the definition.

#endif