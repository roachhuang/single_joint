#ifndef robot_hardware_interface_h
#define robot_hardware_interface_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// #include <rospy_tutorials/Floats.h>
#include <diff_drive/joint_state.h>
#include <angles/angles.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
	// must be no return type for constructor
	MyRobot(ros::NodeHandle& nh);
	void write(ros::Time time, ros::Duration period);
	void read(ros::Time time, ros::Duration period);

private:
	void lwheel_cb(const std_msgs::Int32& msg);
	void rwheel_cb(const std_msgs::Int32& msg);
	bool init(ros::NodeHandle& nh);
	float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
	double ticksToRad(const int32_t& ticks);
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
	// 0->joint1 (right); 1->joint2 (left)
	double cmd[2];

	// Data member arrays to store the state of the robot's resources (joints, sensors)
	double pos[2];
	double vel[2];
	double eff[2];
	int	encoder_ticks[2];
	const double N = 20.0;
	ros::Publisher vr_pub;
	ros::Publisher vl_pub;
	ros::Subscriber left_encoder_sub;
	ros::Subscriber right_encoder_sub;

	ros::ServiceClient client;
	// rospy_tutorials::Floats joints_pub;
	diff_drive::joint_state joint_read;
};

#endif