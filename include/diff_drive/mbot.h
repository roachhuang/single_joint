#ifndef robot_hardware_interface_h
#define robot_hardware_interface_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// #include <rospy_tutorials/Floats.h>
// #include <diff_drive/joint_state.h>
#include <angles/angles.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
	// must be no return type for constructor
	MyRobot(ros::NodeHandle& nh);
	void read(ros::Time time, ros::Duration period) {
		ros::Duration elapsed_time = period;
		double wheel_angles[2];
		double wheel_angle_deltas[2];

		for (std::size_t i = 0; i < 2; i++) {
			wheel_angles[i] = ticksToAngle(encoder_ticks[i]);
			//double wheel_angle_normalized = normalizeAngle(wheel_angle);
			wheel_angle_deltas[i] = wheel_angles[i] - pos[i];
			pos[i]+= wheel_angle_deltas[i];
			vel[i]= wheel_angle_deltas[i] / period.toSec();
			eff[i] = 0;
		}			
	}

	void write(ros::Time time, ros::Duration period) {
		std_msgs::Int32 vr;
		std_msgs::Int32 vl;

		// effortJointSaturationInterface.enforceLimits(elapsed_time);    		
		ROS_INFO("PWM Cmd: [%3.2f, %3.2f]", cmd[0], cmd[1]);
		vr.data = (int)cmd[0];
		vr_pub.publish(vr);

		/*left_motor.data = output_left / max_velocity_ * 100.0;
        right_motor.data = output_right / max_velocity_ * 100.0;		
		*/
		// left motor
		vl.data = (int)cmd[1];
		vl_pub.publish(vl);
		// pub.publish(joints_pub);
	}	

	double ticksToAngle(const int32_t& ticks) const
	{
		// Convert number of encoder ticks to angle in radians
		double angle = (double)ticks * (2.0 * M_PI / N);
		ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
		return angle;
	}

private:	
	void lwheel_cb(const std_msgs::Int32& msg);
	void rwheel_cb(const std_msgs::Int32& msg);
	bool init(ros::NodeHandle &nh);
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
	int	encoder_ticks[2];
	const double N = 20.0;
	ros::Publisher vr_pub;
	ros::Publisher vl_pub;
	ros::Subscriber left_encoder_sub;
	ros::Subscriber right_encoder_sub;
	
	// ros::ServiceClient client;
	// rospy_tutorials::Floats joints_pub;
	// diff_drive::joint_state joint_read;	
};

#endif