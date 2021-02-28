#ifndef robot_hardware_interface_h
#define robot_hardware_interface_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Int32.h>

// #include <rospy_tutorials/Floats.h>
// #include <diff_drive/joint_state.h>
#include <angles/angles.h>
#define N 20.0

class MyRobot : public hardware_interface::RobotHW
{
public:
	MyRobot(ros::NodeHandle &nh): nh_(nh) 
	{	
		ROS_INFO("Initializing roachbot Hardware Interface ...");
		// num_joints_ = joint_names_.size();
		//ROS_INFO("Number of joints: %d", (int)num_joints_);

		// Initialization of the robot's resources (joints, sensors, actuators) and
		// interfaces can be done here or inside init().
		// E.g. parse the URDF for joint names & interfaces, then initialize them
		// Create a JointStateHandle for each joint and register them with the 
		// JointStateInterface.
		hardware_interface::JointStateHandle state_handle_a("joint1", &pos[0], &vel[0], &eff[0]);
		jnt_state_interface.registerHandle(state_handle_a);
		hardware_interface::JointStateHandle state_handle_b("joint2", &pos[1], &vel[1], &eff[1]);
		jnt_state_interface.registerHandle(state_handle_b);
		// Register the JointStateInterface containing the read only joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&jnt_state_interface);

		// Create a JointHandle (read and write) for each controllable joint
		// using the read-only joint handles within the JointStateInterface and 
		// register them with the JointPositionInterface.
		hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("joint1"), &cmd[0]);
		jnt_pos_interface.registerHandle(pos_handle_a);
		hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("joint2"), &cmd[1]);
		jnt_pos_interface.registerHandle(pos_handle_b);
		// Register the JointPositionInterface containing the read/write joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&jnt_pos_interface);

		hardware_interface::JointHandle effort_handle_a(jnt_state_interface.getHandle("joint1"), &cmd[0]);
		effort_joint_interface.registerHandle(effort_handle_a);	
		hardware_interface::JointHandle effort_handle_b(jnt_state_interface.getHandle("joint2"), &cmd[1]);
		effort_joint_interface.registerHandle(effort_handle_b);
		// Register the JointEffortInterface containing the read/write joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&effort_joint_interface);

		// pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
		vl_pub = nh.advertise<std_msgs::Int32>("/vl", 5);
		vr_pub = nh.advertise<std_msgs::Int32>("/vr", 10);

		left_encoder_sub = nh.subscribe<std_msgs::Int32>("/lwheel", 1, &lwheel_cb);
		right_encoder_sub = nh.subscribe<std_msgs::Int32>("/rwheel", 1, &rwheel_cb);
		// client = nh_.serviceClient<diff_drive::joint_state>("/read_joint_state");		
		// return true;
	}

	bool init(ros::NodeHandle& root_nh)
	{
		ROS_INFO("... Done Initializing DiffBot Hardware Interface");
		return true;
	}

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
protected:
	void lwheel_cb(const std_msgs::Int32& msg) {
		encoder_ticks[0] = msg.data;
		ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg.data);
	}
	void rwheel_cb(const std_msgs::Int32& msg) {
		encoder_ticks[1] = msg.data;
		ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg.data);
	}

	double ticksToAngle(const int32_t& ticks) const
	{
		// Convert number of encoder ticks to angle in radians
		double angle = (double)ticks * (2.0 * M_PI / N);
		ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
		return angle;
	}

private:
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

	ros::Publisher vr_pub;
	ros::Publisher vl_pub;
	ros::Subscriber left_encoder_sub;
	ros::Subscriber right_encoder_sub;
	
	// ros::ServiceClient client;
	// rospy_tutorials::Floats joints_pub;
	// diff_drive::joint_state joint_read;	
};

#endif