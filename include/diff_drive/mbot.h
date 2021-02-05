#ifndef robot_hardware_interface_h
#define robot_hardware_interface_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/ros.h>
// #include <controller_manager/controller_manager.h>

#include <rospy_tutorials/Floats.h>
// #include <sensor_msgs/JointState.h>
#include <diff_drive/joint_state.h>
#include <angles/angles.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
	MyRobot() {
		// Initialization of the robot's resources (joints, sensors, actuators) and
		// interfaces can be done here or inside init().
		// E.g. parse the URDF for joint names & interfaces, then initialize them
		/* Create a JointStateHandle for each joint and register them with the JointStateInterface.
		** connect and register the joint state interface
		*/
		hardware_interface::JointStateHandle state_handle_a("joint1", &pos[0], &vel[0], &eff[0]);
		jnt_state_interface.registerHandle(state_handle_a);
		// left joint
		hardware_interface::JointStateHandle state_handle_b("join2", &pos[1], &vel[1], &eff[1]);
		jnt_state_interface.registerHandle(state_handle_b);

		// Register the JointStateInterface containing the read only joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&jnt_state_interface);

		// connect and register the joint position interface
		// Create a JointHandle (read and write) for each controllable joint
		// using the read-only joint handles within the JointStateInterface and 
		// register them with the JointEffortInterface.
		hardware_interface::JointHandle effort_handle_a(jnt_state_interface.getHandle("joint1"), &cmd[0]);
		effort_joint_interface.registerHandle(effort_handle_a);

		hardware_interface::JointHandle effort_handle_b(jnt_state_interface.getHandle("joint2"), &cmd[1]);
		effort_joint_interface.registerHandle(effort_handle_b);

		// Register the JointEffortInterface containing the read/write joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&effort_joint_interface);

		pub = n.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
		client = n.serviceClient<diff_drive::joint_state>("/read_joint_state");
	}

	/* if not working, move init into construct and delete init func.
	bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {

		return true;
	}
	*/

	void read(const ros::Time& time, const ros::Duration& period) {
		if (client.call(joint_read))
		{
			pos[0] = angles::from_degrees(joint_read.response.pos);
			vel[0] = angles::from_degrees(joint_read.response.vel);
			ROS_INFO("Current Pos: %.2f, Vel: %.2f", joint_position_, joint_velocity_);
			/*
			if more than one joint,
					get values for joint_position_2, joint_velocity_2,......
			*/
		}
		else
		{
			pos[0] = 0;
			vel[0] = 0;
			pos[1] = 0;
			vel[1] = 0;
		}
	}

	void write(const ros::Time& time, const ros::Duration& period) {
		joints_pub.data.clear();
		joints_pub.data.push_back(cmd[0]);

		/*
		if more than one joint,
			publish values for joint_effort_command_2,......
		*/

		ROS_INFO("PWM Cmd: %.2f", cmd[0]);
		pub.publish(joints_pub);
	}

private:
	// hardware_interface::JointStateInterface gives read access to all joint values 
	// without conflicting with other controllers.
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::EffortJointInterface effort_joint_interface;

	// Data member array to store the controller commands which are sent to the 
	// robot's resources (joints, actuators)
	double cmd[2];

	// Data member arrays to store the state of the robot's resources (joints, sensors)
	double pos[2];
	double vel[2];
	double eff[2];

	// no namespace
	ros::NodeHandle n;

	ros::Publisher pub;
	ros::ServiceClient client;

	rospy_tutorials::Floats joints_pub;
	diff_drive::joint_state joint_read;
};

/* also the read method can be put here
void MyRobot::read() {

}
*/

#endif