#include <ros/ros.h>
#include <diff_drive/mbot.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "my_robot");

	// Create an instance of your robot so that this instance knows about all 
	// the resources that are available.
	ros::NodeHandle nh;
	MyRobot robot=MyRobot(nh, nh);

	// Create an instance of the controller manager and pass it the robot, 
	// so that it can handle its resources.
	controller_manager::ControllerManager cm(&robot);

	// Setup a separate thread that will be used to service ROS callbacks.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Setup for the control loop.
	ros::Time prev_time = ros::Time::now();
	ros::Rate rate(10.0); // 10 Hz rate

	while (ros::ok())
	{
		// Basic bookkeeping to get the system time in order to compute the control period.
		const ros::Time     time = ros::Time::now();
		const ros::Duration period = time - prev_time;

		// Execution of the actual control loop.
		robot.read(time, period);
		// If needed, its possible to define transmissions in software by calling the 
		// transmission_interface::ActuatorToJointPositionInterface::propagate()
		// after reading the joint states.
		cm.update(time, period);
		// In case of software transmissions, use 
		// transmission_interface::JointToActuatorEffortHandle::propagate()
		// to convert from the joint space to the actuator space.
		robot.write(time, period);

		// All these steps keep getting repeated with the specified rate.
		rate.sleep();
	}
	return 0;
}