// #include <ros/ros.h>
// h/w interface
#include <diff_drive/mbot.h>

MyRobot::MyRobot(ros::NodeHandle &nh)
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

	init(nh);
	// client = nh_.serviceClient<diff_drive::joint_state>("/read_joint_state");		
	// return true;
}

void MyRobot::lwheel_cb(const std_msgs::Int32& msg) {
	encoder_ticks[0] = msg.data;
	//ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg.data);
}

void MyRobot::rwheel_cb(const std_msgs::Int32& msg) {
	encoder_ticks[1] = msg.data;
	// ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg.data);
}
bool MyRobot::init(ros::NodeHandle& nh)
{
	// pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
	vl_pub = nh.advertise<std_msgs::Float32>("/vl", 5);
	vr_pub = nh.advertise<std_msgs::Float32>("/vr", 10);
	// coz cb type is a class method https://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
	left_encoder_sub = nh.subscribe("/lwheel", 1, &MyRobot::lwheel_cb, this);
	right_encoder_sub = nh.subscribe("/rwheel", 1, &MyRobot::rwheel_cb, this);
	ROS_INFO("... Done Initializing DiffBot Hardware Interface");
	return true;
}
float MyRobot::mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
	return (x-in_min)*(out_max-out_min) / (in_max - in_min) + out_min;
}	

double MyRobot::ticksToRad(const int32_t& ticks)
{
	// Convert number of encoder ticks to angle in radians
	int degree;
	degree = (360 * ticks)/N;
	degree %= 360;	
	//double rad = (double)ticks * (2.0 * M_PI / N);
	// ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	return degree * M_PI/180;
}

void MyRobot::read(ros::Time time, ros::Duration period) {
	ros::Duration elapsed_time = period;
	double wheel_angles[2];
	double wheel_angle_deltas[2];
	double now_rad;

	for (std::size_t i = 0; i < 2; i++) {
		// wheel_angles[i] = ticksToRad(encoder_ticks[i]);
		//double wheel_angle_normalized = normalizeAngle(wheel_angle);
		now_rad = ticksToRad(encoder_ticks[i]);
		// wheel_angle_deltas[i] = wheel_angles[i] - pos[i];
		// pos[i]+= wheel_angle_deltas[i];		
		vel[i]= (now_rad - pos[i]) / period.toSec();
		pos[i]= now_rad;
		eff[i] = 0;
	}
	ROS_INFO("pos/vel:: %.2f, %.2f, left: %.2f, %2f",pos[0], vel[0], pos[1], vel[1]);			
}	

void MyRobot::write(ros::Time time, ros::Duration period) {
	std_msgs::Float32 vr;
	std_msgs::Float32 vl;
	const float in_min= -0.5;	// rad/s
	const float in_max= 0.5;
	const float out_min=48.0;		// pwm
	const float out_max=200.0;

	// effortJointSaturationInterface.enforceLimits(elapsed_time);    		
	
	vr.data = mapFloat(cmd[0], in_min, in_max, out_min, out_max);		

	/*left_motor.data = output_left / max_velocity_ * 100.0;
	right_motor.data = output_right / max_velocity_ * 100.0;		
	*/
	// left motor
	vl.data = mapFloat(cmd[1], in_min, in_max, out_min, out_max);

	// ROS_INFO("PWM Cmd: [%d, %d]", (int)vr.data, (int)vl.data);

	// to do: publish array of data 
	vr_pub.publish(vr);
	vl_pub.publish(vl);
	// pub.publish(joints_pub);
}		

int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "my_robot");

	// Create an instance of your robot so that this instance knows about all 
	// the resources that are available.
	ros::NodeHandle nh;
	MyRobot robot=MyRobot(nh);

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