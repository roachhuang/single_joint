#include <diff_drive/mbot_class.h>

MyRobot::MyRobot(ros::NodeHandle* nh):nh_(*nh)
{
    ROS_INFO("in class constructor of MyRobot class");
    initializeHardwareInterface();
    initializeSubscribers();
    initializePublishers();

    // initialize varables
}

void MyRobot::initializeHardwareInterface(){
    // Initialization of the robot's resources (joints, sensors, actuators) and
    // interfaces can be done here or inside init().
    // E.g. parse the URDF for joint names & interfaces, then initialize them
    // Create a JointStateHandle for each joint and register them with the 
    // JointStateInterface.
    // num_joints_ = joint_names_.size();
    // ROS_INFO("Number of joints: %d", (int)num_joints_);
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
}

// member helper function to set up subscribers
void MyRobot::initializeSubscribers(){     
	lwheel_sub = nh_.subscribe("lwheel", 1, &MyRobot::lwheel_cb, this);
	rwheel_sub = nh_.subscribe("rwheel", 1, &MyRobot::rwheel_cb, this);

}
void MyRobot::initializePublishers(){
    // pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
	vl_pub = nh_.advertise<std_msgs::Int32>("/vl", 10);
	vr_pub = nh_.advertise<std_msgs::Int32>("/vr", 10);
}
double ticksToAngle(const int &ticks)
{
	// Convert number of encoder ticks to angle in radians
	double angle = (double)ticks * (2.0 * M_PI / 542.0);
	ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	return angle;
}

void MyRobot::lwheel_cb(const std_msgs::Int32& msg) {
	encoder_ticks[1] = msg.data;
	// ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg->data);
}
void MyRobot::rwheel_cb(const std_msgs::Int32& msg) {
	encoder_ticks[0] = msg.data;
	// ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg->data);
}

void MyRobot::read(ros::Time time, ros::Duration period) {
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
void MyRobot::write(ros::Time time, ros::Duration period) {
    std_msgs::Int32 vr;
    std_msgs::Int32 vl;

    // effortJointSaturationInterface.enforceLimits(elapsed_time);    		
    ROS_INFO("PWM Cmd: [%5.2f, %5.2f]", cmd[0], cmd[1]);
    vr.data = cmd[0];
    vr_pub.publish(vr);

    /*left_motor.data = output_left / max_velocity_ * 100.0;
    right_motor.data = output_right / max_velocity_ * 100.0;		
    */
    // left motor
    vl.data = cmd[1];
    vl_pub.publish(vl);
    // pub.publish(joints_pub);
}

int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "MyRobotClass");  // node name

	// Create an instance of your robot so that this instance knows about all 
	// the resources that are available. need to pass this to the class constructor
	ros::NodeHandle nh;

    ROS_INFO("main: instantiating an object of type MyRobotClass");
	MyRobot roachbot=MyRobot(&nh);

	// Create an instance of the controller manager and pass it the robot, 
	// so that it can handle its resources.
	controller_manager::ControllerManager cm(&roachbot);

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
		roachbot.read(time, period);
		// If needed, its possible to define transmissions in software by calling the 
		// transmission_interface::ActuatorToJointPositionInterface::propagate()
		// after reading the joint states.
		cm.update(time, period);
		// In case of software transmissions, use 
		// transmission_interface::JointToActuatorEffortHandle::propagate()
		// to convert from the joint space to the actuator space.
		roachbot.write(time, period);

		// All these steps keep getting repeated with the specified rate.
		rate.sleep();
	}
	return 0;
}
