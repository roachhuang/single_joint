#include <diff_drive/robot_hardware_interface.h>
// #include <ros_control_example/robot_hardware_interface.h>

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=5;	
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
	client = nh_.serviceClient<diff_drive::joint_state>("/read_joint_state");
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
	// Initialization of the robot's resources (joints, sensors, actuators) and
	// interfaces can be done here or inside init().
	// E.g. parse the URDF for joint names & interfaces, then initialize them
	/* Create a JointStateHandle for each joint and register them with the JointStateInterface.
	** connect and register the joint state interface
	*/
	hardware_interface::JointStateHandle state_handle_a("joint1", &pos[0], &vel[0], &eff[0]);
	jnt_state_interface_.registerHandle(state_handle_a);

	// left joint
	hardware_interface::JointStateHandle state_handle_b("joint2", &pos[1], &vel[1], &eff[1]);
	jnt_state_interface_.registerHandle(state_handle_b);

	// Register the JointStateInterface containing the read only joints
	// with this robot's hardware_interface::RobotHW.
	registerInterface(&jnt_state_interface_);

	// connect and register the joint position interface
	// Create a JointHandle (read and write) for each controllable joint
	// using the read-only joint handles within the JointStateInterface and 
	// register them with the JointEffortInterface.
	hardware_interface::JointHandle effort_handle_a(jnt_state_interface_.getHandle("joint1"), &cmd[0]);
	effort_joint_interface_.registerHandle(effort_handle_a);
	
	hardware_interface::JointHandle effort_handle_b(jnt_state_interface_.getHandle("joint2"), &cmd[1]);
	effort_joint_interface_.registerHandle(effort_handle_b);

	// Register the JointEffortInterface containing the read/write joints
	// with this robot's hardware_interface::RobotHW.
	registerInterface(&effort_joint_interface_);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

	// joint_read.request.req=1.0;
	
	if(client.call(joint_read))
	{
	    pos[0] = angles::from_degrees(joint_read.response.pos);
		pos[0] = angles::normalize_angle(joint_read.response.pos);
	    vel[0] = angles::from_degrees(joint_read.response.vel);
	    ROS_INFO("Current Pos: %.2f, Vel: %.2f",pos[0], vel[0]);
/*
if more than one joint,
        get values for joint_position_2, joint_velocity_2,......
*/	    
	    
	}
	else
	{
	    pos[2] = {0};
	    vel[2] = {0};		
	}      
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    // effortJointSaturationInterface.enforceLimits(elapsed_time);    
	joints_pub.data.clear();
	joints_pub.data.push_back(cmd[0]);
	joints_pub.data.push_back(cmd[1]);
/*
if more than one joint,
    publish values for joint_effort_command_2,......
*/	
	ROS_INFO("PWM Cmd: [%5.2f, %5.2f]", cmd[0], cmd[1]);
	pub.publish(joints_pub);	
}

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "single_joint_hardware_interface");
    ros::init(argc, argv, "roachbot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}