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
		joint_state_interface_.registerHandle(state_handle_a);

		// left joint
		hardware_interface::JointStateHandle state_handle_b("joint2", &pos[1], &vel[1], &eff[1]);
		joint_state_interface_.registerHandle(state_handle_b);

		// Register the JointStateInterface containing the read only joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&joint_state_interface_);

		// connect and register the joint position interface
		// Create a JointHandle (read and write) for each controllable joint
		// using the read-only joint handles within the JointStateInterface and 
		// register them with the JointEffortInterface.
		hardware_interface::JointHandle effort_handle_a(joint_state_interface_.getHandle("joint1"), &cmd[0]);
        effort_joint_interface_.registerHandle(effort_handle_a);

		hardware_interface::JointHandle effort_handle_b(joint_state_interface_.getHandle("joint2"), &cmd[1]);
		effort_joint_interface_.registerHandle(effort_handle_b);

		// Register the JointEffortInterface containing the read/write joints
		// with this robot's hardware_interface::RobotHW.
		registerInterface(&effort_joint_interface_);

		//pub = n.advertise<rospy_tutorials::Floats>("/joints_to_aurdino", 10);
		//client = n.serviceClient<diff_drive::joint_state>("/read_joint_state");
}

/*
void ROBOTHardwareInterface::init() {
    
    
	joint_name_="joint1";
    
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_name_, &pos[0], &vel[0], &eff[0]);
    joint_state_interface_.registerHandle(jointStateHandle);

// Create position joint interface
    //hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_);
    //position_joint_interface_.registerHandle(jointPositionHandle);
    
// Create velocity joint interface
	//hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    //effort_joint_interface_.registerHandle(jointVelocityHandle);
    
// Create effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &cmd[0]);
	effort_joint_interface_.registerHandle(jointEffortHandle);
	
// Create Joint Limit interface   
    // joint_limits_interface::JointLimits limits;
    // joint_limits_interface::getJointLimits("joint1", nh_, limits);
    
	// joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
	// effortJointSaturationInterface.registerHandle(jointLimitsHandle);
	
-----------
If you have more joints then,
    joint_name_= "joint2"
    
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle2(joint_name_, &joint_position_2, &joint_velocity_2, &joint_effort_2);
    joint_state_interface_.registerHandle(jointStateHandle);
//create the position/velocity/effort interface according to your actuator 
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_2);
    position_joint_interface_.registerHandle(jointPositionHandle2);
    
    hardware_interface::JointHandle jointVelocityHandle2(jointStateHandle2, &joint_velocity_command_2);
    effort_joint_interface_.registerHandle(jointVelocityHandle2);
    
    hardware_interface::JointHandle jointEffortHandle2(jointStateHandle2, &joint_effort_command_2);
	effort_joint_interface_.registerHandle(jointEffortHandle2);
	
//create joint limit interface.
    joint_limits_interface::getJointLimits("joint2", nh_, limits);
	joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle2(jointEffortHandle2, limits);
	effortJointSaturationInterface.registerHandle(jointLimitsHandle2);
	
	Repeat same for other joints
----------------
	

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    // registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    // registerInterface(&effortJointSaturationInterface);
}
*/

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
	    vel[0] = angles::from_degrees(joint_read.response.vel);
	    ROS_INFO("Current Pos: %.2f, Vel: %.2f",pos[0], vel[0]);
/*
if more than one joint,
        get values for joint_position_2, joint_velocity_2,......
*/	    
	    
	}
	else
	{
	    pos[0] = 0;
	    vel[0] = 0;
	}
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    // effortJointSaturationInterface.enforceLimits(elapsed_time);    
	joints_pub.data.clear();
	joints_pub.data.push_back(cmd[0]);
	
/*
if more than one joint,
    publish values for joint_effort_command_2,......
*/	
	
	ROS_INFO("PWM Cmd: %.2f", cmd[0]);
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