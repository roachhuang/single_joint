// this is to prevent from being included more than once.
#ifndef robot_hardware_interface_h
#define robot_hardware_interface_h

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
// #include <sensor_msgs/JointState.h>
#include <diff_drive/joint_state.h>

#include <angles/angles.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        ros::Publisher pub;
        ros::ServiceClient client;        
        rospy_tutorials::Floats joints_pub;
        diff_drive::joint_state joint_read;
        // create/catkin_make a service file in fordiff pkg to build a Float_array for joint_read 
                
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;        
        hardware_interface::EffortJointInterface effort_joint_interface_;

        //joint_limits_interface::SoftJointLimits SoftJointLimits_;
        //joint_limits_interface::JointLimits JointLimits;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        //joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
                
        int num_joints_;
        std::string joint_name_; 
		/* 
        double joint_position_;
        double joint_velocity_;
        double joint_effort_;
        double joint_position_command_;
        double joint_effort_command_;
        double joint_velocity_command_;
		*/
	
		// Data member array to store the controller commands which are sent to the 
		// robot's resources (joints, actuators)
		double cmd[2];

		// Data member arrays to store the state of the robot's resources (joints, sensors)
		double pos[2];
		double vel[2];
		double eff[2];
        
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
#endif
