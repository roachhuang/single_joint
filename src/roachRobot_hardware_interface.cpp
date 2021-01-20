#include <ros/ros.h>
#include <std_msgs/Float32.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

const unsigned int NUM_JOINTS = 2;

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    hardware_interface::JointHandle vel_handle_a(vel_joint_interface.getHandle("A"), &vel[0]);
    jnt_pos_interface.registerHandle(vel_handle_a);
    hardware_interface::JointHandle vel_handle_b(vel_joint_interface.getHandle("B"), &vel[1]);
    jnt_pos_interface.registerHandle(vel_handle_b);

    registerInterface(&jnt_pos_interface);
  }
  void write()
  {
    double diff_ang_speed_left = cmd[0];
    double diff_ang_speed_right = cmd[1];
    limitDifferentialSpeed(diff_ang_speed_left, diff_ang_speed_right);
    // Publish results
    std_msgs::Float32 left_wheel_vel_msg;
    std_msgs::Float32 right_wheel_vel_msg;
    left_wheel_vel_msg.data = diff_ang_speed_left;
    right_wheel_vel_msg.data = diff_ang_speed_right;
    left_wheel_vel_pub_.publish(left_wheel_vel_msg);
    right_wheel_vel_pub_.publish(right_wheel_vel_msg);
  }

  /**
   * Reading encoder values and setting position and velocity of enconders 
   */
  void read(const ros::Duration &period)
  {
    double ang_distance_left = _wheel_angle[0];
    double ang_distance_right = _wheel_angle[1];
    pos[0] += ang_distance_left;
    vel[0] += ang_distance_left / period.toSec();
    pos[1] += ang_distance_right;
    vel[1] += ang_distance_right / period.toSec();
    /*
    std::ostringstream os;
    for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    {
      os << cmd[i] << ", ";
      pos[i] = cmd[i];
    }
    os << cmd[NUM_JOINTS - 1];
    ROS_INFO_STREAM("Commands for joints: " << os.str());
    */
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface vel_joint_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  ros::Subscriber left_wheel_angle_sub_;
  ros::Subscriber right_wheel_angle_sub_;
  ros::Publisher left_wheel_vel_pub_;
  ros::Publisher right_wheel_vel_pub_;

   void leftWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[0] = msg.data;
  }

  void rightWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[1] = msg.data;
  }
};

main()
{
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  while (true)
  {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     sleep();
  }
}