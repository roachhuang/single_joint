#include <ros/ros.h>
#include <include/my_robot.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "my_robot");
    
    MyRobot::MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
    
    // setup a separate thread that will be used to service ROS callbacks
    ros:AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0); // 10 Hz rate
    
    while (ros::ok())
    {
        const ros::Time     time   = ros:Time::now();
        const ros::Duration period = time - prev_time;
        
        robot.read();
        cm.update(time, period);
        robot.write();        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
