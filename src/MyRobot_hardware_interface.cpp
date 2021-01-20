#include "inlcude/MyRobt_hardware_interface.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "husky_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Initialize robot hardware and link to controller manager
  husky_base::HuskyHardware husky(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&husky, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue husky_queue;
  ros::AsyncSpinner husky_spinner(1, &husky_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(husky), boost::ref(cm), boost::ref(last_time)),
    &husky_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions diagnostic_timer(
    ros::Duration(1 / diagnostic_frequency),
    boost::bind(diagnosticLoop, boost::ref(husky)),
    &husky_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  husky_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
