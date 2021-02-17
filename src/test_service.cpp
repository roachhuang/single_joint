/*
#include "ros/ros.h"
#include "diff_drive/joint_state.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_service"); 

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<diff_drive::joint_state>("read_joint_state");
  diff_drive::joint_state srv;
  // srv.request.a = atoll(argv[1]);
  // srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("pos: %.2f", srv.response.pos);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
*/