#include "wolf_planner/WolfMpc.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wolf_mpc_node");
  ros::NodeHandle nh("wolf_planner");

  wolf_planner::WolfMpc wolf_mpc;

  wolf_mpc.init(nh);

  ros::spin();

  return 0;
}
