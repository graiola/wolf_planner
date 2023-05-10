#include "wolf_planner/WolfMpc.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wolf_mpc_node");

  wolf_planner::WolfMpc wolf_mpc;

  wolf_mpc.init();

  ros::spin();

  return 0;
}
