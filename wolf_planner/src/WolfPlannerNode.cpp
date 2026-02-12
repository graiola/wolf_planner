#include "wolf_planner/WolfPlannerRos.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wolf_planner_node");

  wolf_planner::WolfPlannerRos planner;

  planner.init();

  ros::spin();

  return 0;
}
