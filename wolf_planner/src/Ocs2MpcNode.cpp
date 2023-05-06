#include "legged_controllers/Ocs2Mpc.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wolf_mpc_node");
  ros::NodeHandle nh("wolf_planner");
  //ros::Rate loop_rate(100);

  legged::MpcClass ocs2_mpc;

  ocs2_mpc.init(nh);
  //ocs2_mpc.starting();

  ros::spin();

  //while (ros::ok())
  //{
  //  ocs2_mpc.update();
  //  //ocs2_mpc.retrieveAndPublish();
  //  ros::spinOnce();
  //  loop_rate.sleep();
  //}

  return 0;
}
