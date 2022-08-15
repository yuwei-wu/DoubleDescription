#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/replan_fsm.h>

using namespace opt_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh("~");

  ReplanFSM rebo_replan;

  rebo_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();
  // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  //spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}
