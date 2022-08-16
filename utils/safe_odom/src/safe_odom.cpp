#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Empty.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace message_filters;
/**
 * @brief for vins-fusion safety
 * the odom from vicon: nav_msgs/Odometry
 * /vins_fusion/imu_propagate    : nav_msgs/Odometry
 * 
 */

enum ODOM_MODE
{
    VICON_ODOM = 1,
    VINS_ODOM = 2,
    EMERGENCY_STOP = 3
};
ODOM_MODE _odom_mode;

ros::Publisher _new_odom_pub, _estop_pub;

nav_msgs::Odometry odom_msg;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>
      SyncPolicyViconVins;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyViconVins>> SynchronizerViconVins;
SynchronizerViconVins sync_vicon_vins_;


bool _set_start_pose = false;
Eigen::Vector3d _start_pos;



void viconVinsCallback(const nav_msgs::OdometryConstPtr &vicon_odom,
                       const nav_msgs::OdometryConstPtr &vins_odom)
{   

    if(!_set_start_pose){
      _start_pos << vicon_odom->pose.pose.position.x, 
                   vicon_odom->pose.pose.position.y, 
                   vicon_odom->pose.pose.position.z;
      _set_start_pose = true;
    }



    odom_msg = *vins_odom;
    
    /*   position difference  */
    Eigen::Vector3d vicon_pos(vicon_odom->pose.pose.position.x, 
                              vicon_odom->pose.pose.position.y, 
                              vicon_odom->pose.pose.position.z);

    Eigen::Vector3d vins_pos(vins_odom->pose.pose.position.x, 
                             vins_odom->pose.pose.position.y, 
                             vins_odom->pose.pose.position.z);

    vicon_pos -= _start_pos;

    if ((vicon_pos - vins_pos).norm() > 5.0 ){
     
        ROS_WARN("The vins-fusion pos estimation has been larger than 5 meters, autonomous E-STOP for safety");

        ROS_INFO_STREAM("The vicon pos is "<< vicon_pos);
        ROS_INFO_STREAM("The vins pos is "<< vins_pos);

        std_msgs::Empty estop_cmd;
        _estop_pub.publish(estop_cmd);

    } 

    /*   orientation difference  */
    Eigen::Quaterniond vicon_q = Eigen::Quaterniond(vicon_odom->pose.pose.orientation.w,
                                                    vicon_odom->pose.pose.orientation.x,
                                                    vicon_odom->pose.pose.orientation.y,
                                                    vicon_odom->pose.pose.orientation.z);

    Eigen::Quaterniond vins_q = Eigen::Quaterniond(vins_odom->pose.pose.orientation.w,
                                                   vins_odom->pose.pose.orientation.x,
                                                   vins_odom->pose.pose.orientation.y,
                                                   vins_odom->pose.pose.orientation.z);

    _new_odom_pub.publish(odom_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "safe_odom_node");
    ros::NodeHandle node("~");


    message_filters::Subscriber<nav_msgs::Odometry> vicon_sub(node, "vicon_odom", 200);
    message_filters::Subscriber<nav_msgs::Odometry> vins_sub(node, "vins_odom", 200);

    sync_vicon_vins_.reset(new message_filters::Synchronizer<SyncPolicyViconVins>(
        SyncPolicyViconVins(200), vicon_sub, vins_sub));
    sync_vicon_vins_->registerCallback(boost::bind(&viconVinsCallback, _1, _2));

    _estop_pub = node.advertise<std_msgs::Empty>("estop", 10);
    _new_odom_pub = node.advertise<nav_msgs::Odometry>("odom", 200);


    //ros::Subscriber odom_sub_ =  node_.subscribe<nav_msgs::Odometry>("odom_world", 10, &GridMap::odomCallback, this);

    ros::spin();
    return 0;
}
