#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tracking/PoseWithCovarianceStampedArray.h"

std::string node_name;

int main(int argc, char **argv)
{
 ros::init(argc, argv, "fake_cluster");

 ros::NodeHandle n;
 ros::Publisher cluster_pub = n.advertise<tracking::PoseWithCovarianceStampedArray>("human_poses", 10);

 node_name = ros::this_node::getName();

  ros::Duration(10.0).sleep(); // sleep for ten second

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
 
    tracking::PoseWithCovarianceStampedArray my_msg;

    geometry_msgs::PoseWithCovarianceStamped my_pose_0;
    my_pose_0.header.stamp = now;
    my_pose_0.header.frame_id = "map";
    my_pose_0.pose.pose.position.x = 0.75; 
    my_pose_0.pose.pose.position.y = 6.00; 
    my_msg.poses.push_back(my_pose_0);
 
	
    cluster_pub.publish(my_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
