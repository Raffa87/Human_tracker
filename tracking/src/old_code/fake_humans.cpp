#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tracking/PoseWithCovarianceStampedArray.h"


std::string node_name;

int main(int argc, char **argv)
{
 ros::init(argc, argv, "fake_humans");

 ros::NodeHandle n;
 ros::Publisher humans_pub = n.advertise<tracking::PoseWithCovarianceStampedArray>("human_tracked", 1000);

 node_name = ros::this_node::getName();

  ros::Duration(10.0).sleep(); // sleep for ten second
  ROS_INFO("fake humans started publishing");
  ros::Time starting_time = ros::Time::now();
  double speed = 1.0; // [m/s]

  ros::Rate loop_rate(10);
  float initial_value = -50.0;
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration interval = now - starting_time;
    float value;
 
    tracking::PoseWithCovarianceStampedArray my_msg;

    geometry_msgs::PoseWithCovarianceStamped my_pose_0;
    my_pose_0.header.stamp = now;
    my_pose_0.header.frame_id = "/map";
    value = initial_value + interval.toSec()*10.0*speed;
    my_pose_0.pose.pose.position.x = value; 
    my_pose_0.pose.pose.position.y = value; 
    my_pose_0.pose.covariance[0] = 0.25; 
    my_pose_0.pose.covariance[7] = 0.25; 
    if (value < 50.0)
    	my_msg.poses.push_back(my_pose_0);
 

    geometry_msgs::PoseWithCovarianceStamped my_pose_1;
    my_pose_1.header.stamp = now;
    my_pose_1.header.frame_id = "/map";
    value = initial_value + interval.toSec()*speed;
    my_pose_1.pose.pose.position.x = value; 
    my_pose_1.pose.pose.position.y = 0.0;
    my_pose_1.pose.covariance[0] = 0.25; 
    my_pose_1.pose.covariance[7] = 0.25; 
    if (value < 50.0)
    	my_msg.poses.push_back(my_pose_1);



    geometry_msgs::PoseWithCovarianceStamped my_pose_2;
    my_pose_2.header.stamp = now;
    my_pose_2.header.frame_id = "/map";
    value = initial_value + interval.toSec()*0.5*speed; 
    my_pose_2.pose.pose.position.x = 0.0; 
    my_pose_2.pose.pose.position.y = value;
    my_pose_2.pose.covariance[0] = 0.25; 
    my_pose_2.pose.covariance[7] = 0.25; 
    if (value < 50.0)
    	my_msg.poses.push_back(my_pose_2);


/*
    geometry_msgs::PoseWithCovarianceStamped my_pose_3;
    my_pose_3.header.stamp = now;
    my_pose_3.header.frame_id = "/map";
    my_pose_3.pose.pose.position.x = value/4.0; 
    my_pose_3.pose.pose.position.y = value/4.0;
    my_pose_3.pose.covariance[0] = 0.25; 
    my_pose_3.pose.covariance[7] = 0.25; 
  
*/
    if (value/2.0 < 100.0)
    	;
    else
	break;


 	
    humans_pub.publish(my_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  ROS_INFO("fake humans publishing completed");

  return 0;
}
