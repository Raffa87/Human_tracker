#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

int main(int argc, char **argv)
{
 ros::init(argc, argv, "fake_publisher");

 ros::NodeHandle n;
 ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("human_pose", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::PoseWithCovarianceStamped my_pose;
    my_pose.header.stamp = ros::Time::now();
    my_pose.header.frame_id = "/map";    
    my_pose.pose.covariance[0] = 0.25; 
    my_pose.pose.covariance[7] = 0.25;   
 	
   chatter_pub.publish(my_pose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
