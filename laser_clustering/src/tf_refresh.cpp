#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include "tf2_msgs/TFMessage.h"

ros::Publisher tf_publisher;

void tfCallback (const tf2_msgs::TFMessage::ConstPtr& tf_in)
{
	ROS_INFO("received tf2_msgs - check timestamp");
	tf2_msgs::TFMessage new_tf;
	static tf::TransformBroadcaster br;
	ros::Time now = ros::Time::now();
	for (int i=0; i<tf_in->transforms.size(); i++)
	{
		geometry_msgs::Transform my_transform_msg = tf_in->transforms[i].transform;
		//my_transform_msg.header.stamp = now;
		tf::Transform my_transform;
		tf::transformMsgToTF(my_transform_msg, my_transform);
		br.sendTransform(tf::StampedTransform(my_transform, now, tf_in->transforms[i].header.frame_id, tf_in->transforms[i].child_frame_id));
	
	}
	
	//tf_publisher.publish(new_tf); 
		
}



int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "tf_refresh");
  ros::NodeHandle n;

  ros::Subscriber tf_sub = n.subscribe("/tf_old", 10,tfCallback);
  ros::Publisher tf_publisher = n.advertise<tf2_msgs::TFMessage>("/tf", 10);
  ros::spin();
  
  return 0;
}


