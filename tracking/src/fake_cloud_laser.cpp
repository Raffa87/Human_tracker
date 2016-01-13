#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/PointCloud.h"

int main(int argc, char **argv)
{
 ros::init(argc, argv, "fake_cloud_laser");

 ros::NodeHandle n;
 ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("laser_cloud_fake", 10);

  geometry_msgs::Point32 center;
  center.x = 1.0;
  center.y = 6.0;
  center.z = 0.0;	

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    


    sensor_msgs::PointCloud my_cloud;
    for(int i = 0; i < 40; i++)
    {
	geometry_msgs::Point32 point;
	point.x = center.x;
	point.y = center.y + i*0.025;
	my_cloud.points.push_back(point);		
    } 
    for(int i = 0; i < 40; i++)
    {
	geometry_msgs::Point32 point;
	point.x = center.x;
	point.y = center.y - i*0.025;
	my_cloud.points.push_back(point);		
    } 
 	
    my_cloud.header.seq = count;
    count++;
    my_cloud.header.stamp = ros::Time::now();
    my_cloud.header.frame_id = "map";  //(laser_2)
    cloud_pub.publish(my_cloud);
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
