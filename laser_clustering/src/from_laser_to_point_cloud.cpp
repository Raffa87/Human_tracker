#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"


std::string param_space;
std::string sensor_coordinate_frame;
std::string output_coordinate_frame;
std::string sensor_coordinate_frame_default = "/laser";
std::string output_coordinate_frame_default = "/map";

ros::Publisher laser_cloud_publisher;
laser_geometry::LaserProjection projector;
tf::TransformListener *transform_listener;

std::vector<sensor_msgs::LaserScan> my_scans;


double Dist_Between_Points( double x1, double x2, double y1, double y2);


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{


	try
	{	if(!transform_listener->waitForTransform(scan_in->header.frame_id,output_coordinate_frame, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),ros::Duration(1.0)))
     		{
			ROS_ERROR("message dropped");
			return;	
		}
		sensor_msgs::PointCloud cloud;
  		projector.transformLaserScanToPointCloud(output_coordinate_frame,*scan_in, cloud, *transform_listener);
		//masking data too close to robot
	
		geometry_msgs::PoseStamped laser_origin;
        	laser_origin.header.frame_id = sensor_coordinate_frame;
        	laser_origin.header.stamp = scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment);
		laser_origin.pose.position.x = 0.0;
        	laser_origin.pose.position.y = 0.0;
		laser_origin.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);	

                try
                {
                        transform_listener->transformPose(output_coordinate_frame, laser_origin , laser_origin);
                }
                catch (...)
                {
                        ROS_ERROR("unable to find trasformation between %s and %s", output_coordinate_frame.c_str(), sensor_coordinate_frame.c_str());
                        ros::Duration(1.0).sleep();
                }

		int my_size = cloud.points.size();
		for (int i=0; i<my_size; i++)
		{
			double dist = Dist_Between_Points(cloud.points[i].x, laser_origin.pose.position.x, cloud.points[i].y, laser_origin.pose.position.y);
			if (dist < 0.15)
			{
				cloud.points.erase(cloud.points.begin() + i);
				my_size = cloud.points.size();
			}
				 
		}
		//test
		//cloud.points.resize(100);

	
		laser_cloud_publisher.publish(cloud);
	}
	catch(tf2::ConnectivityException)
	{
		ROS_ERROR("message dropped for connectivity exception");
	}
}


double Dist_Between_Points( double x1, double x2, double y1, double y2){
  return ( sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) );
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "from_laser_to_point_cloud");
  ros::NodeHandle n;

  param_space = ros::this_node::getName();

  if (n.getParam(param_space + "/sensor_coordinate_frame", sensor_coordinate_frame))
  {
 	 ROS_INFO("[%s] got (sensor_coordinate_frame) from param server, value is %s", param_space.c_str(), sensor_coordinate_frame.c_str() );
  }
  else
  {	
	sensor_coordinate_frame = sensor_coordinate_frame_default;
	ROS_INFO("[%s] not found (sensor_coordinate_frame) from param server, set to default value %s", param_space.c_str(), sensor_coordinate_frame.c_str() );
  }

  if (n.getParam(param_space + "/output_coordinate_frame", output_coordinate_frame))
  {
         ROS_INFO("[from_laser_point_to_cloud] got (output_coordinate_frame) from param server, value is %s", output_coordinate_frame.c_str() );
  }
  else
  {
        output_coordinate_frame = output_coordinate_frame_default;
        ROS_INFO("[from_laser_point_to_cloud] not found (output_coordinate_frame) from param server, set to default value %s", output_coordinate_frame.c_str() );
  }
  transform_listener = new tf::TransformListener;
  try
  {
	transform_listener->waitForTransform (output_coordinate_frame, sensor_coordinate_frame, ros::Time::now(), ros::Duration(20.0));
  }  
  catch (tf::TransformException ex)
  {
	ROS_ERROR("%s",ex.what());
  }


  laser_cloud_publisher	= n.advertise<sensor_msgs::PointCloud>("/laser_cloud", 1);
  ros::Subscriber laser_sub = n.subscribe("/scan", 10, scanCallback);
  ros::spin();
  
  return 0;
}


