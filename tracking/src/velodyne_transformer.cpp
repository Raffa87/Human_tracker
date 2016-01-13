#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

ros::Publisher velodyne_pub; 
tf::TransformListener* tf_listener;

std::string map_frame;
std::string map_frame_default = "/map";
std::string velodyne_frame;
std::string velodyne_frame_default = "/velodyne";


bool get_parameter(std::string param_space, std::string param_name, std::string* param, std::string param_default);


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	sensor_msgs::PointCloud2 pc2SensorFrame = *msg;
	sensor_msgs::PointCloud  pcSensorFrame;
	sensor_msgs::PointCloud2 pc2MapFrame;
	sensor_msgs::PointCloud pcMapFrame;

   	try{
		ros::Time now = msg->header.stamp;//ros::Time::now();
    		tf_listener->waitForTransform(map_frame, velodyne_frame,
                             now, ros::Duration(0.2));
	
		sensor_msgs::convertPointCloud2ToPointCloud(pc2SensorFrame,pcSensorFrame); 
		tf_listener->transformPointCloud(map_frame, pcSensorFrame, pcMapFrame); 
		sensor_msgs::convertPointCloudToPointCloud2(pcMapFrame,pc2MapFrame);
		//ROS_INFO("I heard: [%s]", msg->data.c_str());
	}	
    	catch (tf::TransformException ex){
     		ROS_ERROR("%s",ex.what());
     	 	ros::Duration(1.0).sleep();
   	 }	

	velodyne_pub.publish(pc2MapFrame);
}

bool get_parameter(std::string param_space, std::string param_name, std::string* param, std::string param_default)
{
        std::string node_name = ros::this_node::getName();
        std::string temp;
        if (ros::param::get(param_space + param_name, temp))
        {
                *param = temp;  
                ROS_INFO("[%s] got %s from param server, value is %s", node_name.c_str(), param_name.c_str(), param->c_str() );
                return true;
        }
        else
        {
                *param = param_default;
                ROS_INFO("[%s] not found %s from param server, set to default value %s", node_name.c_str(), param_name.c_str(), param->c_str() );
        }
        return false;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_transformer");
	ros::NodeHandle n;
        std::string param_space = ros::this_node::getName();
        get_parameter(param_space, "/map_frame", &map_frame, map_frame_default);
        get_parameter(param_space, "/velodyne_frame", &velodyne_frame, velodyne_frame_default);
	tf_listener = new tf::TransformListener();
  	ros::Subscriber velodyne_sub = n.subscribe("velodyne_points", 10, cloudCallback);
	velodyne_pub = n.advertise<sensor_msgs::PointCloud2>("velodyne_cloud", 10);
  	ros::spin();
	return 0;
}
