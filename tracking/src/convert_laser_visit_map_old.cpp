#include <sstream>
#include "signal.h"
#include <algorithm>    // std::swap
#include "ros/ros.h"
#include "fstream"
#include "tf/tf.h"
#include "tf/transform_listener.h" 
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud.h"
#include "tracking/PoseWithCovarianceStampedArray.h"
#include "sensor_msgs/Image.h"
#include "tracking/Int32StampedArray.h"


nav_msgs::OccupancyGrid static_map;
nav_msgs::OccupancyGrid visit_grid;


std::vector<bool> checked_map;
std::vector<bool> current_checked_map;
std::vector<unsigned char> image_zero;
std::vector<unsigned char> image_data;
std::vector<int> zero_count;

ros::Publisher pub_visit_map;
ros::Publisher pub_visit_occupancy_grid; 
  	

typedef struct{
        unsigned int x;
        unsigned int y;
}cell;


tf::TransformListener* listener;


bool get_parameter(std::string param_space, std::string param_name, std::string* param, std::string param_default);
bool get_parameter(std::string param_space, std::string param_name, double* param, double param_default);
bool get_parameter(std::string param_space, std::string param_name, int* param, int param_default);



void mySigintHandler(int sig){
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
 //save_map();
 ros::shutdown();
}

int map_counter = 0;

void intArrayCallback(const tracking::Int32StampedArray::ConstPtr& msg)
{
    current_checked_map = checked_map;
	image_data = image_zero;
	for (int i=0; i<msg->value.size(); i++)
	{
        int index = msg->value[i].data;
		if (index > current_checked_map.size())
        {
            ROS_ERROR ("index over size - index is %d", index);
            continue;

        }       
        current_checked_map[index] = true;
		image_data[index] = 1;
		visit_grid.data[index] = image_data[index] * 100;
	
	}
	
	//fill image of map
	sensor_msgs::Image visit_map_image;
	visit_map_image.data = image_data;
	
	visit_map_image.header.seq = map_counter;
	visit_map_image.header.stamp = msg->header.stamp; 
	visit_map_image.header.frame_id = "/map";	
	pub_visit_map.publish(visit_map_image);
	map_counter++;
	pub_visit_occupancy_grid.publish(visit_grid); 
 
}



bool get_parameter(std::string param_space, std::string param_name, double* param, double param_default)
{
        std::string node_name = ros::this_node::getName();
        double temp;
        if (ros::param::get(param_space + param_name, temp))
        {
                *param = temp;  
                ROS_INFO("[%s] got %s from param server, value is %f", node_name.c_str(), param_name.c_str(), *param );
                return true;
        }
        else
        {
                *param = param_default;
                ROS_INFO("[%s] not found %s from param server, set to default value %f", node_name.c_str(), param_name.c_str(), *param );
        }
        return false;
}

bool get_parameter(std::string param_space, std::string param_name, int* param, int param_default)
{
        std::string node_name = ros::this_node::getName();
        double temp;
        if (ros::param::get(param_space + param_name, temp))
        {
                *param = temp;  
                ROS_INFO("[%s] got %s from param server, value is %d", node_name.c_str(), param_name.c_str(), *param );
                return true;
        }
        else
        {
                *param = param_default;
                ROS_INFO("[%s] not found %s from param server, set to default value %d", node_name.c_str(), param_name.c_str(), *param );
        }
        return false;
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
	ros::init(argc, argv, "convert_laser_visit_map");
	ros::NodeHandle n;
	listener = new tf::TransformListener();
	
	std::string param_space = ros::this_node::getName();

        if (!ros::service::waitForService("/static_map", ros::Duration(60,0)))//wait for service to get map
        {
                ROS_ERROR("unable to find /static_map service for get the map");
                return -1;
        }
        ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
        nav_msgs::GetMap get_map_srv;
        map_client.call(get_map_srv);
        static_map = get_map_srv.response.map;
	for (int i=0; i<static_map.data.size(); i++)
	{
		checked_map.push_back(false);
		image_zero.push_back(0);		
		
	}
	visit_grid = static_map;
	ros::Subscriber sub_laser = n.subscribe("laser_visit_cell_visited", 10, intArrayCallback);
	pub_visit_map = n.advertise<sensor_msgs::Image>("laser_visit_map", 10);
	pub_visit_occupancy_grid = n.advertise<nav_msgs::OccupancyGrid>("laser_visit_occupancy_grid", 10); 
	
	// Override the default ros sigint handler.
	// This must be set after the first NodeHandle is created.
        signal(SIGINT, mySigintHandler);			
	ros::spin();
}


