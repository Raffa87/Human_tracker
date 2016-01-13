#include <sstream>
#include <vector>
#include <fstream>

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "tracking/BoolStampedArray.h"

#include "tracking/EmptyService.h"


typedef struct{
        unsigned int x;
        unsigned int y;
}cell;

nav_msgs::OccupancyGrid static_map;

std::vector<int> density_map_num;
std::vector<int> density_map_den;
std::vector<int> affordance_map;
std::vector<int> visit_map;
std::vector<int> zero_map;



ros::Publisher publisher_map;
bool get_parameter(std::string param_space, std::string param_name, std::string* param, std::string param_default);

cell int_to_cell(int cell_number);
int cell_to_int(cell my_cell);
cell find_cell(double x, double y);
geometry_msgs::Point cell_centroid(cell my_cell);
void save_map();


bool save_map_service(tracking::EmptyService::Request  &req,
         tracking::EmptyService::Response &res)
{
  save_map();
  return true;
}

bool reset_map_service(tracking::EmptyService::Request  &req,
         tracking::EmptyService::Response &res)
{
	ROS_INFO("[%s] reset counting map ", ros::this_node::getName().c_str());
	density_map_num = zero_map;
	density_map_den = zero_map;
	affordance_map = zero_map;
	visit_map = zero_map;
	return true;
}



void laser_visit_callback(const tracking::BoolStampedArrayConstPtr& current_visit_map)
{

	for(int i=0; i<current_visit_map->value.size(); i++)
	{
		if (current_visit_map->value[i])
		{
	
			visit_map[i] = visit_map[i]+1;
			density_map_den[i] = density_map_den[i]+1;
			//ROS_INFO("density_map_den is %d", density_map_den[i]);
		}
	}
}
void affordance_callback(const tracking::BoolStampedArrayConstPtr& current_affordance_map)
{
	for(int i=0; i<current_affordance_map->value.size(); i++)
	{
		if (current_affordance_map->value[i])
		{
			affordance_map[i] = affordance_map[i]+1;
		}
	}

}
void density_callback(const tracking::BoolStampedArrayConstPtr& current_density_map)
{
	for(int i=0; i<current_density_map->value.size(); i++)
	{
		density_map_num[i] = density_map_num[i]+1;
	}

}

void save_map()
{

        ROS_INFO("called save_map");

	std::string param_space = ros::this_node::getName();
        //global
        std::string affordance_file, density_file, human_file, laser_file;

        get_parameter(param_space, "/affordance_file_name", &affordance_file, "affordance_map.txt");
        get_parameter(param_space, "/density_file_name", &density_file, "density_map.txt");
        get_parameter(param_space, "/human_file_name", &human_file, "human_map.txt");
        get_parameter(param_space, "/laser_file_name", &laser_file, "laser_map.txt");
        

        std::ofstream density_outfile;
        density_outfile.open(density_file.c_str(), std::ios::out | std::ios::trunc );
        std::ofstream human_outfile;
        human_outfile.open(human_file.c_str(), std::ios::out | std::ios::trunc);
        std::ofstream laser_outfile;
        laser_outfile.open(laser_file.c_str(), std::ios::out | std::ios::trunc);


        for (int i=0; i<density_map_den.size(); i++)
        {
                if(static_map.data[i] == -1)
			continue;
		if(density_map_den[i]!=0)
                {
		
			cell temp_cell = int_to_cell(i);
                        geometry_msgs::Point temp_centroid = cell_centroid(temp_cell);

                        float density = (float)affordance_map[i] / (float)density_map_den[i];
			//ROS_INFO("[%s] num %d den %d", ros::this_node::getName().c_str(), affordance_map[i], density_map_den[i]);
                        if (density_map_den[i] < 10)
				continue;
			density_outfile << i << '\t' << density << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
			if (density_map_num[i]!=0)
			{
 	                 	human_outfile << i << '\t' << density_map_num[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
		
			}
			laser_outfile << i << '\t' << visit_map[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
			
			
                }
        }
        density_outfile.close();
        human_outfile.close();
        laser_outfile.close();

	std::ofstream affordance_outfile;
        affordance_outfile.open(affordance_file.c_str(), std::ios::out | std::ios::trunc );
	for(int i=0; i<affordance_map.size(); i++)
	{
		if(affordance_map[i]!=0)
        	{
			cell temp_cell = int_to_cell(i);
                        geometry_msgs::Point temp_centroid = cell_centroid(temp_cell);

                        affordance_outfile << i << '\t' << affordance_map[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
			
		} 
	}
 

}
cell int_to_cell(int cell_number)
{
        cell my_cell;
	my_cell.y = cell_number / static_map.info.width;
	my_cell.x = cell_number - (my_cell.y * static_map.info.width);
	return my_cell;
}
int cell_to_int(cell my_cell)
{
        return my_cell.y * static_map.info.width + my_cell.x;
}

cell find_cell(double x, double y)
{
        int grid_x = (unsigned int)((x - static_map.info.origin.position.x)/static_map.info.resolution);
        int grid_y = (unsigned int)((y - static_map.info.origin.position.y)/static_map.info.resolution);
        cell my_cell;
        my_cell.x = grid_x;
        my_cell.y = grid_y;
        return my_cell;
}

geometry_msgs::Point cell_centroid(cell my_cell)
{
        double x = (double)((int)my_cell.x * static_map.info.resolution) + static_map.info.origin.position.x;
        double y = (double)((int)my_cell.y * static_map.info.resolution) + static_map.info.origin.position.y;

        geometry_msgs::Point my_point;
        my_point.x = x;
        my_point.y = y;
        return my_point;
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

  ros::init(argc, argv, "build_map");
  ros::NodeHandle n;
	
  if (!ros::service::waitForService("/static_map", ros::Duration(60,0)))//wait for service to get map
  {
  	ROS_ERROR("unable to find /static_map service for get the map");
        return -1;
  }
  ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap get_map_srv;
  map_client.call(get_map_srv);
  static_map = get_map_srv.response.map;

  int my_size = static_map.info.width * static_map.info.height;
  for (int i=0; i<my_size; i++)
  {
        if(static_map.data[i] > 65 || static_map.data[i] == -1)   //occupied
		zero_map.push_back(-1);
	else
		zero_map.push_back(0);
  }
  density_map_num = zero_map;
  density_map_den = zero_map;
  affordance_map = zero_map;
  visit_map = zero_map;

  ros::Subscriber laser_visit_sub = n.subscribe("/current_visit_map", 10, laser_visit_callback);
  ros::Subscriber density_sub = n.subscribe("/current_density_map", 10, density_callback);
  ros::Subscriber affordance_sub = n.subscribe("/current_affordance_map", 10, affordance_callback);

  ros::ServiceServer save_service = n.advertiseService("/save_counting_maps", save_map_service);
  ros::ServiceServer reset_service = n.advertiseService("/reset_counting_maps", reset_map_service);
 
  ROS_INFO("Ready to save counting maps.");


  ros::spin(); 

  return 0;  

}
