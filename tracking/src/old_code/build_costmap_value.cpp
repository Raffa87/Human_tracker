#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <algorithm> // for std::copy

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"


typedef struct{
        unsigned int x;
        unsigned int y;
}cell;

nav_msgs::OccupancyGrid static_map;
nav_msgs::OccupancyGrid cost_map;

std::vector<double> density_map;
ros::Publisher publisher_map;




cell int_to_cell(int cell_number)
{
        cell my_cell;
	my_cell.y = cell_number / 4000;
	my_cell.x = cell_number - (my_cell.y * 4000);
	return my_cell;
}
int cell_to_int(cell my_cell)
{
        return my_cell.y * 4000 + my_cell.x;
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

void build_costmap()
{
	//change range
	double max_value = *std::max_element(density_map.begin(),density_map.end());
	double min_value = 0; //*std::min_element(density_map.begin(),density_map.end());
	double max_range = 253; //254;
	double min_range = 0;
	for (int i=0; i<density_map.size(); i++)
	{
		if (density_map[i]!=-1)
		{
			cost_map.data.push_back((uint)((max_range - min_range)*(density_map[i] - min_value))/(max_value - min_value) + min_range);
		}
		else
		{
			cost_map.data.push_back(-1);
		}
	}
	ROS_INFO("size is %d", (int)cost_map.data.size()); 
}

void publish_map()
{
	cost_map.info = static_map.info;
	publisher_map.publish(cost_map);
	ROS_INFO("published");
}

void load_data(std::string file_to_load, std::vector<double> &count_map)
{
  std::ifstream is(file_to_load.c_str());
  std::istream_iterator<double> start(is), end;
  std::vector<double> numbers(start, end); 


  
  std::vector<int> index;
  std::vector<int> value;
  for(int i=0; i<numbers.size(); i++)
  {
	//printf("%f \t %f \t %f \t %f \t %f \t %f \n", numbers[i], numbers[i+1], numbers[i+2], numbers[i+3], numbers[i+4], numbers[i+5]);
	index.push_back((int)numbers[i]);
	value.push_back((int)numbers[i+1]);
	i = i+5;
  }
  for (int i=0; i<index.size(); i++)
  {
	count_map[index[i]] = value[i];
  }
  return; 
}

int main(int argc, char **argv)
{

  std::string file_density_to_load;
	
  if (argc >=2)
  {
	file_density_to_load = argv[1];
  }
  else
  {
	ROS_ERROR("i need the name of files to load");
	ROS_ERROR("rosrun ... build_costmap_value density.txt");
	return -1;
  }
  ros::init(argc, argv, "build_costmap_value");
  ros::NodeHandle n;
	
  publisher_map = n.advertise<nav_msgs::OccupancyGrid>("/affordance_map", 10);

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
	density_map.push_back(-1);
  }

  load_data(file_density_to_load, density_map);
  build_costmap();

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
	publish_map();
	ros::spinOnce();
	sleep(1);
	//loop_rate.sleep();
  }
  return 0;  

}
