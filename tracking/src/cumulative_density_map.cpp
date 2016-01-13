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
std::vector<int> count_map_laser;
std::vector<int> count_map_human;



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

void save_map()
{
        std::ofstream density_outfile;
        density_outfile.open("density_total.txt", std::ios::out | std::ios::trunc );
        std::ofstream human_outfile;
        human_outfile.open("human_total.txt", std::ios::out | std::ios::trunc);
        std::ofstream laser_outfile;
        laser_outfile.open("laser_total.txt", std::ios::out | std::ios::trunc);


        for (int i=0; i<count_map_laser.size(); i++)
        {
                if(count_map_laser[i]!=0)
                {
		
                        cell temp_cell = int_to_cell(i);
			geometry_msgs::Point temp_centroid = cell_centroid(temp_cell);

                        float density = (float)count_map_human[i] / (float)count_map_laser[i];
			//this is a bag
			if (density > 1)
				density = 0;
                        density_outfile << i << '\t' << density << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
                        if(count_map_human[i]!=0)
                        {
                                human_outfile << i << '\t' << count_map_human[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
                        }
                        laser_outfile << i << '\t' << count_map_laser[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;

                }
        }

        density_outfile.close();
        human_outfile.close();
        laser_outfile.close();
}

void load_data(std::string file_to_load, std::vector<int> &count_map)
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

  std::string file_laser_to_load;
  std::string file_human_to_load;
	
  if (argc >=3)
  {
	file_laser_to_load = argv[1];
	file_human_to_load = argv[2];
  }
  else
  {
	ROS_ERROR("i need the name of files to load");
	ROS_ERROR("rosrun ... cumulative_density_map laser.txt human.txt");
	return -1;
  }
  ros::init(argc, argv, "cumulative_density_map");
  ros::NodeHandle n;

  //if (!ros::service::waitForService("/static_map", ros::Duration(60,0)))//wait for service to get map
  //{
  //	ROS_ERROR("unable to find /static_map service for get the map");
  //      return -1;
  //}
  //ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
  //nav_msgs::GetMap get_map_srv;
  //map_client.call(get_map_srv);
  //static_map = get_map_srv.response.map;

  int my_size = 4000 * 4000;
  for (int i=0; i<my_size; i++)
  {
	count_map_laser.push_back(0);
	count_map_human.push_back(0);
  }

  load_data("laser_total.txt", count_map_laser);
  load_data("human_total.txt", count_map_human);
  

  load_data(file_laser_to_load, count_map_laser);
  load_data(file_human_to_load, count_map_human);

  save_map();
  return 0;  

}
