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
#include "tracking/SaveCountingMap.h"
#include "sensor_msgs/Image.h"
#include "tracking/Int32StampedArray.h"


std::string map_frame;
std::string map_frame_default = "/map";
std::string laser_frame; 
std::string laser_frame_default = "/laser";
double max_distance;
double max_distance_default = 0.20;
double num_of_beam_to_skip;
double num_of_beam_to_skip_default = 0;




nav_msgs::OccupancyGrid static_map;
nav_msgs::OccupancyGrid visit_grid;

std::vector<bool> checked_map;
std::vector<bool> current_checked_map;
std::vector<unsigned char> image_zero;
std::vector<unsigned char> image_data;
std::vector<int> zero_count;

ros::Publisher pub_visit_map;
ros::Publisher pub_visit_occupancy_grid; 
  	
int count_save = 0; 

typedef struct{
        unsigned int x;
        unsigned int y;
}cell;

cell cell_end_old;

tf::TransformListener* listener;
int seq_old = 0;

cell find_cell(double x, double y);
int cell_to_int(cell);
cell int_to_cell(int cell_number);
geometry_msgs::Point cell_centroid(cell my_cell);
void sampling_ray(cell ray_origin, cell ray_end);
double Dist_Between_Points( double x1, double x2, double y1, double y2);
bool find_next_point(geometry_msgs::Point origin, geometry_msgs::Point end, double m, double q, double *x_result, double *y_result);
void populate_map_laser(cell my_cell);
void Line( float x1, float y1, float x2, float y2 );


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
	//ROS_INFO("in the callback");
	int seq_now =  msg->header.seq;
	if((seq_now - seq_old)>1)
	{
		ROS_ERROR("I'm to slow!!!");
		ROS_ERROR("diff is %d", seq_now - seq_old);
	}
	seq_old = seq_now;

	std_msgs::Int32 my_value = msg->value.front();
	int value_laser = my_value.data;
	cell cell_laser = int_to_cell(value_laser);
	
	
	
	std::vector<cell> cell_array;
	for (int i=1; i<msg->value.size(); i++)
	{
		cell temp_cell = int_to_cell(msg->value[i].data);
		cell_array.push_back(temp_cell);
	}
	
	current_checked_map = checked_map;
	image_data = image_zero;

	for(int i=1; i<cell_array.size(); i++)
	{
		
		sampling_ray(cell_laser, cell_array[i]);
		i = i + num_of_beam_to_skip;
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
void sampling_ray(cell ray_origin, cell ray_end)
{
	
	//ROS_INFO("[%s] sampling between (%d;%d), (%d;%d)",  ray_origin.x, ray_origin.y, ray_origin., ray_origin.y,
	cell cell_origin = ray_origin;
	cell cell_end =  ray_end;
	//if ((cell_end.x == cell_end_old.x) && (cell_end.y == cell_end_old.y))
	//	return;
	//else
	//{
	//	cell_end_old.x = cell_end.x;
	//	cell_end_old.y = cell_end.y;
	//} 
	
        // Bresenham's line algorithm
	float x1 = (float)cell_origin.x;
	float y1 = (float)cell_origin.y;
	
	float x2 = (float)cell_end.x;
	float y2 = (float)cell_end.y;
	
	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  	if(steep)
  	{
    		std::swap(x1, y1);
		std::swap(x2, y2);
  	}
 
  	if(x1 > x2)
  	{
	    std::swap(x1, x2);
	    std::swap(y1, y2);
  	}
 
	const float dx = x2 - x1;
	const float dy = fabs(y2 - y1);
 
	float error = dx / 2.0f;
	const int ystep = (y1 < y2) ? 1 : -1;
	int y = (int)y1;
 
	const int maxX = (int)x2;
 
	for(int x=(int)x1; x<maxX; x++)
	{
	    	cell temp_cell;
		if(steep)
		{
		        temp_cell.x = y;
			temp_cell.y = x;
			populate_map_laser(temp_cell);			
			//ROS_INFO("point in %d, %d", y , x);//SetPixel(y,x, color);
		}	
		else
		{
			temp_cell.x = x;
			temp_cell.y = y;
			populate_map_laser(temp_cell);			
			
			//ROS_INFO("point in %d, %d", x, y); //SetPixel(x,y, color);
		}
 

		error -= dy;
		if(error < 0)
		{
			y += ystep;
			error += dx;
		}
  	}

}



void populate_map_laser(cell my_cell)
{
	
	bool cell_already_checked = current_checked_map[cell_to_int(my_cell)];
	if(!cell_already_checked)
	{
		int index = cell_to_int(my_cell);
		current_checked_map[index] = true;
		image_data[index] = 1;
		visit_grid.data[index] = image_data[index] * 100;
		std_msgs::Int32 my_value;
		my_value.data = index;
	}
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
int cell_to_int(cell my_cell)
{
	return my_cell.y * static_map.info.width + my_cell.x;
}
cell int_to_cell(int cell_number)
{
        cell my_cell;
        my_cell.y = cell_number / static_map.info.width;
        my_cell.x = cell_number - (my_cell.y * static_map.info.width);
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

double Dist_Between_Points( double x1, double x2, double y1, double y2){
  return ( sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) );
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
	ros::init(argc, argv, "laser_visit_map");
	ros::NodeHandle n;
	listener = new tf::TransformListener();
	
	std::string param_space = ros::this_node::getName();
	get_parameter(param_space, "/map_frame", &map_frame, map_frame_default);
	get_parameter(param_space, "/laser_frame", &laser_frame, laser_frame_default);
	get_parameter(param_space, "/max_distance", &max_distance, max_distance_default);
	get_parameter(param_space, "/num_of_beam_to_skip", &num_of_beam_to_skip, num_of_beam_to_skip_default);


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

void Line( float x1, float y1, float x2, float y2 )
{
        // Bresenham's line algorithm
  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if(steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }
 
  if(x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }
 
  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);
 
  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;
 
  const int maxX = (int)x2;
 
  for(int x=(int)x1; x<maxX; x++)
  {
    if(steep)
    {
        ROS_INFO("point in %d, %d", y , x);//SetPixel(y,x, color);
    }
    else
    {
        ROS_INFO("point in %d, %d", x, y); //SetPixel(x,y, color);
    }
 
    error -= dy;
    if(error < 0)
    {
        y += ystep;
        error += dx;
    }
  }
}
