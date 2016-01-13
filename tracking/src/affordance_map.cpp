#include <sstream>
#include "signal.h"
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


nav_msgs::OccupancyGrid static_map;
std::vector<bool> checked_map;
std::vector<bool> current_checked_map;

std::vector<unsigned char> image_zero;
std::vector<unsigned char> image_data;

sensor_msgs::Image affordance_map;
int map_counter = 0;

double max_distance = 0.25;

typedef struct{
        unsigned int x;
        unsigned int y;
}cell;

tf::TransformListener* listener;
ros::Publisher pub_affordance_map;
 

cell find_cell(double x, double y);
int cell_to_int(cell);
cell int_to_cell(int cell_number);
geometry_msgs::Point cell_centroid(cell my_cell);
std::vector<geometry_msgs::Point> sampling_ray(geometry_msgs::Point ray_origin, geometry_msgs::Point32 ray_end);
double Dist_Between_Points( double x1, double x2, double y1, double y2);
void populate_map_human(geometry_msgs::PoseWithCovarianceStamped pose);
bool evaluate_cell_human(cell temp_cell, double distance, std::vector<bool> &temp_checked_map);


void humanCallback(const tracking::PoseWithCovarianceStampedArray::ConstPtr& msg)
{
	image_data = image_zero;
        for (int i=0; i<msg->poses.size(); i++)
        {
                populate_map_human(msg->poses[i]);
        }
             
        affordance_map.header.seq = map_counter;
        affordance_map.header.stamp = ros::Time::now();        
        affordance_map.header.frame_id = "/map";
	affordance_map.data = image_data;
	pub_affordance_map.publish(affordance_map);
        map_counter++;

}

void populate_map_human(geometry_msgs::PoseWithCovarianceStamped pose)
{
        geometry_msgs::Point my_point = pose.pose.pose.position;
        cell human_cell = find_cell(my_point.x, my_point.y);
        //ROS_INFO("my point is x:%f y:%f", my_point.x, my_point.y);

        std::vector<bool> temp_checked_map = checked_map;

        int search_size = 0;
        bool find_one_occupied_cell = false;
        do{
                search_size = search_size + 1;
                find_one_occupied_cell = false;

                for(int i=0; i<search_size; i++)
                {
			for(int j=0; j<search_size; j++)
                        {
				cell temp_cell;
                                temp_cell.x = human_cell.x + i;
                                temp_cell.y = human_cell.y + j;
                                geometry_msgs::Point my_centroid = cell_centroid(temp_cell);

                                double my_distance = Dist_Between_Points(my_point.x, my_centroid.x, my_point.y, my_centroid.y);
                        
			        find_one_occupied_cell = evaluate_cell_human(temp_cell, my_distance, temp_checked_map);
				if (j!=0)
                                {
                                        int j_minus=-j;
                                        temp_cell.x = human_cell.x + i;
                                        temp_cell.y = human_cell.y + j_minus;
                                        geometry_msgs::Point my_centroid = cell_centroid(temp_cell);

                                        double my_distance = Dist_Between_Points(my_point.x, my_centroid.x, my_point.y, my_centroid.y);
					find_one_occupied_cell = evaluate_cell_human(temp_cell, my_distance, temp_checked_map);
				}

                        }
                        if(i!=0)
                        {
                                int i_minus = -i;
                                for(int j=0; j<search_size; j++)
                                {
                                        cell temp_cell;
                                        temp_cell.x = human_cell.x + i_minus;
                                        temp_cell.y = human_cell.y + j;
                                        geometry_msgs::Point my_centroid = cell_centroid(temp_cell);

                                        double my_distance = Dist_Between_Points(my_point.x, my_centroid.x, my_point.y, my_centroid.y);
                                        find_one_occupied_cell = evaluate_cell_human(temp_cell, my_distance, temp_checked_map);
			                
					if (j!=0)
                                        {
                                                int j_minus=-j;
                                                temp_cell.x = human_cell.x + i_minus;
                                                temp_cell.y = human_cell.y + j_minus;
                                                geometry_msgs::Point my_centroid = cell_centroid(temp_cell);

                                                double my_distance = Dist_Between_Points(my_point.x, my_centroid.x, my_point.y, my_centroid.y);
                                                find_one_occupied_cell = evaluate_cell_human(temp_cell, my_distance, temp_checked_map);
			
					}
                                }
                        }
                }
        }while(find_one_occupied_cell);
}




bool evaluate_cell_human(cell temp_cell, double distance, std::vector<bool> &temp_checked_map)
{
        bool check_distance = distance < max_distance;
        if (!check_distance)
        {
                //ROS_INFO("cell to far");
                return false;
        }
        bool cell_already_checked = temp_checked_map[cell_to_int(temp_cell)];
        if(cell_already_checked)
        {
                //ROS_INFO("cell already checked");
                return false;
        }
        if(check_distance && !cell_already_checked)
        {
		int index = cell_to_int(temp_cell);
		image_data[index] = 1;                
		temp_checked_map[index] = true;
                
//		int temp_cell_x = temp_cell.x;
//		int temp_cell_y = temp_cell.y;
//		ROS_INFO("check %d %d", temp_cell.x, temp_cell.y);

		return true;
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


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "affordance_map");
	ros::NodeHandle n;
	listener = new tf::TransformListener();

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
        ros::Subscriber sub_human = n.subscribe("human_tracked", 10, humanCallback);
        pub_affordance_map = n.advertise<sensor_msgs::Image>("affordance_map", 10);
 
	
	ros::spin();
}
