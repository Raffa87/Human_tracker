#include "signal.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h" 
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud.h"


std::string map_frame = "/map";
std::string laser_frame = "/laser";

nav_msgs::OccupancyGrid static_map;
std::vector<bool> checked_map;
std::vector<bool> current_checked_map;
std::vector<int> count_map;
double max_distance = 0.25;

typedef struct{
        unsigned int x;
        unsigned int y;
}cell;

tf::TransformListener* listener;
int seq_old = 0;

cell find_cell(double x, double y);
int cell_to_int(cell);
cell int_to_cell(int cell_number);
geometry_msgs::Point cell_centroid(cell my_cell);
std::vector<geometry_msgs::Point> sampling_ray(geometry_msgs::Point ray_origin, geometry_msgs::Point32 ray_end);
double Dist_Between_Points( double x1, double x2, double y1, double y2);
bool find_next_point(geometry_msgs::Point origin, geometry_msgs::Point end, double m, double q, double *x_result, double *y_result);
void populate_map(geometry_msgs::Point point);


void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  for (int i=0; i<count_map.size(); i++)
  {
	if(count_map[i]!=0)
	{
		 cell temp_cell = int_to_cell(i);
		 geometry_msgs::Point temp_centroid = cell_centroid(temp_cell);
        	 printf("%d\t%d\t%d\t%d\t%f\t%f\t\n", i, count_map[i], temp_cell.x, temp_cell.y, temp_centroid.x, temp_centroid.y);
	}
  }
  ros::shutdown();
}


void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	//ROS_INFO("in the callback");
	int seq_now =  msg->header.seq;
	if((seq_now - seq_old)>1)
	{
		ROS_ERROR("I'm to slow!!!");
		ROS_ERROR("diff is %d", seq_now - seq_old);
	}
	seq_old = seq_now;
	current_checked_map = checked_map;
	geometry_msgs::PoseStamped laser_origin;
	laser_origin.header.frame_id = laser_frame;
	laser_origin.pose.position.x = 0.0;
	laser_origin.pose.position.y = 0.0;
	laser_origin.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);	

	//test
	try
	{
    		ros::Time time_stamp = msg->header.stamp;
    		listener->waitForTransform(map_frame, laser_frame, time_stamp, ros::Duration(3.0));
		listener->transformPose(map_frame, laser_origin, laser_origin);	
	}
 	catch (tf::TransformException ex)
	{
  	       ROS_ERROR("%s",ex.what());
	       ros::Duration(1.0).sleep();
	       ROS_INFO("message dropped");
		return;
	}

	

	for(int i=0; i<msg->points.size(); i++)
	{
		std::vector<geometry_msgs::Point> my_sampled_ray = sampling_ray(laser_origin.pose.position, msg->points[i]);
		if(my_sampled_ray.size()==0)
		{
			ROS_INFO("my sampled ray is empty");
			ROS_INFO("origin is: %f, %f - my_point is %f, %f", laser_origin.pose.position.x, laser_origin.pose.position.y, msg->points[i].x, msg->points[i].y);
		}
			
		//for (int j=0; j<my_sampled_ray.size(); j++)
		//{
                //        ROS_INFO("value %f %f", my_sampled_ray[j].x, my_sampled_ray[j].y);
		//}
		for (int j=0; j<my_sampled_ray.size(); j++)
		{
			populate_map(my_sampled_ray[j]);
		}
		i = i+20; 
	}
	
}
std::vector<geometry_msgs::Point> sampling_ray(geometry_msgs::Point ray_origin, geometry_msgs::Point32 ray_end)
{
	//rect 
	double x_1, x_2, y_1, y_2;
	x_1 = ray_end.x;
	x_2 = ray_origin.x;
	y_1 = ray_end.y;
	y_2 = ray_origin.y;	

	double m = (y_2 - y_1)/(x_2 - x_1);	
	double q = -x_1*m + y_1;

	//sampling
	std::vector<geometry_msgs::Point>sampled_ray;
	bool keep_sampling = true;
	geometry_msgs::Point current_point = ray_origin;
	geometry_msgs::Point end_point;
	end_point.x = ray_end.x;
	end_point.y = ray_end.y;
	sampled_ray.push_back(ray_origin);
	while (keep_sampling)
	{
		double x_result, y_result;
		bool next_point_founded = find_next_point(current_point, end_point, m, q, &x_result, &y_result);
		if (next_point_founded)
		{
			keep_sampling = true;
			geometry_msgs::Point my_point;
			my_point.x = x_result;
			my_point.y = y_result;
			sampled_ray.push_back(my_point);
			current_point = my_point;
		}
		else
			keep_sampling = false; 	
	}
	sampled_ray.push_back(end_point);
	return sampled_ray;
}

bool find_next_point(geometry_msgs::Point origin, geometry_msgs::Point end, double m, double q, double *x_result, double *y_result)
{
	std::vector<double> my_vector;
	my_vector.push_back(end.x - origin.x);
	my_vector.push_back(end.y - origin.y);
	double norm = sqrt(pow(my_vector[0],2) + pow(my_vector[1],2));
	my_vector[0] = my_vector[0]/norm * max_distance;
        my_vector[1] = my_vector[1]/norm * max_distance;
	*x_result = origin.x + my_vector[0];
	*y_result = origin.y + my_vector[1];


	bool x_ok, y_ok;

        if (origin.x < end.x)
                x_ok = (origin.x <= *x_result) && (*x_result <= end.x);
        else
                x_ok = (end.x <= *x_result) && (*x_result <= origin.x);
        
        if (origin.y < end.y)
                y_ok = (origin.y <= *y_result) && (*y_result <= end.y);
        else
                y_ok = (end.y <= *y_result) && (*y_result <= origin.y);
        
	if(x_ok && y_ok)
                return true;            
	else
		return false;
}

void populate_map(geometry_msgs::Point point)
{
	
	cell point_cell = find_cell(point.x, point.y);
	bool cell_already_checked = current_checked_map[cell_to_int(point_cell)];
	if(!cell_already_checked)
	{
		count_map[(cell_to_int(point_cell))] = count_map[(cell_to_int(point_cell))] + 1;
		current_checked_map[cell_to_int(point_cell)] = true;
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
  	ros::init(argc, argv, "visited_map");
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
 		count_map.push_back(0);
	}
	ros::Subscriber sub = n.subscribe("laser_cloud", 10, cloudCallback);

	// Override the default ros sigint handler.
	// This must be set after the first NodeHandle is created.
	signal(SIGINT, mySigintHandler);			
	ros::spin();
}
