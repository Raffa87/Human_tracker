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

std::string map_frame = "/map";
std::string laser_frame = "/laser";

nav_msgs::OccupancyGrid static_map;
std::vector<bool> checked_map;
std::vector<bool> current_checked_map;
std::vector<int> count_map_laser;
std::vector<int> count_map_human;

std::vector<int> temporary_count_map_laser;
std::vector<int> temporary_count_map_human;

std::vector<int> zero_count;

int count_save = 0; 

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
void populate_map_laser(geometry_msgs::Point point);
void populate_map_human(geometry_msgs::PoseWithCovarianceStamped pose);
bool evaluate_cell_human(cell temp_cell, double distance, std::vector<bool> &temp_checked_map);
void save_map();

void mySigintHandler(int sig){
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
 save_map();
 ros::shutdown();
}

bool save_map_service(tracking::SaveCountingMap::Request  &req,
         tracking::SaveCountingMap::Response &res)
{
  save_map();
  return true;
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
			populate_map_laser(my_sampled_ray[j]);
		}
		//i = i+20; 
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
	//sampled_ray.push_back(end_point);
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

	//check also the map
	cell current_cell = find_cell(*x_result, *y_result);
	int num_current_cell = cell_to_int(current_cell);
	if(static_map.data[num_current_cell] > 65 || static_map.data[num_current_cell] == -1)	//occupied
	{
		//ROS_INFO("static_map.data[%d]: %d", num_current_cell, (int)static_map.data[num_current_cell]);
		return false; 
	}
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

void populate_map_laser(geometry_msgs::Point point)
{
	
	cell point_cell = find_cell(point.x, point.y);
	bool cell_already_checked = current_checked_map[cell_to_int(point_cell)];
	if(!cell_already_checked)
	{
		count_map_laser[(cell_to_int(point_cell))] = count_map_laser[(cell_to_int(point_cell))] + 1;
		temporary_count_map_laser[(cell_to_int(point_cell))] = temporary_count_map_laser[(cell_to_int(point_cell))] + 1;
		
		current_checked_map[cell_to_int(point_cell)] = true;
	}
} 

void humanCallback(const tracking::PoseWithCovarianceStampedArray::ConstPtr& msg)
{
        for (int i=0; i<msg->poses.size(); i++)
        {
                populate_map_human(msg->poses[i]);
        }
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
                count_map_human[(cell_to_int(temp_cell))] = count_map_human[(cell_to_int(temp_cell))] + 1;
                temporary_count_map_human[(cell_to_int(temp_cell))] = temporary_count_map_human[(cell_to_int(temp_cell))] + 1;
                
		temp_checked_map[cell_to_int(temp_cell)] = true;
                
		
		int temp_cell_x = temp_cell.x;
		int temp_cell_y = temp_cell.y;
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

void save_map()
{

	ROS_INFO("called save_map");
	
	//global
	std::string density_file, human_file, laser_file;
	if (ros::param::get("/density_file_name", density_file))
	        ROS_INFO("[density_map] got (/density_file_name) from param server, value is %s", density_file.c_str());
	else
	{
        	density_file = "density_global.txt";
	        ROS_INFO("[density_map] not found (/density_file_name) from param server, default value is %s", density_file.c_str());
	}	
        if (ros::param::get("/human_file_name", human_file))
                ROS_INFO("[density_map] got (/human_file_name) from param server, value is %s", human_file.c_str());
        else
	{
                human_file = "human_global.txt";
                ROS_INFO("[density_map] not found (/human_file_name) from param server, default value is %s", human_file.c_str());
	}
        if (ros::param::get("/laser_file_name", laser_file))
                ROS_INFO("[density_map] got (/laser_file_name) from param server, value is %s", laser_file.c_str());
        else
        {
                laser_file = "laser_global.txt";
                ROS_INFO("[density_map] not found (/laser_file_name) from param server, default value is %s", laser_file.c_str());
        }


	std::ofstream density_outfile;
	density_outfile.open(density_file.c_str(), std::ios::out | std::ios::trunc );
	std::ofstream human_outfile;
	human_outfile.open(human_file.c_str(), std::ios::out | std::ios::trunc);
	std::ofstream laser_outfile;
	laser_outfile.open(laser_file.c_str(), std::ios::out | std::ios::trunc);	


  	for (int i=0; i<count_map_laser.size(); i++)
  	{	
        	if(count_map_laser[i]!=0)
        	{
			//ROS_INFO("cell number %d", i);
                	cell temp_cell = int_to_cell(i);
			geometry_msgs::Point temp_centroid = cell_centroid(temp_cell);

                 	float density = (float)count_map_human[i] / (float)count_map_laser[i];
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

	//temporary
	std::stringstream ss;
	ss.str(""); // empty the stringstream
	ss.clear();
	ss << count_save;
	std::string string_count = ss.str();

	std::string density_file_temporary, human_file_temporary, laser_file_temporary;
	if (ros::param::get("/density_file_name", density_file_temporary))
	        ROS_INFO("[density_map] got (/density_file_name_temporary) from param server, value is %s", density_file_temporary.c_str());
	else
	{
        	density_file_temporary = "density_temporary.txt_" + string_count;
	        ROS_INFO("[density_map] not found (/density_file_name_temporary) from param server, default value is %s", density_file_temporary.c_str());
	}	
        if (ros::param::get("/human_file_name", human_file_temporary))
                ROS_INFO("[density_map] got (/human_file_name_temporary) from param server, value is %s", human_file_temporary.c_str());
        else
                human_file_temporary = "human_temporary.txt_" + string_count;
                ROS_INFO("[density_map] not found (/human_file_name_temporary) from param server, default value is %s", human_file_temporary.c_str());

        if (ros::param::get("/laser_file_name", laser_file_temporary))
                ROS_INFO("[density_map] got (/laser_file_name_temporary) from param server, value is %s", laser_file_temporary.c_str());
        else
        {
                laser_file_temporary = "laser_temporary.txt_" + string_count;
                ROS_INFO("[density_map] not found (/laser_file_name_temporary) from param server, default value is %s", laser_file_temporary.c_str());
        }	
	
	count_save = count_save + 1;

	std::ofstream density_temporary_outfile;
	density_temporary_outfile.open(density_file_temporary.c_str(), std::ios::out | std::ios::trunc );
	std::ofstream human_temporary_outfile;
	human_temporary_outfile.open(human_file_temporary.c_str(), std::ios::out | std::ios::trunc);
	std::ofstream laser_temporary_outfile;
	laser_temporary_outfile.open(laser_file_temporary.c_str(), std::ios::out | std::ios::trunc);	


  	for (int i=0; i<temporary_count_map_laser.size(); i++)
  	{	
        	if(temporary_count_map_laser[i]!=0)
        	{
                	cell temp_cell = int_to_cell(i);
			geometry_msgs::Point temp_centroid = cell_centroid(temp_cell);

                 	float density = (float)temporary_count_map_human[i] / (float)temporary_count_map_laser[i];
                 	density_temporary_outfile << i << '\t' << density << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
                 	if(temporary_count_map_human[i]!=0)
                 	{
                        	human_temporary_outfile << i << '\t' << temporary_count_map_human[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;
                 	}
                 	laser_temporary_outfile << i << '\t' << temporary_count_map_laser[i] << '\t' << temp_cell.x << '\t' << temp_cell.y << '\t' << temp_centroid.x << '\t' << temp_centroid.y << std::endl;

        	}
  	}
  	density_temporary_outfile.close();
	human_temporary_outfile.close();
	laser_temporary_outfile.close();

	temporary_count_map_laser = zero_count;
	temporary_count_map_human = zero_count;

	ROS_INFO("map saved");
}

int main(int argc, char **argv)
{
	if (argc>=2)
	{
		std::istringstream ss(argv[1]);
		int x;
		if (!(ss >> x))
   			std::cerr << "Invalid number " << argv[1] << '\n';
		count_save = x;
	}
	printf ("count_save %d \n", count_save);
  	ros::init(argc, argv, "density_map");
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
 		count_map_laser.push_back(1000);
		count_map_human.push_back(0);
		
		temporary_count_map_laser.push_back(1000);
		temporary_count_map_human.push_back(0);	
				
		zero_count.push_back(0);
		
	}
	ros::Subscriber sub_laser = n.subscribe("laser_cloud", 10, cloudCallback);
        ros::Subscriber sub_human = n.subscribe("human_tracked", 10, humanCallback);
  
	ros::ServiceServer service = n.advertiseService("save_counting_maps", save_map_service);
  	ROS_INFO("Ready to save counting maps.");
	
	// Override the default ros sigint handler.
	// This must be set after the first NodeHandle is created.
	signal(SIGINT, mySigintHandler);			
	ros::spin();
}
