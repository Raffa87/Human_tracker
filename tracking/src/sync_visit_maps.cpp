#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <tracking/BoolStampedArray.h>
#include <tracking/SetLaserUsed.h> 

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image, Image> Sync_2_visit;
typedef sync_policies::ApproximateTime<Image, Image, Image, Image> Sync_3_visit;
 

Synchronizer<Sync_2_visit>* sync_2_ptr;
Synchronizer<Sync_3_visit>* sync_3_ptr;

nav_msgs::OccupancyGrid static_map;
nav_msgs::OccupancyGrid visit_grid;
tracking::BoolStampedArray empty_map;

ros::Publisher current_visit_map_pub;
ros::Publisher current_density_map_pub;
ros::Publisher current_affordance_map_pub;
ros::Publisher current_visit_occupancy_grid_pub;


int num_of_laser_used;
int num_of_laser_used_default = 2;

bool set_parameter(std::string param_space, std::string param_name, int value);
void callback(const ImageConstPtr& affordance_map, const ImageConstPtr& visit_map_1, const ImageConstPtr& visit_map_2);
void callback(const ImageConstPtr& affordance_map, const ImageConstPtr& visit_map_1, const ImageConstPtr& visit_map_2, const ImageConstPtr& visit_map_3);


bool change_sync_srv(tracking::SetLaserUsed::Request  &req,
         tracking::SetLaserUsed::Response &res)
{

	set_parameter(ros::this_node::getName(), "/num_of_laser_used", req.num_of_laser_used.data);
	ros::shutdown();
}

void callback(const ImageConstPtr& affordance_map, const ImageConstPtr& visit_map_0, const ImageConstPtr& visit_map_1)
{

	tracking::BoolStampedArray current_visit_map;
	current_visit_map = empty_map;
	tracking::BoolStampedArray current_density_map;
	current_density_map = empty_map;
	tracking::BoolStampedArray current_affordance_map;
	current_affordance_map = empty_map;
	
	for (int i = 0; i < current_visit_map.value.size(); i++)
	{
		current_visit_map.value[i] = (visit_map_0->data[i]||visit_map_1->data[i]);
		current_density_map.value[i] = (current_visit_map.value[i] && affordance_map->data[i]);
		current_affordance_map.value[i] = (affordance_map->data[i]);
		int temp = current_visit_map.value[i] * 100;
		visit_grid.data[i] = temp;

	}
	current_visit_map_pub.publish(current_visit_map);
  	current_density_map_pub.publish(current_density_map);
  	current_affordance_map_pub.publish(current_affordance_map);
	current_visit_occupancy_grid_pub.publish(visit_grid);
	

}

void callback(const ImageConstPtr& affordance_map, const ImageConstPtr& visit_map_0, const ImageConstPtr& visit_map_1, const ImageConstPtr& visit_map_2)
{
  	tracking::BoolStampedArray current_visit_map;
	current_visit_map = empty_map;
	tracking::BoolStampedArray current_density_map;
	current_density_map = empty_map;
	tracking::BoolStampedArray current_affordance_map;
	current_affordance_map = empty_map;

	
		
	for (int i = 0; i < current_visit_map.value.size(); i++)
	{
		current_visit_map.value[i] = (visit_map_0->data[i]||visit_map_1->data[i]||visit_map_2->data[i]);
		current_density_map.value[i] = (current_visit_map.value[i] && affordance_map->data[i]);
		current_affordance_map.value[i] = (affordance_map->data[i]);

		int temp = current_visit_map.value[i] * 100;
		visit_grid.data[i] = temp;

	}
	current_visit_map_pub.publish(current_visit_map);
  	current_density_map_pub.publish(current_density_map);
  	current_affordance_map_pub.publish(current_affordance_map);
	current_visit_occupancy_grid_pub.publish(visit_grid);
	
	//ROS_INFO("[%s] data sync, index are %d %d %d", ros::this_node::getName().c_str(), affordance_map->header.seq, visit_map_0->header.seq, visit_map_1->header.seq);   
}

bool get_parameter(std::string param_space, std::string param_name, int* param, int param_default)
{
        std::string node_name = ros::this_node::getName();
        int temp;
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

bool set_parameter(std::string param_space, std::string param_name, int value)
{
        std::string node_name = ros::this_node::getName();
        ros::param::set(param_space + param_name, value);
        ROS_INFO("[%s] set %s in param server, value is %d", node_name.c_str(), param_name.c_str(), value );
        return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "merge_visit_map");

  ros::NodeHandle nh;
  std::string param_space = ros::this_node::getName();
  
  if (!ros::service::waitForService("/static_map", ros::Duration(60,0)))//wait for service to get map
  {
	ROS_ERROR("unable to find /static_map service for get the map");
	return -1;
  }
  ros::ServiceClient map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap get_map_srv;
  map_client.call(get_map_srv);
  static_map = get_map_srv.response.map;
  for (int i=0; i<static_map.data.size(); i++)
  {
	empty_map.value.push_back(false);
  }
  visit_grid = static_map;

  current_visit_map_pub = nh.advertise<tracking::BoolStampedArray>("/current_visit_map", 10);
  current_density_map_pub = nh.advertise<tracking::BoolStampedArray>("/current_density_map", 10);
  current_affordance_map_pub = nh.advertise<tracking::BoolStampedArray>("/current_affordance_map", 10);

  current_visit_occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/current_visit_occupancy_grid", 10);

  get_parameter(param_space, "/num_of_laser_used", &num_of_laser_used, num_of_laser_used_default);

  message_filters::Subscriber<Image> affordance_map_sub(nh, "affordance_map", 1000);
  message_filters::Subscriber<Image> visit_map0_sub(nh, "laser_visit_map_fixed_0", 1000);
  message_filters::Subscriber<Image> visit_map1_sub(nh, "laser_visit_map_fixed_1", 1000);
  message_filters::Subscriber<Image> visit_map2_sub(nh, "laser_visit_map_robot", 1000);

  switch(num_of_laser_used)
  {
	case 2:
		sync_2_ptr = new Synchronizer<Sync_2_visit>(Sync_2_visit(1000), affordance_map_sub, visit_map0_sub, visit_map1_sub);
		sync_2_ptr->registerCallback(boost::bind(&callback, _1, _2, _3));
		break;
	case 3:
		sync_3_ptr = new Synchronizer<Sync_3_visit>(Sync_3_visit(1000), affordance_map_sub, visit_map0_sub, visit_map1_sub, visit_map2_sub);
		sync_3_ptr->registerCallback(boost::bind(&callback, _1, _2, _3, _4));
		break;
	default:
		ROS_ERROR("[%s] service not implemented for %d laser", ros::this_node::getName().c_str(), num_of_laser_used);	
		break;
  }
	
  
  ros::ServiceServer service = nh.advertiseService(param_space + "/change_num_laser", change_sync_srv);
  ROS_INFO("[%s] Ready to add change number of laser used", ros::this_node::getName().c_str());
  ros::spin(); 
  return 0;
}
